#!/usr/bin/env python3
"""
Network conditioner — dt_teleop_testbed.

Applies `tc netem` qdisc configurations to an egress network interface on the
host, so the twin→robot command path experiences controlled, reproducible
network impairment. The node subscribes to /network_profile_request and on
each new profile name, runs the corresponding `tc` command. It publishes the
currently-applied profile on /network_profile (transient_local, latched) so
that late-joining nodes such as the AoI logger pick up the current condition.

Why a separate topic for request vs status:
    The request topic is "what the experimenter asked for."
    The status topic is "what tc actually applied."
    These can diverge if tc returns an error (e.g., insufficient privileges,
    interface missing, invalid profile). The logger tags CSV rows with the
    actually-applied profile — which is what matters for experimental validity.

Privileges:
    `tc` requires CAP_NET_ADMIN. Two options:
    (A) Passwordless sudo for tc:
            In /etc/sudoers.d/tc-netem (owned by root, 0440):
                <you> ALL=(ALL) NOPASSWD: /usr/sbin/tc
            Then run the node with default `use_sudo:=true`.
    (B) Capability on the tc binary itself:
            sudo setcap cap_net_admin+ep $(which tc)
            Then run the node with `use_sudo:=false`.
    Option B is cleaner — it does not affect any other command. Prefer it.

Topics:
    Sub:  /network_profile_request   std_msgs/String
    Pub:  /network_profile           std_msgs/String (transient_local, latched)

Parameters:
    interface       (required)   Interface name, e.g., "enx0050b6...".
    profiles_file   (required)   Path to YAML with netem profile definitions.
    use_sudo        (bool=True)  Prepend `sudo -n` to tc calls.
    apply_on_startup (str)       Profile to apply at launch (default 'ideal').
    clear_on_shutdown (bool=True) Clear the qdisc when the node exits.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy,
)
from std_msgs.msg import String


class NetworkConditioner(Node):
    """Apply tc netem qdisc configurations based on profile requests."""

    def __init__(self) -> None:
        super().__init__("network_conditioner")

        # ---------------- Parameters ---------------- #
        self.declare_parameter("interface", "")
        self.declare_parameter("profiles_file", "")
        self.declare_parameter("use_sudo", True)
        self.declare_parameter(
            "profile_request_topic", "/network_profile_request")
        self.declare_parameter("profile_status_topic", "/network_profile")
        self.declare_parameter("apply_on_startup", "ideal")
        self.declare_parameter("clear_on_shutdown", True)

        self.iface: str = self.get_parameter("interface").value
        if not self.iface:
            raise RuntimeError(
                "Parameter 'interface' must be set (e.g., 'enx00e04c680123'). "
                f"Available: {self._list_interfaces()}")

        iface_path = Path(f"/sys/class/net/{self.iface}")
        if not iface_path.exists():
            raise RuntimeError(
                f"Interface '{self.iface}' does not exist.\n"
                f"Available: {self._list_interfaces()}")

        profiles_file: str = self.get_parameter("profiles_file").value
        if not profiles_file:
            raise RuntimeError("Parameter 'profiles_file' must be set")
        self._profiles = self._load_profiles(profiles_file)

        self._tc_bin = shutil.which("tc")
        if not self._tc_bin:
            raise RuntimeError(
                "'tc' binary not found. Install iproute2: sudo apt install iproute2")

        self._use_sudo: bool = bool(self.get_parameter("use_sudo").value)
        self._clear_on_shutdown: bool = bool(
            self.get_parameter("clear_on_shutdown").value)

        # ---------------- State ---------------- #
        self._current: str = "unknown"

        # ---------------- ROS ---------------- #
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        req_topic: str = self.get_parameter("profile_request_topic").value
        stat_topic: str = self.get_parameter("profile_status_topic").value

        self.sub = self.create_subscription(
            String, req_topic, self._on_request, 10)
        self.pub_status = self.create_publisher(String, stat_topic, latched)

        self.get_logger().info(
            f"network_conditioner: iface='{self.iface}' "
            f"profiles={list(self._profiles.keys())} "
            f"use_sudo={self._use_sudo}\n"
            f"  request_topic={req_topic}\n"
            f"  status_topic ={stat_topic}")

        # Apply startup profile (also clears any stale qdisc from a prior run)
        startup: str = self.get_parameter("apply_on_startup").value
        self._apply(startup)

    # =========================================================== #
    # Profile loading
    # =========================================================== #

    @staticmethod
    def _load_profiles(path: str) -> Dict[str, dict]:
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        profiles = data.get("profiles", {})
        if not isinstance(profiles, dict):
            raise RuntimeError("profiles YAML must contain a 'profiles:' map")
        return profiles

    @staticmethod
    def _list_interfaces() -> List[str]:
        try:
            return sorted(os.listdir("/sys/class/net"))
        except Exception:
            return []

    # =========================================================== #
    # tc wrapper
    # =========================================================== #

    def _run_tc(self, args: List[str], tolerate_fail: bool = False) -> bool:
        cmd: List[str] = []
        if self._use_sudo:
            cmd += ["sudo", "-n"]
        cmd += [self._tc_bin] + args
        self.get_logger().debug(f"$ {' '.join(cmd)}")
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                f"tc command timed out — is sudo asking for a password? "
                f"Command: {' '.join(cmd)}")
            return False
        except Exception as e:
            self.get_logger().error(f"tc command error: {e}")
            return False

        if result.returncode != 0:
            err = result.stderr.strip() or result.stdout.strip()
            if tolerate_fail:
                self.get_logger().debug(
                    f"tc returned {result.returncode} (tolerated): {err}")
                return False
            self.get_logger().error(
                f"tc {args} failed (rc={result.returncode}): {err}")
            return False
        return True

    def _clear_qdisc(self) -> None:
        """Delete any root qdisc on the interface. Safe if none exists."""
        self._run_tc(["qdisc", "del", "dev", self.iface, "root"],
                     tolerate_fail=True)

    def _build_netem_args(self, profile: dict) -> List[str]:
        """Translate a profile dict into tc netem arguments.

        YAML schema:
            delay_ms: <float>          mean one-way delay
            jitter_ms: <float>         +/- jitter (pareto/normal by default)
            delay_correlation_pct: <float>   optional
            loss_pct: <float>
            duplicate_pct: <float>
            corrupt_pct: <float>
            reorder_pct: <float>
            rate_mbit: <float>         optional bandwidth cap
        """
        args: List[str] = []

        if "delay_ms" in profile:
            args += ["delay", f"{float(profile['delay_ms'])}ms"]
            if profile.get("jitter_ms"):
                args.append(f"{float(profile['jitter_ms'])}ms")
                if profile.get("delay_correlation_pct") is not None:
                    args.append(f"{float(profile['delay_correlation_pct'])}%")

        if profile.get("loss_pct") is not None:
            args += ["loss", f"{float(profile['loss_pct'])}%"]

        if profile.get("duplicate_pct") is not None:
            args += ["duplicate", f"{float(profile['duplicate_pct'])}%"]

        if profile.get("corrupt_pct") is not None:
            args += ["corrupt", f"{float(profile['corrupt_pct'])}%"]

        if profile.get("reorder_pct") is not None:
            args += ["reorder", f"{float(profile['reorder_pct'])}%"]

        # Rate limiting is applied via tbf further down the qdisc chain;
        # netem alone doesn't handle bandwidth. For first version, skip rate.
        # (If rate_mbit present, log a warning.)
        if "rate_mbit" in profile:
            self.get_logger().warn(
                "rate_mbit is set but not yet supported — "
                "use tbf+netem composition for bandwidth limiting.")

        return args

    def _apply(self, profile_name: str) -> None:
        if profile_name not in self._profiles:
            self.get_logger().warn(
                f"Unknown profile '{profile_name}'. Available: "
                f"{list(self._profiles.keys())}")
            return

        profile = self._profiles[profile_name] or {}
        netem_args = self._build_netem_args(profile)

        if not netem_args:
            # No impairment (e.g., 'ideal') — just clear qdisc
            self._clear_qdisc()
            self._set_current(profile_name)
            self.get_logger().info(
                f"Applied '{profile_name}' on {self.iface}: (no impairment)")
            return

        cmd = ["qdisc", "replace", "dev", self.iface, "root", "netem"] + netem_args
        if self._run_tc(cmd):
            self._set_current(profile_name)
            self.get_logger().info(
                f"Applied '{profile_name}' on {self.iface}: {' '.join(netem_args)}")
        else:
            self.get_logger().error(
                f"Failed to apply '{profile_name}' — qdisc state unchanged")

    def _set_current(self, name: str) -> None:
        self._current = name
        msg = String()
        msg.data = name
        self.pub_status.publish(msg)

    # =========================================================== #
    # Subscription handler
    # =========================================================== #

    def _on_request(self, msg: String) -> None:
        requested = (msg.data or "").strip()
        if not requested:
            self.get_logger().warn("Empty profile request — ignoring")
            return
        if requested == self._current:
            self.get_logger().debug(
                f"Profile '{requested}' already active — no-op")
            return
        self.get_logger().info(
            f"Profile change requested: '{self._current}' -> '{requested}'")
        self._apply(requested)

    # =========================================================== #
    # Shutdown
    # =========================================================== #

    def destroy_node(self) -> bool:
        if self._clear_on_shutdown:
            self.get_logger().info(f"Clearing qdisc on {self.iface}")
            self._clear_qdisc()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = NetworkConditioner()
    except RuntimeError as e:
        print(f"[network_conditioner] FATAL: {e}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
