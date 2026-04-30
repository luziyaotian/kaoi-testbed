#!/usr/bin/env python3
"""
AoI + sync error logger — dt_teleop_testbed.

The experimental instrument of the testbed. Subscribes to the twin and real
robot joint-state topics (and a network-profile topic so each row is tagged
with its experimental condition), and writes a CSV row at a fixed rate with
everything needed for post-hoc analysis of synchronisation degradation.

Why record raw state rather than pre-computed metrics:
    The paper's core analysis involves computing sync-error-as-a-function-of-
    lag, finding best-fit delays, and comparing to theoretical predictions.
    Those computations are cleaner done offline in pandas/NumPy with the full
    dataset in memory. This node's job is to log a clean, complete record —
    not to prejudge what analysis will be done.

Recorded columns:
    wall_time_s         Wall clock at log tick, high precision
    ros_time_s          ROS time at log tick
    network_profile     Currently-applied tc netem profile
    twin_stamp_s        header.stamp of most recent twin message
    real_stamp_s        header.stamp of most recent real message
    aoi_s               wall_time_s - twin_stamp_s  (Age of Information)
    msg_dt_s            real_stamp_s - twin_stamp_s (stamp gap)
    twin_j1..j6         Latest twin joint positions (rad)
    real_j1..j6         Latest real joint positions (rad)
    err_j1..j6          twin - real, per joint
    err_l2_rad          L2 norm of the error vector
    err_max_rad         max(|err_ji|)
    n_twin              Running count of twin messages received
    n_real              Running count of real messages received

Topics:
    Sub:  /twin/joint_states         sensor_msgs/JointState
    Sub:  /lite6_real/joint_states   sensor_msgs/JointState
    Sub:  /network_profile           std_msgs/String  (latched, transient_local)

Output:
    CSV file at `csv_path` (or auto-generated under ~/testbed_logs/ if empty).
    File is flushed after every row so a crash doesn't lose data.
"""

from __future__ import annotations

import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


LITE6_JOINT_NAMES: List[str] = [f"joint{i + 1}" for i in range(6)]
N_JOINTS: int = 6


def _stamp_to_s(stamp) -> float:
    """Convert builtin_interfaces.msg.Time to seconds float."""
    return stamp.sec + stamp.nanosec * 1e-9


class AoILogger(Node):
    """Record twin + real joint states with AoI and sync error to CSV."""

    def __init__(self) -> None:
        super().__init__("aoi_logger")

        # ---------------- Parameters ---------------- #
        self.declare_parameter("twin_topic", "/twin/joint_states")
        self.declare_parameter("real_topic", "/lite6_real/joint_states")
        self.declare_parameter("profile_topic", "/network_profile")
        self.declare_parameter("gripper_state_topic", "/gripper/state")
        self.declare_parameter("log_rate_hz", 20.0)
        self.declare_parameter("csv_path", "")
        self.declare_parameter("joint_names", LITE6_JOINT_NAMES)
        self.declare_parameter("require_both_before_logging", True)
        self.declare_parameter("status_report_period_s", 5.0)

        twin_topic: str = self.get_parameter("twin_topic").value
        real_topic: str = self.get_parameter("real_topic").value
        profile_topic: str = self.get_parameter("profile_topic").value
        gripper_topic: str = self.get_parameter("gripper_state_topic").value
        self.log_rate_hz: float = float(self.get_parameter("log_rate_hz").value)
        csv_path: str = self.get_parameter("csv_path").value
        self.joint_names: List[str] = list(
            self.get_parameter("joint_names").value)
        self.require_both: bool = bool(
            self.get_parameter("require_both_before_logging").value)
        self.status_period_s: float = float(
            self.get_parameter("status_report_period_s").value)

        if len(self.joint_names) != N_JOINTS:
            raise RuntimeError(
                f"Expected {N_JOINTS} joint names, got {len(self.joint_names)}")
        if self.log_rate_hz <= 0:
            raise RuntimeError("log_rate_hz must be > 0")

        # ---------------- CSV setup ---------------- #
        self._csv_path = self._resolve_csv_path(csv_path)
        self._csv_file = open(self._csv_path, "w", newline="", buffering=1)
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self._csv_header())
        self._csv_file.flush()

        # ---------------- State ---------------- #
        self._latest_twin: Optional[JointState] = None
        self._latest_real: Optional[JointState] = None
        self._latest_twin_rx_wall: float = float("nan")
        self._latest_real_rx_wall: float = float("nan")
        self._current_profile: str = "unknown"
        self._gripper_closed: Optional[bool] = None  # None = never heard
        self._n_twin: int = 0
        self._n_real: int = 0
        self._n_rows: int = 0
        self._t_start = time.time()
        self._last_status = self._t_start

        # ---------------- ROS interfaces ---------------- #
        # Use a transient_local QoS on the profile topic so late-joining
        # loggers still get the currently-active profile.
        profile_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.sub_twin = self.create_subscription(
            JointState, twin_topic, self._on_twin, 50)
        self.sub_real = self.create_subscription(
            JointState, real_topic, self._on_real, 50)
        self.sub_profile = self.create_subscription(
            String, profile_topic, self._on_profile, profile_qos)

        self.timer = self.create_timer(1.0 / self.log_rate_hz, self._tick)

        self.get_logger().info(
            f"aoi_logger: rate={self.log_rate_hz} Hz\n"
            f"  twin_topic    = {twin_topic}\n"
            f"  real_topic    = {real_topic}\n"
            f"  profile_topic = {profile_topic}\n"
            f"  csv_path      = {self._csv_path}")

    # =========================================================== #
    # Path resolution
    # =========================================================== #

    @staticmethod
    def _resolve_csv_path(requested: str) -> str:
        if requested:
            path = Path(requested).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            return str(path)
        # Auto: ~/testbed_logs/aoi_YYYYmmdd_HHMMSS.csv
        log_dir = Path.home() / "testbed_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return str(log_dir / f"aoi_{ts}.csv")

    # =========================================================== #
    # CSV schema
    # =========================================================== #

    def _csv_header(self) -> List[str]:
        head = [
            "wall_time_s", "ros_time_s", "network_profile",
            "twin_stamp_s", "real_stamp_s",
            "aoi_s",           # legacy column == aoi_real_s (primary metric)
            "aoi_twin_s",      # Unity->ROS localhost latency
            "aoi_real_s",      # Lite6->ROS ethernet latency (tc netem affected)
            "msg_dt_s",
        ]
        head += [f"twin_{n}" for n in self.joint_names]
        head += [f"real_{n}" for n in self.joint_names]
        head += [f"err_{n}" for n in self.joint_names]
        head += ["err_l2_rad", "err_max_rad", "n_twin", "n_real"]
        return head

    # =========================================================== #
    # Callbacks
    # =========================================================== #

    def _on_twin(self, msg: JointState) -> None:
        self._latest_twin = msg
        self._latest_twin_rx_wall = time.time()  # local wall time on receipt
        self._n_twin += 1

    def _on_real(self, msg: JointState) -> None:
        self._latest_real = msg
        self._latest_real_rx_wall = time.time()
        self._n_real += 1

    def _on_profile(self, msg: String) -> None:
        new = msg.data.strip() if msg.data else "unknown"
        if new != self._current_profile:
            self.get_logger().info(
                f"Network profile changed: '{self._current_profile}' -> '{new}'")
        self._current_profile = new

    # =========================================================== #
    # Helpers
    # =========================================================== #

    def _positions_by_name(self, msg: JointState) -> Optional[List[float]]:
        """Return positions in self.joint_names order, or None if any missing."""
        if not msg.name or not msg.position:
            # If name field empty, assume positional match
            if len(msg.position) == N_JOINTS:
                return list(msg.position)
            return None
        idx_map = {n: i for i, n in enumerate(msg.name)}
        out: List[float] = []
        for target in self.joint_names:
            if target not in idx_map:
                return None
            out.append(float(msg.position[idx_map[target]]))
        return out

    # =========================================================== #
    # Periodic log tick
    # =========================================================== #

    def _tick(self) -> None:
        wall = time.time()
        ros_now_s = self.get_clock().now().nanoseconds * 1e-9

        twin = self._latest_twin
        real = self._latest_real

        if self.require_both and (twin is None or real is None):
            self._maybe_status(wall, waiting=True)
            return

        twin_pos = self._positions_by_name(twin) if twin else None
        real_pos = self._positions_by_name(real) if real else None

        if self.require_both and (twin_pos is None or real_pos is None):
            # Shape mismatch; don't log a malformed row.
            self._maybe_status(wall, waiting=True)
            return

        twin_stamp = _stamp_to_s(twin.header.stamp) if twin else float("nan")
        real_stamp = _stamp_to_s(real.header.stamp) if real else float("nan")

        # ---- AoI decomposition --------------------------------------- #
        # Two AoI values matter for this testbed:
        #
        #   aoi_twin_s  = wall - twin.rx_wall
        #       Time since the logger last received a twin (Unity) message.
        #       Reflects Unity -> ROS (localhost) latency. Under tc netem
        #       this stays near zero because loopback isn't impaired.
        #
        #   aoi_real_s  = wall - real.rx_wall
        #       Time since the logger last received a real-robot message.
        #       Under tc netem, the real robot's joint_states packets are
        #       delayed in the kernel before reaching user space, so this
        #       metric spikes during impaired windows.
        #
        # IMPORTANT — why we use rx_wall and NOT msg.header.stamp:
        #     The xarm_driver stamps messages AFTER reading the TCP bytes
        #     from the robot, which happens after the kernel releases the
        #     netem-delayed packets. So the stamp already "incorporates"
        #     the delay (stamp ≈ time of kernel release). Computing
        #     wall - stamp gives a near-zero value regardless of netem.
        #     We want the end-to-end AoI as seen by the logger, which is
        #     (wall - rx_wall) — the time since the logger last heard
        #     from the robot. That IS sensitive to netem: each delayed
        #     packet stalls the flow, and rx_wall stays pinned to the
        #     last successful receipt.
        #
        #   aoi_s (kept for backward compatibility) = aoi_real_s
        if not math.isnan(self._latest_twin_rx_wall):
            aoi_twin = wall - self._latest_twin_rx_wall
        else:
            aoi_twin = float("nan")

        if not math.isnan(self._latest_real_rx_wall):
            aoi_real = wall - self._latest_real_rx_wall
        else:
            aoi_real = float("nan")

        # Primary AoI = real-robot receipt-age (the network-impaired channel)
        aoi = aoi_real

        if math.isnan(twin_stamp) or math.isnan(real_stamp):
            msg_dt = float("nan")
        else:
            msg_dt = real_stamp - twin_stamp

        # Error vector
        if twin_pos is not None and real_pos is not None:
            err = [twin_pos[i] - real_pos[i] for i in range(N_JOINTS)]
            err_l2 = math.sqrt(sum(e * e for e in err))
            err_max = max(abs(e) for e in err)
        else:
            err = [float("nan")] * N_JOINTS
            err_l2 = float("nan")
            err_max = float("nan")

        row: List[object] = [
            f"{wall:.9f}",
            f"{ros_now_s:.9f}",
            self._current_profile,
            f"{twin_stamp:.9f}" if not math.isnan(twin_stamp) else "nan",
            f"{real_stamp:.9f}" if not math.isnan(real_stamp) else "nan",
            f"{aoi:.9f}" if not math.isnan(aoi) else "nan",
            f"{aoi_twin:.9f}" if not math.isnan(aoi_twin) else "nan",
            f"{aoi_real:.9f}" if not math.isnan(aoi_real) else "nan",
            f"{msg_dt:.9f}" if not math.isnan(msg_dt) else "nan",
        ]
        row += [f"{v:.9f}" if (twin_pos and not math.isnan(v)) else "nan"
                for v in (twin_pos or [float("nan")] * N_JOINTS)]
        row += [f"{v:.9f}" if (real_pos and not math.isnan(v)) else "nan"
                for v in (real_pos or [float("nan")] * N_JOINTS)]
        row += [f"{v:.9f}" if not math.isnan(v) else "nan" for v in err]
        row += [
            f"{err_l2:.9f}" if not math.isnan(err_l2) else "nan",
            f"{err_max:.9f}" if not math.isnan(err_max) else "nan",
            self._n_twin,
            self._n_real,
        ]

        self._csv_writer.writerow(row)
        self._n_rows += 1

        self._maybe_status(wall, waiting=False, aoi=aoi, err_l2=err_l2)

    def _maybe_status(self, wall: float, waiting: bool,
                      aoi: float = float("nan"),
                      err_l2: float = float("nan")) -> None:
        if wall - self._last_status < self.status_period_s:
            return
        self._last_status = wall
        if waiting:
            self.get_logger().info(
                f"Waiting for messages: twin_recv={self._n_twin}, "
                f"real_recv={self._n_real}, rows={self._n_rows}")
        else:
            aoi_str = f"{aoi * 1000:.1f}ms" if not math.isnan(aoi) else "n/a"
            err_str = f"{err_l2 * 1000:.2f}mrad" if not math.isnan(err_l2) else "n/a"
            self.get_logger().info(
                f"Logging: profile='{self._current_profile}' "
                f"AoI={aoi_str} err_L2={err_str} "
                f"rows={self._n_rows} twin={self._n_twin} real={self._n_real}")

    # =========================================================== #
    # Shutdown
    # =========================================================== #

    def destroy_node(self) -> bool:
        try:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
                self.get_logger().info(
                    f"Closed CSV: {self._csv_path} ({self._n_rows} rows)")
        except Exception as e:
            self.get_logger().warn(f"Error closing CSV: {e}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AoILogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
