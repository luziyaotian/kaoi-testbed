#!/usr/bin/env python3
"""
Experiment runner — dt_teleop_testbed.

Cycles the testbed through a sequence of network profiles, publishing each
one to /network_profile_request for a configurable dwell time. The AoI
logger (already running as part of the testbed launch) tags each CSV row
with the currently-applied profile, so one run produces a single CSV
containing all conditions.

Typical usage:

    # Terminal 1 — bring up the testbed
    ros2 launch dt_teleop_testbed testbed.launch.py \\
         interface:=<your_eth_iface> use_mock:=true

    # Terminal 2 — run the sweep (dwell=30s per profile, 7 profiles = 3.5 min)
    ros2 run dt_teleop_testbed experiment_runner

Default sequence: ideal → lan → wifi_good → wifi_congested → 4g → poor_4g
                  → satellite → ideal

The final 'ideal' restores the baseline so the network is clean after the
run. Ctrl-C interrupts gracefully and re-applies 'ideal' on exit.

Parameters:
    profiles            List of profile names to visit in order.
    dwell_s             Seconds to spend at each profile.
    settle_s            Seconds after applying a profile before the logger
                        treats data as "clean" for this condition. A note
                        is printed but nothing is actually gated — useful
                        for your offline analysis to know which rows to
                        discard as transient.
    restore_on_exit     Profile name to re-apply when done (default 'ideal').
"""

from __future__ import annotations

import sys
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_SEQUENCE = [
    "ideal",
    "lan",
    "wifi_good",
    "wifi_congested",
    "4g",
    "poor_4g",
    "satellite",
    "ideal",   # final cleanup pass
]


class ExperimentRunner(Node):

    def __init__(self) -> None:
        super().__init__("experiment_runner")

        self.declare_parameter("profiles", DEFAULT_SEQUENCE)
        self.declare_parameter("dwell_s", 30.0)
        self.declare_parameter("settle_s", 2.0)
        self.declare_parameter("restore_on_exit", "ideal")
        self.declare_parameter(
            "request_topic", "/network_profile_request")

        self._profiles: List[str] = [
            str(p) for p in self.get_parameter("profiles").value]
        self._dwell_s: float = float(self.get_parameter("dwell_s").value)
        self._settle_s: float = float(self.get_parameter("settle_s").value)
        self._restore: str = self.get_parameter("restore_on_exit").value

        request_topic: str = self.get_parameter("request_topic").value

        # Use RELIABLE QoS for commands — we want every request to land
        self.pub = self.create_publisher(String, request_topic, 10)

        # Brief wait for publisher to register with subscribers
        time.sleep(0.5)

        total_s = self._dwell_s * len(self._profiles)
        self.get_logger().info(
            f"Experiment plan: {len(self._profiles)} profiles "
            f"× {self._dwell_s}s each = {total_s:.0f}s "
            f"({total_s/60:.1f} min) total")
        self.get_logger().info(
            f"Sequence: {' -> '.join(self._profiles)}")

    def publish_profile(self, name: str) -> None:
        msg = String()
        msg.data = name
        self.pub.publish(msg)

    def run(self) -> None:
        t_start = time.time()
        for i, profile in enumerate(self._profiles, start=1):
            t_profile = time.time()
            self.get_logger().info(
                f"[{i}/{len(self._profiles)}] profile='{profile}' "
                f"(elapsed {time.time() - t_start:.0f}s)")
            self.publish_profile(profile)

            if self._settle_s > 0:
                self.get_logger().info(
                    f"  Settling for {self._settle_s}s — rows during this "
                    f"window will be transient")

            # Dwell
            end = t_profile + self._dwell_s
            while time.time() < end:
                remaining = end - time.time()
                time.sleep(min(1.0, max(0.0, remaining)))
                # Allow spin if needed (not required here but keeps node alive)
                rclpy.spin_once(self, timeout_sec=0.0)

        total = time.time() - t_start
        self.get_logger().info(
            f"Experiment complete in {total:.0f}s "
            f"({len(self._profiles)} profiles processed)")

    def restore(self) -> None:
        if self._restore:
            self.get_logger().info(
                f"Restoring network profile to '{self._restore}'")
            self.publish_profile(self._restore)
            time.sleep(0.3)  # let it publish


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExperimentRunner()
    exit_code = 0
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted — restoring baseline")
        exit_code = 130
    finally:
        try:
            node.restore()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
