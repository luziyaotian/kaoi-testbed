#!/usr/bin/env python3
"""
gripper_bridge_node — Bool topic → Lite 6 gripper service.

Listens on /gripper/command as std_msgs/Bool:
    msg.data = True  → call /ufactory/close_lite6_gripper
    msg.data = False → call /ufactory/open_lite6_gripper

Publishes latched /gripper/state (Bool) with the last commanded state so
downstream consumers (AoI logger, Unity overlay) can correlate gripper
actions with joint data.

Quality-of-life features:
    - Auto-stop: if no new command arrives for `idle_stop_s` seconds, we
      call /ufactory/stop_lite6_gripper to silence the motor. Set to 0 to
      disable. Default 3.0 s.
    - Hardware quirk handling: after stop, close is invalid until open has
      been called. This node tracks whether an open has been issued this
      session and automatically inserts an open before the first close.
    - Deduplication: same command twice in a row is a no-op.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool
from xarm_msgs.srv import Call


class GripperBridge(Node):
    def __init__(self) -> None:
        super().__init__("gripper_bridge")

        self.declare_parameter("command_topic", "/gripper/command")
        self.declare_parameter("state_topic", "/gripper/state")
        self.declare_parameter("open_service", "/ufactory/open_lite6_gripper")
        self.declare_parameter("close_service", "/ufactory/close_lite6_gripper")
        self.declare_parameter("stop_service", "/ufactory/stop_lite6_gripper")
        self.declare_parameter("dry_run", False)
        self.declare_parameter(
            "idle_stop_s", 3.0,
            descriptor=None)  # Call stop after this many seconds idle. 0 = off.

        self.cmd_topic = self.get_parameter("command_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.open_name = self.get_parameter("open_service").value
        self.close_name = self.get_parameter("close_service").value
        self.stop_name = self.get_parameter("stop_service").value
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.idle_stop_s = float(self.get_parameter("idle_stop_s").value)

        self._open_cli = self.create_client(Call, self.open_name)
        self._close_cli = self.create_client(Call, self.close_name)
        self._stop_cli = self.create_client(Call, self.stop_name)

        if not self.dry_run:
            self.get_logger().info(
                f"Waiting for services:\n"
                f"  {self.open_name}\n"
                f"  {self.close_name}\n"
                f"  {self.stop_name}")
            self._open_cli.wait_for_service(timeout_sec=10.0)
            self._close_cli.wait_for_service(timeout_sec=10.0)
            self._stop_cli.wait_for_service(timeout_sec=10.0)
            if not all([self._open_cli.service_is_ready(),
                        self._close_cli.service_is_ready(),
                        self._stop_cli.service_is_ready()]):
                self.get_logger().error(
                    "Gripper services not available. "
                    "Is the xarm_api driver running and configured with "
                    "open_lite6_gripper/close_lite6_gripper/stop_lite6_gripper "
                    "set to true in xarm_params.yaml?")

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._state_pub = self.create_publisher(Bool, self.state_topic, latched)

        self._cmd_sub = self.create_subscription(
            Bool, self.cmd_topic, self._on_cmd, 10)

        # --- internal state ---
        self._has_opened = False        # Have we issued an open this session?
        self._last_cmd: bool | None = None   # Last commanded bool (T=closed)
        self._stopped = False           # Are we in the stopped state?

        # Auto-stop idle timer. We use a wall-clock time marker so we can
        # check elapsed time in a periodic tick.
        self._last_activity_time = self.get_clock().now()

        if self.idle_stop_s > 0:
            # Tick at 2 Hz — good enough to auto-stop within a half-second
            # of the threshold.
            self._idle_timer = self.create_timer(0.5, self._check_idle)
        else:
            self._idle_timer = None

        self._publish_state(False)

        self.get_logger().info(
            f"gripper_bridge ready.\n"
            f"  command    = {self.cmd_topic}\n"
            f"  state      = {self.state_topic}\n"
            f"  idle_stop_s= {self.idle_stop_s:.1f}s "
            f"({'enabled' if self.idle_stop_s > 0 else 'disabled'})\n"
            f"  dry_run    = {self.dry_run}")

    # ================================================================ #
    def _publish_state(self, closed: bool) -> None:
        msg = Bool()
        msg.data = bool(closed)
        self._state_pub.publish(msg)

    def _bump_activity(self) -> None:
        self._last_activity_time = self.get_clock().now()

    # ================================================================ #
    def _call_open(self) -> None:
        self._bump_activity()
        self._stopped = False
        if self.dry_run:
            self.get_logger().info("[dry_run] open")
            self._has_opened = True
            return
        try:
            fut = self._open_cli.call_async(Call.Request())
            fut.add_done_callback(lambda f: self._log_result("open", f))
            self._has_opened = True
        except Exception as e:
            self.get_logger().error(f"open service call failed: {e}")

    def _call_close(self) -> None:
        self._bump_activity()
        self._stopped = False
        if self.dry_run:
            self.get_logger().info("[dry_run] close")
            return
        try:
            fut = self._close_cli.call_async(Call.Request())
            fut.add_done_callback(lambda f: self._log_result("close", f))
        except Exception as e:
            self.get_logger().error(f"close service call failed: {e}")

    def _call_stop(self) -> None:
        # Do NOT bump activity here — we're stopping because of inactivity.
        self._stopped = True
        # After stop, a fresh open must happen before any close.
        self._has_opened = False
        if self.dry_run:
            self.get_logger().info("[dry_run] stop (idle)")
            return
        try:
            fut = self._stop_cli.call_async(Call.Request())
            fut.add_done_callback(lambda f: self._log_result("stop", f))
        except Exception as e:
            self.get_logger().error(f"stop service call failed: {e}")

    def _log_result(self, label: str, future) -> None:
        try:
            result = future.result()
            if result is None:
                self.get_logger().warning(f"{label}: no response")
                return
            if result.ret != 0:
                self.get_logger().warning(
                    f"{label}: non-zero return {result.ret} "
                    f"({result.message})")
            else:
                self.get_logger().info(f"{label}: OK")
        except Exception as e:
            self.get_logger().error(f"{label}: callback exception: {e}")

    # ================================================================ #
    def _check_idle(self) -> None:
        """Periodic tick: if no activity for idle_stop_s, call stop()."""
        if self._stopped or self.idle_stop_s <= 0:
            return

        now = self.get_clock().now()
        elapsed = (now - self._last_activity_time).nanoseconds * 1e-9
        if elapsed >= self.idle_stop_s:
            self.get_logger().info(
                f"Idle for {elapsed:.1f}s (threshold {self.idle_stop_s:.1f}s); "
                f"calling stop to silence motor.")
            self._call_stop()

    # ================================================================ #
    def _on_cmd(self, msg: Bool) -> None:
        want_closed = bool(msg.data)

        # Deduplicate — same command twice in a row is a no-op.
        # BUT if we've gone into the stopped state, allow re-issue.
        if (self._last_cmd is not None
                and self._last_cmd == want_closed
                and not self._stopped):
            self._bump_activity()   # still counts as activity
            return
        self._last_cmd = want_closed

        if want_closed:
            # Hardware quirk: close is invalid until open has been called.
            if not self._has_opened:
                self.get_logger().info(
                    "First close (or close after stop) — issuing open first.")
                self._call_open()
            self._call_close()
            self._publish_state(True)
        else:
            self._call_open()
            self._publish_state(False)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
