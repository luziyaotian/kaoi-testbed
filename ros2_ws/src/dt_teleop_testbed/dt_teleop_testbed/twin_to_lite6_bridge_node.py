#!/usr/bin/env python3
"""
Twin-to-Lite6 bridge — dt_teleop_testbed.

The core testbed component. Bridges the digital twin to the real UFactory
Lite 6 via the xarm_ros2 driver. Responsibilities:

    1. Subscribe to /twin/joint_states — the operator's command stream
       (whether from the headless twin, Unity VR, or a replay).
    2. Forward those commands to the real robot at a controlled rate
       using a pluggable RobotCommander (planner or service).
    3. Subscribe to the driver's state topic (/ufactory/joint_states)
       and republish it as /lite6_real/joint_states so the AoI logger
       can measure twin ↔ real sync on a consistent topic name.

Rate limiting:
    The twin typically publishes at 50-100 Hz. The xarm_planner path can
    sustain ~5-10 Hz (planning takes time). The service path can sustain
    ~20-30 Hz. We decouple the twin input rate from the commander output
    rate: we always keep the latest twin message, and push it to the
    commander at most forward_rate_hz times per second.

Safety:
    - commands are clipped to joint limits
    - a deadman timeout halts commanding if /twin/joint_states stops
    - the commander must be initialised before the first command flows

Topics:
    Sub:  /twin/joint_states         (sensor_msgs/JointState)
    Sub:  <driver_state_topic>       (sensor_msgs/JointState, from xarm driver)
    Pub:  /lite6_real/joint_states   (sensor_msgs/JointState, republished)

Parameters:
    commander_kind                   'planner' or 'service'
    forward_rate_hz                  rate at which twin poses are sent to robot
    driver_state_topic               e.g. /ufactory/joint_states
    twin_topic                       e.g. /twin/joint_states
    real_state_topic                 e.g. /lite6_real/joint_states
    joint_names                      list of 6 joint names
    joint_lower_limits_rad           list of 6 lower limits
    joint_upper_limits_rad           list of 6 upper limits
    deadman_timeout_s                halt if no twin msg within this time
    dry_run                          if True, log commands but don't send them
"""

from __future__ import annotations

import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from dt_teleop_testbed.robot_commanders import make_commander


LITE6_JOINT_NAMES: List[str] = [f"joint{i + 1}" for i in range(6)]
N_JOINTS: int = 6

DEFAULT_LOWER = [-2.8, -2.8, -2.8, -2.8, -2.8, -2.8]
DEFAULT_UPPER = [ 2.8,  2.8,  2.8,  2.8,  2.8,  2.8]


class TwinToLite6Bridge(Node):
    """Relay commands from twin to real robot and state back to logger."""

    def __init__(self) -> None:
        super().__init__("twin_to_lite6_bridge")

        # ---------------- Parameters ---------------- #
        self.declare_parameter("commander_kind", "planner")
        self.declare_parameter("forward_rate_hz", 10.0)
        self.declare_parameter("twin_topic", "/twin/joint_states")
        self.declare_parameter(
            "driver_state_topic", "/ufactory/joint_states")
        self.declare_parameter(
            "real_state_topic", "/lite6_real/joint_states")
        self.declare_parameter("joint_names", LITE6_JOINT_NAMES)
        self.declare_parameter("joint_lower_limits_rad", DEFAULT_LOWER)
        self.declare_parameter("joint_upper_limits_rad", DEFAULT_UPPER)
        self.declare_parameter("deadman_timeout_s", 1.0)
        self.declare_parameter("dry_run", False)

        # Commander-specific parameters (safe defaults even if unused)
        self.declare_parameter("speed_rad_s", 0.5)
        self.declare_parameter("acc_rad_s2", 5.0)
        self.declare_parameter("plan_service", "/xarm_joint_plan")
        self.declare_parameter("exec_service", "/xarm_exec_plan")
        self.declare_parameter(
            "service_name", "/ufactory/set_servo_angle")
        self.declare_parameter("service_timeout_s", 5.0)

        self.commander_kind: str = self.get_parameter("commander_kind").value
        self.forward_rate_hz: float = float(
            self.get_parameter("forward_rate_hz").value)
        twin_topic: str = self.get_parameter("twin_topic").value
        driver_state_topic: str = self.get_parameter(
            "driver_state_topic").value
        real_state_topic: str = self.get_parameter(
            "real_state_topic").value
        self.joint_names: List[str] = list(
            self.get_parameter("joint_names").value)
        self._lower: List[float] = [
            float(x) for x in self.get_parameter(
                "joint_lower_limits_rad").value]
        self._upper: List[float] = [
            float(x) for x in self.get_parameter(
                "joint_upper_limits_rad").value]
        self._deadman_s: float = float(
            self.get_parameter("deadman_timeout_s").value)
        self._dry_run: bool = bool(self.get_parameter("dry_run").value)

        if len(self.joint_names) != N_JOINTS:
            raise RuntimeError(
                f"Expected {N_JOINTS} joint names, got {len(self.joint_names)}")

        # ---------------- Commander ---------------- #
        kwargs = {}
        if self.commander_kind in ("planner", "xarm_planner"):
            kwargs = dict(
                plan_service=self.get_parameter("plan_service").value,
                exec_service=self.get_parameter("exec_service").value,
                service_timeout_s=float(
                    self.get_parameter("service_timeout_s").value),
            )
        elif self.commander_kind in ("service", "set_servo_angle", "xarm_api"):
            kwargs = dict(
                service_name=self.get_parameter("service_name").value,
                speed_rad_s=float(self.get_parameter("speed_rad_s").value),
                acc_rad_s2=float(self.get_parameter("acc_rad_s2").value),
                service_timeout_s=float(
                    self.get_parameter("service_timeout_s").value),
            )
        self._commander = make_commander(self, self.commander_kind, **kwargs)

        # ---------------- State ---------------- #
        self._latest_twin: Optional[JointState] = None
        self._latest_twin_t: Optional[float] = None
        self._last_sent_positions: Optional[List[float]] = None
        self._n_twin_rx: int = 0
        self._n_sent: int = 0
        self._n_dropped_inflight: int = 0
        self._n_deadman: int = 0

        # ---------------- ROS ---------------- #
        self.sub_twin = self.create_subscription(
            JointState, twin_topic, self._on_twin, 50)
        self.sub_driver = self.create_subscription(
            JointState, driver_state_topic, self._on_driver_state, 50)
        self.pub_real = self.create_publisher(
            JointState, real_state_topic, 50)

        # Commander init happens after construction so it can log properly
        ok = self._commander.initialize()
        if not ok and not self._dry_run:
            self.get_logger().error(
                "Commander initialisation failed. "
                "Check that the xarm driver is running and the robot is enabled.")
        elif self._dry_run:
            self.get_logger().warn(
                "Running in DRY-RUN mode — no commands will be sent to robot.")

        self.timer = self.create_timer(
            1.0 / self.forward_rate_hz, self._forward_tick)
        self.status_timer = self.create_timer(5.0, self._status_tick)

        self.get_logger().info(
            f"twin_to_lite6_bridge ready:\n"
            f"  commander = {self.commander_kind}\n"
            f"  forward   = {self.forward_rate_hz} Hz\n"
            f"  twin_topic= {twin_topic}\n"
            f"  driver    = {driver_state_topic}\n"
            f"  publish   = {real_state_topic}\n"
            f"  dry_run   = {self._dry_run}")

    # =========================================================== #
    # Subscription callbacks
    # =========================================================== #

    def _on_twin(self, msg: JointState) -> None:
        self._latest_twin = msg
        self._latest_twin_t = time.time()
        self._n_twin_rx += 1

    def _on_driver_state(self, msg: JointState) -> None:
        # Republish verbatim so the logger sees it on the testbed's topic.
        self.pub_real.publish(msg)

    # =========================================================== #
    # Helpers
    # =========================================================== #

    def _twin_positions(self) -> Optional[List[float]]:
        msg = self._latest_twin
        if msg is None:
            return None
        if not msg.position:
            return None
        # Reorder by joint_names if names given
        if msg.name and len(msg.name) == len(msg.position):
            idx_map = {n: i for i, n in enumerate(msg.name)}
            try:
                return [float(msg.position[idx_map[n]]) for n in self.joint_names]
            except KeyError:
                return None
        if len(msg.position) == N_JOINTS:
            return [float(x) for x in msg.position]
        return None

    def _clip(self, positions: List[float]) -> List[float]:
        return [max(self._lower[i], min(self._upper[i], positions[i]))
                for i in range(N_JOINTS)]

    def _positions_changed(self, new: List[float]) -> bool:
        if self._last_sent_positions is None:
            return True
        # Only send if any joint changed by > 1 mrad
        return any(abs(new[i] - self._last_sent_positions[i]) > 1e-3
                   for i in range(N_JOINTS))

    # =========================================================== #
    # Forward tick
    # =========================================================== #

    def _forward_tick(self) -> None:
        now = time.time()

        # Deadman check
        if (self._latest_twin_t is None
                or (now - self._latest_twin_t) > self._deadman_s):
            self._n_deadman += 1
            return

        positions = self._twin_positions()
        if positions is None:
            return
        positions = self._clip(positions)

        if not self._positions_changed(positions):
            return

        if self._dry_run:
            self._n_sent += 1
            self._last_sent_positions = positions
            return

        if not self._commander.ready():
            return

        ok = self._commander.send_joint_command(positions)
        if ok:
            self._n_sent += 1
            self._last_sent_positions = positions
        else:
            self._n_dropped_inflight += 1

    def _status_tick(self) -> None:
        self.get_logger().info(
            f"bridge: twin_rx={self._n_twin_rx} sent={self._n_sent} "
            f"dropped_inflight={self._n_dropped_inflight} "
            f"deadman_ticks={self._n_deadman}")

    # =========================================================== #
    # Shutdown
    # =========================================================== #

    def destroy_node(self) -> bool:
        try:
            self._commander.shutdown()
        except Exception as e:
            self.get_logger().warn(f"Commander shutdown error: {e}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinToLite6Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
