#!/usr/bin/env python3
"""
Mock Lite 6 node — dt_teleop_testbed.

Stands in for the real UFactory Lite 6 during testing. Subscribes to joint-
space commands on /lite6_real/cmd_joint_state, simulates realistic first-order
joint dynamics (so the 'robot' can't instantaneously reach commanded poses),
and publishes its current state on /lite6_real/joint_states at a fixed rate —
the same topic the real xarm_api driver will publish to.

This means the rest of the testbed (bridge, AoI logger, network conditioner,
launch file) does not need to change when we swap the mock for the real robot
tomorrow. Just stop this node, start the real driver, everything else flows.

Design:
    - Each joint is modelled as a first-order lag:  dq/dt = (q_cmd - q) / tau
      where tau is the response time constant (default 50 ms, typical of a
      well-tuned position-controlled servo).
    - Joint positions are clipped to configurable limits.
    - Optional gaussian measurement noise can be added to the published state
      (off by default) for stress-testing the AoI logger.
    - If no command is received within cmd_timeout_s, the current target is
      frozen — the mock holds its last pose rather than drifting.

Topics:
    Sub:  /lite6_real/cmd_joint_state   (sensor_msgs/JointState, position)
    Pub:  /lite6_real/joint_states      (sensor_msgs/JointState)
"""

from __future__ import annotations

import math
import random
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


LITE6_JOINT_NAMES: List[str] = [f"joint{i + 1}" for i in range(6)]
N_JOINTS: int = 6

# Conservative default limits (radians). Real Lite 6 joint limits are wider
# but these are safe for testing. Override via parameters if needed.
DEFAULT_LOWER = [-2.8, -2.8, -2.8, -2.8, -2.8, -2.8]
DEFAULT_UPPER = [ 2.8,  2.8,  2.8,  2.8,  2.8,  2.8]


class MockLite6(Node):
    """Simulate a UFactory Lite 6 with first-order joint dynamics."""

    def __init__(self) -> None:
        super().__init__("mock_lite6")

        # -------------- Parameters -------------- #
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("response_tau_s", 0.05)
        self.declare_parameter("joint_names", LITE6_JOINT_NAMES)
        self.declare_parameter("initial_positions_rad", [0.0] * N_JOINTS)
        self.declare_parameter("joint_lower_limits_rad", DEFAULT_LOWER)
        self.declare_parameter("joint_upper_limits_rad", DEFAULT_UPPER)
        self.declare_parameter("measurement_noise_std_rad", 0.0)
        self.declare_parameter("cmd_timeout_s", 2.0)
        self.declare_parameter("cmd_topic", "/lite6_real/cmd_joint_state")
        self.declare_parameter("state_topic", "/lite6_real/joint_states")

        self.rate_hz: float = float(self.get_parameter("rate_hz").value)
        self.tau: float = float(self.get_parameter("response_tau_s").value)
        self.joint_names: List[str] = list(
            self.get_parameter("joint_names").value)
        self._positions: List[float] = list(
            self.get_parameter("initial_positions_rad").value)
        self._targets: List[float] = list(self._positions)
        self._lower: List[float] = list(
            self.get_parameter("joint_lower_limits_rad").value)
        self._upper: List[float] = list(
            self.get_parameter("joint_upper_limits_rad").value)
        self._noise_std: float = float(
            self.get_parameter("measurement_noise_std_rad").value)
        self._cmd_timeout: float = float(
            self.get_parameter("cmd_timeout_s").value)
        cmd_topic: str = self.get_parameter("cmd_topic").value
        state_topic: str = self.get_parameter("state_topic").value

        # Validate
        for vec, n in [
            (self.joint_names, "joint_names"),
            (self._positions, "initial_positions_rad"),
            (self._lower, "joint_lower_limits_rad"),
            (self._upper, "joint_upper_limits_rad"),
        ]:
            if len(vec) != N_JOINTS:
                raise RuntimeError(
                    f"Parameter '{n}' must have {N_JOINTS} elements, "
                    f"got {len(vec)}")
        if self.tau <= 0:
            raise RuntimeError("response_tau_s must be > 0")
        if self.rate_hz <= 0:
            raise RuntimeError("rate_hz must be > 0")

        # -------------- State -------------- #
        self._velocities: List[float] = [0.0] * N_JOINTS
        self._last_cmd_time: Optional[float] = None
        self._cmd_count: int = 0

        # -------------- ROS -------------- #
        self.sub = self.create_subscription(
            JointState, cmd_topic, self._on_cmd, 10)
        self.pub = self.create_publisher(JointState, state_topic, 10)

        self._dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"mock_lite6: rate={self.rate_hz} Hz tau={self.tau * 1000:.1f} ms "
            f"cmd_topic='{cmd_topic}' state_topic='{state_topic}' "
            f"initial={self._positions}")

    # =========================================================== #
    # Command handling
    # =========================================================== #

    def _on_cmd(self, msg: JointState) -> None:
        """Receive a target joint-space command."""
        if not msg.position:
            self.get_logger().warn(
                "Received command with empty position field — ignoring.")
            return

        # If msg.name is populated and matches our ordering, use by-name.
        # Otherwise assume positional match to self.joint_names.
        if msg.name and len(msg.name) == len(msg.position):
            targets = list(self._targets)
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_names:
                    idx = self.joint_names.index(name)
                    targets[idx] = float(pos)
                # Silently ignore unknown joints.
        else:
            if len(msg.position) != N_JOINTS:
                self.get_logger().warn(
                    f"Command has {len(msg.position)} positions, "
                    f"expected {N_JOINTS} — ignoring.")
                return
            targets = [float(p) for p in msg.position]

        # Clip to joint limits
        self._targets = [
            max(self._lower[i], min(self._upper[i], targets[i]))
            for i in range(N_JOINTS)
        ]
        self._last_cmd_time = self._now_s()
        self._cmd_count += 1

        if self._cmd_count == 1:
            self.get_logger().info(
                f"First command received: targets={self._targets}")

    # =========================================================== #
    # Dynamics + publish tick
    # =========================================================== #

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        # First-order lag: q_{k+1} = q_k + (q_target - q_k) * (1 - exp(-dt/tau))
        # This is the discrete-time exact solution of dq/dt = (q_target - q)/tau.
        alpha = 1.0 - math.exp(-self._dt / self.tau)
        new_positions: List[float] = []
        new_velocities: List[float] = []
        for i in range(N_JOINTS):
            q = self._positions[i]
            q_target = self._targets[i]
            dq = (q_target - q) * alpha
            q_new = q + dq
            q_new = max(self._lower[i], min(self._upper[i], q_new))
            # Finite-difference velocity
            v = (q_new - q) / self._dt
            new_positions.append(q_new)
            new_velocities.append(v)

        self._positions = new_positions
        self._velocities = new_velocities

        # Optional measurement noise (published only, state stays clean)
        if self._noise_std > 0.0:
            published = [p + random.gauss(0.0, self._noise_std)
                         for p in self._positions]
        else:
            published = list(self._positions)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = published
        msg.velocity = list(self._velocities)
        msg.effort = []
        self.pub.publish(msg)

        # Warn on command timeout (once, then quietly hold)
        if (self._last_cmd_time is not None
                and self._now_s() - self._last_cmd_time > self._cmd_timeout):
            # Emit at debug level to avoid log spam; state holds last target.
            self.get_logger().debug(
                "No command received for > cmd_timeout_s — holding pose.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockLite6()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
