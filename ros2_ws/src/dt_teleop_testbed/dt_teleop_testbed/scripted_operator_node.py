#!/usr/bin/env python3
"""
Scripted operator node — dt_teleop_testbed.

Replaces Unity + sliders with a reproducible multi-joint trajectory
source. Publishes /twin/joint_states at 50 Hz following a pre-defined
pattern so that E_scripted experiments have zero operator variance.

The scripted motion is designed to approximate the operator's "metronome
sweep" protocol:
    joint 1: ±0.5 rad sinusoid, period 4 s
    joint 2: ±0.3 rad sinusoid, period 6 s (phase-shifted)
    joints 3-6: hold at initial positions

This keeps the primary motion in joints 1 and 2 (avoiding joint 3's
asymmetric limit and joints 4-6's small ranges).

Parameters:
    topic              str     "/twin/joint_states"
    rate_hz            float   50.0  publish frequency
    j1_amplitude_rad   float   0.5   joint 1 swing amplitude
    j1_period_s        float   4.0   joint 1 cycle period
    j2_amplitude_rad   float   0.3   joint 2 swing amplitude
    j2_period_s        float   6.0   joint 2 cycle period
    initial_wait_s     float   2.0   silence before first publish (lets
                                     subscribers register)
    seed_from_topic    str     ""    if non-empty, wait for one message
                                     on this topic and use its position
                                     as the trajectory centre (e.g.,
                                     /ufactory/joint_states to start from
                                     the real robot's current pose)

Usage from launch:
    ros2 run dt_teleop_testbed scripted_operator

    or with custom params:
    ros2 run dt_teleop_testbed scripted_operator --ros-args \
        -p j1_amplitude_rad:=0.3 -p j1_period_s:=3.0

Stops by Ctrl+C. Publishes forever until stopped.

Reproducibility contract:
    Given identical parameters, two runs will produce IDENTICAL joint
    trajectories (deterministic from t=0). Combined with tc netem this
    gives fully reproducible network-impairment experiments.
"""

from __future__ import annotations

import math
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class ScriptedOperator(Node):
    """Publishes a reproducible joint trajectory on /twin/joint_states."""

    def __init__(self) -> None:
        super().__init__("scripted_operator")

        # -------------------- parameters -------------------- #
        self.declare_parameter("topic", "/twin/joint_states")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("j1_amplitude_rad", 0.5)
        self.declare_parameter("j1_period_s", 4.0)
        self.declare_parameter("j2_amplitude_rad", 0.3)
        self.declare_parameter("j2_period_s", 6.0)
        self.declare_parameter("initial_wait_s", 2.0)
        self.declare_parameter("seed_from_topic", "")
        self.declare_parameter("seed_wait_s", 3.0)

        self.topic = self.get_parameter("topic").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.j1_amp = float(self.get_parameter("j1_amplitude_rad").value)
        self.j1_T = float(self.get_parameter("j1_period_s").value)
        self.j2_amp = float(self.get_parameter("j2_amplitude_rad").value)
        self.j2_T = float(self.get_parameter("j2_period_s").value)
        self.initial_wait_s = float(self.get_parameter("initial_wait_s").value)
        self.seed_topic = self.get_parameter("seed_from_topic").value
        self.seed_wait_s = float(self.get_parameter("seed_wait_s").value)

        # -------------------- state -------------------- #
        self._centre = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._seeded = False
        self._seed_sub = None

        # -------------------- seed if requested -------------------- #
        if self.seed_topic:
            self.get_logger().info(
                f"Waiting up to {self.seed_wait_s:.1f}s for seed from "
                f"{self.seed_topic}...")
            self._seed_sub = self.create_subscription(
                JointState, self.seed_topic, self._on_seed, 10)
            # Busy-wait for seed up to seed_wait_s
            start = time.time()
            while not self._seeded and (time.time() - start) < self.seed_wait_s:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not self._seeded:
                self.get_logger().warning(
                    f"Seed timeout; using zero joint angles as trajectory "
                    f"centre.")
            # Unsubscribe once seeded or timed out
            if self._seed_sub is not None:
                self.destroy_subscription(self._seed_sub)
                self._seed_sub = None
        else:
            self.get_logger().info("No seed topic; using zero joint centre.")

        # -------------------- publisher -------------------- #
        self._pub = self.create_publisher(JointState, self.topic, 50)
        self._period = 1.0 / max(1.0, self.rate_hz)
        self._start_time = time.time()
        self._tick_count = 0
        self._timer = self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"ScriptedOperator ready on {self.topic} @ {self.rate_hz:.1f} Hz\n"
            f"  j1: amplitude={self.j1_amp:.3f} rad, period={self.j1_T:.2f} s\n"
            f"  j2: amplitude={self.j2_amp:.3f} rad, period={self.j2_T:.2f} s\n"
            f"  centre = {self._centre}\n"
            f"  initial_wait = {self.initial_wait_s:.1f} s")

    def _on_seed(self, msg: JointState) -> None:
        if self._seeded:
            return
        if msg.position is None or len(msg.position) < 6:
            return
        # Match by name if names are present, else positional
        if msg.name and len(msg.name) == len(msg.position):
            name_to_pos = dict(zip(msg.name, msg.position))
            for i, jn in enumerate(JOINT_NAMES):
                if jn in name_to_pos:
                    self._centre[i] = float(name_to_pos[jn])
        else:
            for i in range(min(6, len(msg.position))):
                self._centre[i] = float(msg.position[i])
        self._seeded = True
        self.get_logger().info(f"Seeded from {self.seed_topic}: {self._centre}")

    def _tick(self) -> None:
        t = time.time() - self._start_time

        # Silence for the initial wait period (lets subscribers register)
        if t < self.initial_wait_s:
            return

        t_motion = t - self.initial_wait_s

        positions = list(self._centre)
        positions[0] = self._centre[0] + self.j1_amp * math.sin(
            2.0 * math.pi * t_motion / self.j1_T)
        positions[1] = self._centre[1] + self.j2_amp * math.sin(
            2.0 * math.pi * t_motion / self.j2_T + math.pi / 2)
        # joints 3-6 stay at centre

        velocities = [0.0] * 6
        velocities[0] = self.j1_amp * (2.0 * math.pi / self.j1_T) * math.cos(
            2.0 * math.pi * t_motion / self.j1_T)
        velocities[1] = self.j2_amp * (2.0 * math.pi / self.j2_T) * math.cos(
            2.0 * math.pi * t_motion / self.j2_T + math.pi / 2)

        now = time.time()
        sec = int(math.floor(now))
        nsec = int((now - sec) * 1e9)

        msg = JointState()
        msg.header = Header()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = "world"
        msg.name = list(JOINT_NAMES)
        msg.position = positions
        msg.velocity = velocities
        msg.effort = []

        self._pub.publish(msg)
        self._tick_count += 1

        if self._tick_count == 1 or self._tick_count % 500 == 0:
            self.get_logger().info(
                f"publish #{self._tick_count}: t_motion={t_motion:.2f}s "
                f"j1={positions[0]:.3f} j2={positions[1]:.3f}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScriptedOperator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
