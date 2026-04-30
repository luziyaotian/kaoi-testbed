#!/usr/bin/env python3
"""
Headless twin node — dt_teleop_testbed.

Publishes /twin/joint_states at a fixed rate based on one of three scripted
trajectory modes. This node intentionally has no physics and no visualisation:
it is the "data contract" for the twin. Unity (or any other front-end) can
replace it later by simply publishing /twin/joint_states itself — downstream
components (network conditioner, robot bridge, AoI logger) do not need to
change.

Modes (select via the `mode` parameter):
    sine       — each joint follows an independent sine wave. Best for
                 frequency-domain characterisation of the pipeline.
    waypoints  — linear interpolation between joint-space waypoints loaded
                 from a YAML file. Best for task-like motion.
    replay     — replay a recorded trajectory from a CSV file. Best for
                 deterministic reproduction of past sessions (including
                 recorded VR sessions in future experiments).
    idle       — publish a constant zero pose (sanity / debugging).

Topic:
    /twin/joint_states  (sensor_msgs/JointState)

Joint naming matches the xarm_description URDF for the Lite 6:
    joint1, joint2, joint3, joint4, joint5, joint6
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


LITE6_JOINT_NAMES: List[str] = [f"joint{i + 1}" for i in range(6)]
N_JOINTS: int = 6


class HeadlessTwin(Node):
    """Scripted twin: publishes joint positions on /twin/joint_states."""

    def __init__(self) -> None:
        super().__init__("headless_twin")

        # ---------------- Parameters ---------------- #
        self.declare_parameter("mode", "sine")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("joint_names", LITE6_JOINT_NAMES)
        self.declare_parameter("topic", "/twin/joint_states")
        self.declare_parameter("publish_velocity", True)

        # Sine-mode parameters (one value per joint)
        self.declare_parameter("sine.amplitudes_rad",
                               [0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
        self.declare_parameter("sine.frequencies_hz",
                               [0.10, 0.15, 0.20, 0.10, 0.15, 0.20])
        self.declare_parameter("sine.phases_rad",
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("sine.centers_rad",
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Waypoint-mode parameters
        self.declare_parameter("waypoints.file", "")
        self.declare_parameter("waypoints.loop", True)

        # Replay-mode parameters
        self.declare_parameter("replay.file", "")
        self.declare_parameter("replay.loop", True)
        self.declare_parameter("replay.speed", 1.0)

        # Read top-level parameters
        self.mode: str = self.get_parameter("mode").value
        self.rate_hz: float = float(self.get_parameter("rate_hz").value)
        self.joint_names: List[str] = list(
            self.get_parameter("joint_names").value)
        self.topic: str = self.get_parameter("topic").value
        self.publish_velocity: bool = bool(
            self.get_parameter("publish_velocity").value)

        if len(self.joint_names) != N_JOINTS:
            raise RuntimeError(
                f"Expected {N_JOINTS} joint names, got {len(self.joint_names)}")

        # ---------------- Mode setup ---------------- #
        self._setup_mode()

        # ---------------- ROS interfaces ---------------- #
        self.pub = self.create_publisher(JointState, self.topic, 10)
        self._start = self.get_clock().now()
        self._last_positions: List[float] = [0.0] * N_JOINTS
        self._last_t: float = 0.0

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"headless_twin: mode='{self.mode}' rate={self.rate_hz} Hz "
            f"topic='{self.topic}' joints={self.joint_names}")

    # =========================================================== #
    # Mode loaders
    # =========================================================== #

    def _setup_mode(self) -> None:
        if self.mode == "sine":
            self._sine_amp = self._param_vec("sine.amplitudes_rad")
            self._sine_freq = self._param_vec("sine.frequencies_hz")
            self._sine_phase = self._param_vec("sine.phases_rad")
            self._sine_center = self._param_vec("sine.centers_rad")

        elif self.mode == "waypoints":
            path = self.get_parameter("waypoints.file").value
            if not path:
                raise RuntimeError(
                    "waypoints.file must be set when mode='waypoints'")
            self._waypoints = self._load_waypoints(path)
            self._wp_total = sum(wp[1] for wp in self._waypoints)
            self._wp_loop = bool(self.get_parameter("waypoints.loop").value)
            if self._wp_total <= 0:
                raise RuntimeError(
                    "Sum of waypoint durations must be > 0")

        elif self.mode == "replay":
            path = self.get_parameter("replay.file").value
            if not path:
                raise RuntimeError(
                    "replay.file must be set when mode='replay'")
            self._replay = self._load_replay(path)
            self._replay_loop = bool(self.get_parameter("replay.loop").value)
            self._replay_speed = float(self.get_parameter("replay.speed").value)
            if len(self._replay) < 2:
                raise RuntimeError(
                    "Replay CSV must contain at least 2 rows")

        elif self.mode == "idle":
            pass

        else:
            raise RuntimeError(
                f"Unknown mode '{self.mode}'. "
                f"Expected one of: sine, waypoints, replay, idle")

    def _param_vec(self, name: str) -> List[float]:
        vec = list(self.get_parameter(name).value)
        if len(vec) != N_JOINTS:
            raise RuntimeError(
                f"Parameter '{name}' must have {N_JOINTS} elements, "
                f"got {len(vec)}")
        return [float(x) for x in vec]

    @staticmethod
    def _load_waypoints(path: str) -> List[Tuple[List[float], float]]:
        """YAML schema:
            waypoints:
              - positions: [j1, j2, j3, j4, j5, j6]    # radians
                duration_to_next_s: 2.0
              - ...
        """
        import yaml  # local import so the node still imports without pyyaml
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        raw = data["waypoints"]
        out: List[Tuple[List[float], float]] = []
        for i, wp in enumerate(raw):
            pos = [float(x) for x in wp["positions"]]
            if len(pos) != N_JOINTS:
                raise RuntimeError(
                    f"Waypoint {i} has {len(pos)} positions, expected {N_JOINTS}")
            dur = float(wp.get("duration_to_next_s", 1.0))
            out.append((pos, dur))
        return out

    @staticmethod
    def _load_replay(path: str) -> List[Tuple[float, List[float]]]:
        """CSV schema: t_s, joint1, joint2, joint3, joint4, joint5, joint6"""
        rows: List[Tuple[float, List[float]]] = []
        with open(path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                t = float(row["t_s"])
                pos = [float(row[f"joint{i + 1}"]) for i in range(N_JOINTS)]
                rows.append((t, pos))
        rows.sort(key=lambda r: r[0])
        return rows

    # =========================================================== #
    # Trajectory evaluation
    # =========================================================== #

    def _elapsed_s(self) -> float:
        return (self.get_clock().now() - self._start).nanoseconds * 1e-9

    def _positions_at(self, t: float) -> List[float]:
        if self.mode == "sine":
            two_pi = 2.0 * math.pi
            return [
                self._sine_center[i]
                + self._sine_amp[i]
                * math.sin(two_pi * self._sine_freq[i] * t + self._sine_phase[i])
                for i in range(N_JOINTS)
            ]

        if self.mode == "waypoints":
            if self._wp_loop:
                t = t % self._wp_total
            elapsed = 0.0
            for i, (pos, dur) in enumerate(self._waypoints):
                if i == len(self._waypoints) - 1 or dur <= 0:
                    if t <= elapsed + dur:
                        return pos
                    elapsed += dur
                    continue
                if elapsed + dur >= t:
                    alpha = max(0.0, min(1.0, (t - elapsed) / dur))
                    nxt = self._waypoints[i + 1][0]
                    return [pos[j] + alpha * (nxt[j] - pos[j])
                            for j in range(N_JOINTS)]
                elapsed += dur
            return self._waypoints[-1][0]

        if self.mode == "replay":
            t_s = t * self._replay_speed
            total = self._replay[-1][0] - self._replay[0][0]
            if self._replay_loop and total > 0:
                t_s = (t_s % total) + self._replay[0][0]
            # Clamp
            if t_s <= self._replay[0][0]:
                return list(self._replay[0][1])
            if t_s >= self._replay[-1][0]:
                return list(self._replay[-1][1])
            # Linear search — fine for typical replay sizes (<10k rows).
            # For very large replays, switch to bisect.
            for i in range(len(self._replay) - 1):
                t0, p0 = self._replay[i]
                t1, p1 = self._replay[i + 1]
                if t0 <= t_s <= t1:
                    alpha = (t_s - t0) / (t1 - t0) if t1 > t0 else 0.0
                    return [p0[j] + alpha * (p1[j] - p0[j])
                            for j in range(N_JOINTS)]
            return list(self._replay[-1][1])

        # idle
        return [0.0] * N_JOINTS

    # =========================================================== #
    # Publish tick
    # =========================================================== #

    def _tick(self) -> None:
        t = self._elapsed_s()
        pos = self._positions_at(t)

        # Finite-difference velocity estimate; zero on the first tick.
        dt = t - self._last_t
        if self.publish_velocity and dt > 1e-6:
            vel = [(pos[i] - self._last_positions[i]) / dt
                   for i in range(N_JOINTS)]
        else:
            vel = [0.0] * N_JOINTS

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = pos
        msg.velocity = vel if self.publish_velocity else []
        msg.effort = []

        self.pub.publish(msg)

        self._last_positions = pos
        self._last_t = t


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadlessTwin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
