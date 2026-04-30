#!/usr/bin/env python3
"""
Robot commander abstractions — dt_teleop_testbed.

Defines the interface that the twin-to-robot bridge uses to command the
real robot. Multiple implementations can plug into this interface, allowing
the bridge to swap between control strategies without any changes to the
rest of the testbed.

Implementations:
    XarmPlannerCommander   — via xarm_planner services (joint plan + exec)
    XarmServiceCommander   — via xarm_api /ufactory/set_servo_angle service
    (future) XarmServoStreamCommander — streaming /ufactory/set_servo_angle_j

All commanders share the same command semantics: given a target joint-space
pose, produce the motion on the real robot. They differ in how they achieve
that (one-shot plan, blocking service call, continuous streaming).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Optional

from rclpy.node import Node


class RobotCommander(ABC):
    """Abstract interface from the bridge to the real robot."""

    @abstractmethod
    def initialize(self) -> bool:
        """Prepare the robot for motion.

        Typically includes enabling motors, setting the correct mode and
        state, waiting for the driver's services/actions to become available.
        Returns True on success.
        """
        ...

    @abstractmethod
    def send_joint_command(self, positions: List[float]) -> bool:
        """Command a target joint-space pose (radians).

        May or may not block — see concrete implementation. Callers should
        treat this as fire-and-forget for teleoperation use.
        Returns True if the command was accepted (not necessarily completed).
        """
        ...

    @abstractmethod
    def ready(self) -> bool:
        """True when the commander has completed initialisation and is ready."""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Release resources, stop motion, disconnect."""
        ...


# --------------------------------------------------------------------- #
# XarmPlannerCommander
# --------------------------------------------------------------------- #

class XarmPlannerCommander(RobotCommander):
    """Command via xarm_planner service interface.

    xarm_planner exposes:
        /xarm_joint_plan   (xarm_msgs/srv/PlanJoint)   — compute plan
        /xarm_exec_plan    (xarm_msgs/srv/PlanExec)    — execute last plan

    Flow: plan(pose) -> wait -> exec(wait=False)

    Notes:
        Each command is a full plan+exec cycle. This is intended for point-
        to-point commands and is NOT suitable for high-rate streaming — plan
        time alone is typically 50-200 ms. For smooth teleop streaming, use
        XarmServoStreamCommander (not yet implemented) or reduce the bridge's
        forward rate.
    """

    def __init__(self, node: Node,
                 plan_service: str = "/xarm_joint_plan",
                 exec_service: str = "/xarm_exec_plan",
                 exec_wait: bool = False,
                 service_timeout_s: float = 5.0) -> None:
        self._node = node
        self._plan_service_name = plan_service
        self._exec_service_name = exec_service
        self._exec_wait = exec_wait
        self._service_timeout_s = service_timeout_s

        self._plan_client = None
        self._exec_client = None
        self._ready = False
        self._in_flight = False

    def initialize(self) -> bool:
        # Import deferred so the module is importable without xarm_msgs.
        try:
            from xarm_msgs.srv import PlanJoint, PlanExec
        except ImportError as e:
            self._node.get_logger().error(
                f"xarm_msgs not available — is xarm_ros2 built and sourced? {e}")
            return False

        self._PlanJoint = PlanJoint
        self._PlanExec = PlanExec

        self._plan_client = self._node.create_client(
            PlanJoint, self._plan_service_name)
        self._exec_client = self._node.create_client(
            PlanExec, self._exec_service_name)

        self._node.get_logger().info(
            f"Waiting for xarm_planner services: "
            f"{self._plan_service_name}, {self._exec_service_name}")
        if not self._plan_client.wait_for_service(
                timeout_sec=self._service_timeout_s):
            self._node.get_logger().error(
                f"Plan service '{self._plan_service_name}' unavailable "
                f"after {self._service_timeout_s}s")
            return False
        if not self._exec_client.wait_for_service(
                timeout_sec=self._service_timeout_s):
            self._node.get_logger().error(
                f"Exec service '{self._exec_service_name}' unavailable "
                f"after {self._service_timeout_s}s")
            return False

        self._ready = True
        self._node.get_logger().info("XarmPlannerCommander ready")
        return True

    def ready(self) -> bool:
        return self._ready

    def send_joint_command(self, positions: List[float]) -> bool:
        if not self._ready:
            return False
        if self._in_flight:
            # Previous command still being processed — drop this one to avoid
            # queueing. This matches teleop semantics: only the newest command
            # matters.
            return False

        req = self._PlanJoint.Request()
        req.target = list(positions)

        self._in_flight = True
        future = self._plan_client.call_async(req)
        future.add_done_callback(self._on_plan_done)
        return True

    def _on_plan_done(self, future) -> None:
        try:
            result = future.result()
        except Exception as e:
            self._node.get_logger().warn(f"Plan service call failed: {e}")
            self._in_flight = False
            return
        if not result or not result.success:
            self._node.get_logger().warn(
                "Plan failed (target unreachable or planner error)")
            self._in_flight = False
            return

        exec_req = self._PlanExec.Request()
        exec_req.wait = self._exec_wait
        fut = self._exec_client.call_async(exec_req)
        fut.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future) -> None:
        try:
            result = future.result()
        except Exception as e:
            self._node.get_logger().warn(f"Exec service call failed: {e}")
        else:
            if result and not result.success:
                self._node.get_logger().warn("Exec reported failure")
        self._in_flight = False

    def shutdown(self) -> None:
        self._ready = False
        # Service clients are cleaned up with the node


# --------------------------------------------------------------------- #
# XarmServiceCommander — simpler alternative using set_servo_angle
# --------------------------------------------------------------------- #

class XarmServiceCommander(RobotCommander):
    """Command via /ufactory/set_servo_angle service (MoveJoint).

    Simpler than the planner path — no plan step, just point-to-point joint
    motion. Fire-and-forget with wait=False for teleop.

    Requires:
        - Driver launched (xarm_api lite6_driver.launch.py)
        - Robot enabled (motion_enable, set_mode 0, set_state 0)
    """

    def __init__(self, node: Node,
                 service_name: str = "/ufactory/set_servo_angle",
                 speed_rad_s: float = 0.5,
                 acc_rad_s2: float = 5.0,
                 service_timeout_s: float = 5.0) -> None:
        self._node = node
        self._service_name = service_name
        self._speed = speed_rad_s
        self._acc = acc_rad_s2
        self._service_timeout_s = service_timeout_s
        self._client = None
        self._ready = False
        self._in_flight = False

    def initialize(self) -> bool:
        try:
            from xarm_msgs.srv import MoveJoint
        except ImportError as e:
            self._node.get_logger().error(
                f"xarm_msgs not available: {e}")
            return False

        self._MoveJoint = MoveJoint
        self._client = self._node.create_client(MoveJoint, self._service_name)

        self._node.get_logger().info(
            f"Waiting for service: {self._service_name}")
        if not self._client.wait_for_service(
                timeout_sec=self._service_timeout_s):
            self._node.get_logger().error(
                f"Service '{self._service_name}' unavailable "
                f"after {self._service_timeout_s}s")
            return False

        self._ready = True
        self._node.get_logger().info("XarmServiceCommander ready")
        return True

    def ready(self) -> bool:
        return self._ready

    def send_joint_command(self, positions: List[float]) -> bool:
        if not self._ready or self._in_flight:
            return False

        req = self._MoveJoint.Request()
        req.angles = list(positions)
        req.speed = float(self._speed)
        req.acc = float(self._acc)
        req.mvtime = 0.0

        self._in_flight = True
        future = self._client.call_async(req)
        future.add_done_callback(self._on_done)
        return True

    def _on_done(self, future) -> None:
        try:
            result = future.result()
            if result and result.ret != 0:
                self._node.get_logger().debug(
                    f"set_servo_angle returned ret={result.ret}")
        except Exception as e:
            self._node.get_logger().debug(f"Service call exception: {e}")
        self._in_flight = False

    def shutdown(self) -> None:
        self._ready = False


# --------------------------------------------------------------------- #
# Factory
# --------------------------------------------------------------------- #

def make_commander(node: Node, kind: str, **kwargs) -> RobotCommander:
    """Create a commander by name."""
    kind = (kind or "").lower()
    if kind in ("planner", "xarm_planner"):
        return XarmPlannerCommander(node, **kwargs)
    if kind in ("service", "set_servo_angle", "xarm_api"):
        return XarmServiceCommander(node, **kwargs)
    raise ValueError(
        f"Unknown commander kind '{kind}'. "
        f"Supported: 'planner', 'service'")
