from __future__ import annotations
import copy
import threading
from geometry_msgs.msg import Pose2D

from pathfinding_system.robot.motion import (
    DriveResult,
    MotionController,
    MotionParameters,
    PathFollower,
    PathStep,
)
from pathfinding_system.robot.robot_status import RobotStatus
from pathfinding_system.robot.robot_state import RobotState
from pathfinding_system.world.node import Node


class TurtleBot:
    def __init__(
        self,
        robot_id: str,
        motion_parameters: MotionParameters = MotionParameters(),
    ) -> None:
        self.id = robot_id
        self._ns = robot_id
        self._motion_controller = MotionController(motion_parameters)
        self._path_follower = PathFollower(self._motion_controller)
        self._state = RobotState(id=robot_id)
        self._stop_requested = False
        self._lock = threading.Lock()

    @property
    def cmd_vel_topic(self) -> str:
        return f'/{self._ns}/cmd_vel'

    @property
    def state_topic(self) -> str:
        return f'/{self._ns}/robot_state'

    def drive_towards(self, node: Node) -> DriveResult:
        with self._lock:
            pose = copy.deepcopy(self._state.pose)
            self._state.status = RobotStatus.MOVING

        return self._motion_controller.drive_towards(pose, node)

    def start_path(self, waypoints: list[Node]) -> None:
        with self._lock:
            self._stop_requested = False
            self._state.status = RobotStatus.MOVING
            self._path_follower.start(waypoints)

    def step_path(self) -> PathStep:
        with self._lock:
            pose = copy.deepcopy(self._state.pose)
            step = self._path_follower.step(pose)
            if step.completed:
                self._state.status = RobotStatus.REACHED
            return step

    def cancel_path(self) -> None:
        with self._lock:
            self._path_follower.cancel()
            self._state.status = RobotStatus.IDLE

    def stop_requested(self) -> bool:
        with self._lock:
            return self._stop_requested

    def current_pose(self) -> Pose2D:
        with self._lock:
            pose = Pose2D()
            pose.x = self._state.pose.x
            pose.y = self._state.pose.y
            pose.theta = self._state.pose.theta
            return pose

    def state_snapshot(self) -> RobotState:
        with self._lock:
            return copy.deepcopy(self._state)

    def update_pose(self, x: float, y: float, theta: float, velocity=None, stamp=None) -> None:
        with self._lock:
            self._state.pose.x = x
            self._state.pose.y = y
            self._state.pose.theta = theta
            if velocity is not None:
                self._state.velocity = velocity
            self._state.stamp = stamp

    def mark_moving(self) -> None:
        with self._lock:
            self._state.status = RobotStatus.MOVING

    def mark_idle(self) -> None:
        with self._lock:
            self._state.status = RobotStatus.IDLE

    def mark_reached(self) -> None:
        with self._lock:
            self._state.status = RobotStatus.REACHED

    def request_stop(self) -> None:
        with self._lock:
            self._stop_requested = True
            self._state.status = RobotStatus.STOPPED

    def clear_stop(self) -> None:
        with self._lock:
            self._stop_requested = False
