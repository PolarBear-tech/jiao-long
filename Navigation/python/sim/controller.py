from __future__ import annotations

import random
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from sim.kinematics import HolonomicKinematics, Kinematics

if TYPE_CHECKING:
    from sim.robot import Robot


@dataclass(frozen=True)
class ControlLimits:
    """两个控制通道的速度和加速度限制。"""

    maximum: tuple[float, float]
    minimum: tuple[float, float]
    acceleration_max: float
    acceleration_min: float

    def __post_init__(self) -> None:
        if any(value < 0 for value in self.maximum + self.minimum):
            raise ValueError("control speed limits must not be negative")
        if self.acceleration_max < 0 or self.acceleration_min > 0:
            raise ValueError("invalid acceleration limits")

    def apply(
        self,
        command: tuple[float, float],
        previous: tuple[float, float],
        time_step: float,
    ) -> tuple[float, float]:
        limited: list[float] = []
        for index, value in enumerate(command):
            maximum = self.maximum[index]
            minimum = self.minimum[index]
            value = max(-maximum, min(maximum, value))
            if value != 0.0 and abs(value) < minimum:
                value = minimum if value > 0.0 else -minimum

            delta = value - previous[index]
            delta = max(
                self.acceleration_min * time_step,
                min(self.acceleration_max * time_step, delta),
            )
            value = previous[index] + delta
            value = max(-maximum, min(maximum, value))
            limited.append(value)
        return limited[0], limited[1]


class Controller:
    """控制器：接收期望控制量，加入噪声，并保存机器人反馈。

    当前默认控制量为地图坐标系下的 `(vx, vy)`，单位为 m/s。
    """

    def __init__(
        self,
        robot: Robot,
        time_step: float,
        speed_noise_std: float = 0.01,
        seed: int | None = None,
        kinematics: Kinematics | None = None,
        limits: ControlLimits | None = None,
    ):
        """创建控制器。

        ``speed_noise_std`` 是速度高斯噪声的标准差。设置 ``seed`` 后可以
        复现实验结果。
        """
        if time_step <= 0:
            raise ValueError("time_step must be greater than zero")
        if speed_noise_std < 0:
            raise ValueError("speed noise standard deviation must not be negative")

        self.robot = robot
        self.time_step = time_step
        self.speed_noise_std = float(speed_noise_std)
        self._random = random.Random(seed)
        self.kinematics = kinematics or HolonomicKinematics()
        self.limits = limits
        self._previous_command = (0.0, 0.0)
        self._expected_first = 0.0
        self._expected_second = 0.0
        self._feedback_first = 0.0
        self._feedback_second = 0.0

    def set_control(self, first: float, second: float) -> None:
        """设置下一次仿真周期使用的两个控制量。

        该函数只保存期望值，不会立即移动机器人；机器人会在
        :meth:`step` 或 ``Simulator.step`` 时移动。
        """
        self._expected_first = float(first)
        self._expected_second = float(second)

    def set_kinematics(self, kinematics: Kinematics) -> None:
        """替换控制指令所使用的运动学模型。"""
        self.kinematics = kinematics

    def get_control(self) -> tuple[float, float]:
        """获取当前期望的两个控制量。"""
        return self._expected_first, self._expected_second

    def step(
        self, can_move: Callable[[tuple[float, float, float]], bool] | None = None
    ) -> tuple[float, float]:
        """执行一个仿真周期，并返回机器人实际执行的两个控制量。

        ``can_move`` 是可选的碰撞检查函数，接收预测位姿并返回是否允许移动。
        """
        command = self.kinematics.add_noise(
            self.get_control(),
            self._random,
            self.speed_noise_std,
        )
        if self.limits is not None:
            command = self.limits.apply(
                command,
                self._previous_command,
                self.time_step,
            )
        next_pose = self.kinematics.predict_pose(
            self.robot.pose, command, self.time_step
        )
        if can_move is not None and not can_move(next_pose):
            command = self.kinematics.blocked_command(command)
            next_pose = self.kinematics.predict_pose(
                self.robot.pose, command, self.time_step
            )

        self._feedback_first, self._feedback_second = command
        self._previous_command = command
        self.robot._commit_pose(next_pose)
        return self.get_feedback()

    def get_feedback(self) -> tuple[float, float]:
        """获取最近一个仿真周期实际执行的两个控制量。"""
        return self._feedback_first, self._feedback_second

    # Compatibility aliases for the original API.
    set_target = set_control
    get_fdb = get_feedback
