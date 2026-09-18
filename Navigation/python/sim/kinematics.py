from __future__ import annotations

import random
from abc import ABC, abstractmethod


Pose = tuple[float, float, float]
Command = tuple[float, float]


class Kinematics(ABC):
    """控制指令到下一位姿的转换模型。"""

    @abstractmethod
    def predict_pose(self, pose: Pose, command: Command, time_step: float) -> Pose:
        """根据控制指令预测下一位姿。"""

    @abstractmethod
    def add_noise(
        self,
        command: Command,
        random_source: random.Random,
        speed_noise_std: float,
    ) -> Command:
        """为控制指令添加符合该模型含义的噪声。"""

    @abstractmethod
    def blocked_command(self, command: Command) -> Command:
        """发生碰撞时返回只允许保留的控制指令。"""

class HolonomicKinematics(Kinematics):
    """全向轮模型，指令为地图坐标系下的 ``(vx, vy)``。"""

    def predict_pose(self, pose: Pose, command: Command, time_step: float) -> Pose:
        x, y, yaw = pose
        vx, vy = command
        return (
            x + vx * time_step,
            y + vy * time_step,
            yaw,
        )

    def add_noise(
        self,
        command: Command,
        random_source: random.Random,
        speed_noise_std: float,
    ) -> Command:
        vx, vy = command
        return (
            vx + random_source.gauss(0.0, speed_noise_std),
            vy + random_source.gauss(0.0, speed_noise_std),
        )

    def blocked_command(self, command: Command) -> Command:
        return 0.0, 0.0
