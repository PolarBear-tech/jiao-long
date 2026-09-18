from collections.abc import Callable

from sim.controller import ControlLimits, Controller
from sim.kinematics import Kinematics


class Robot:
    def __init__(
        self,
        pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
        radius: float = 0.05,
        time_step: float = 0.05,
        speed_noise_std: float = 0.01,
        seed: int | None = None,
        kinematics: Kinematics | None = None,
        limits: ControlLimits | None = None,
    ):
        """创建机器人。

        ``pose`` 为 ``(x, y, yaw)``，位置单位是米，朝向单位是弧度；
        ``radius`` 是机器人圆形碰撞半径，单位是米。
        """
        if radius <= 0:
            raise ValueError("radius must be greater than zero")
        self.radius = float(radius)
        self.__pose = (0.0, 0.0, 0.0)
        self.reset(pose)
        self.controller = Controller(
            self,
            time_step,
            speed_noise_std=speed_noise_std,
            seed=seed,
            kinematics=kinematics,
            limits=limits,
        )

    def move(
        self,
        first: float,
        second: float,
        time_step: float,
        can_move: Callable[[tuple[float, float, float]], bool] | None = None,
    ) -> tuple[float, float]:
        """按照给定的两个控制量移动，并返回实际执行的两个控制量。

        如果 ``can_move`` 判定下一位姿碰撞，机器人不会平移，但仍可以
        原地旋转，因此反馈速度为 0，角速度仍然有效。
        """
        self.controller.time_step = float(time_step)
        self.controller.set_control(first, second)
        return self.controller.step(can_move)

    def _commit_pose(self, pose: tuple[float, float, float]) -> None:
        self.__pose = tuple(float(value) for value in pose)

    @property
    def pose(self) -> tuple[float, float, float]:
        """当前位姿 ``(x, y, yaw)``，位置单位米，角度单位弧度。"""
        return self.__pose

    def reset(self, pose: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        """将机器人位姿重置为 ``(x, y, yaw)``。"""
        if len(pose) != 3:
            raise ValueError("pose must contain exactly three values")
        self.__pose = pose
