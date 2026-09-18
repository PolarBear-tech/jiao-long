from sim.map import Map
from sim.simulator import Simulator


class Planner:
    def __init__(self, robo_map: Map, sim: Simulator) -> None:
        self.map = robo_map
        self.sim = sim


class PathPlanner(Planner):
    def __init__(self, robo_map: Map, sim: Simulator) -> None:
        super().__init__(robo_map, sim)
        self.goal: tuple[float, float, float] = self.sim.robot.pose

    def update_goal(self, goal: tuple[float, float, float]) -> None:
        """更新规划器使用的固定目标位姿。"""
        self.goal = goal

    def plan(self) -> list[tuple[float, float]]:
        """返回世界坐标路径点列表，单位为米。

        路径点格式为 ``[(x1, y1), (x2, y2), ...]``，Simulator 会按实际
        世界坐标绘制路径，而不是将路径强制吸附到栅格中心。
        """
        raise NotImplementedError


class Controller(Planner):
    def __init__(self, robo_map: Map, sim: Simulator) -> None:
        super().__init__(robo_map, sim)
        self.path: list[tuple[float, float]] = []

    def set_path(self, path: list[tuple[float, float]]) -> None:
        self.path = path

    def plan(self) -> tuple[float, float]:
        """返回地图坐标系下的 ``(vx, vy)`` 速度。"""
        raise NotImplementedError
