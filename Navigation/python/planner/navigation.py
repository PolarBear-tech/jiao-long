from dataclasses import dataclass

from planner.planner import Controller as PathController
from planner.planner import PathPlanner
from sim.map import Map
from sim.simulator import Simulator


@dataclass
class NavigationContext:
    """自动导航阶段之间共享的初始化结果。"""

    planning_map: Map
    planner: PathPlanner
    controller: PathController


def nav_init(sim: Simulator) -> NavigationContext:
    """自动导航初始化，只在仿真开始前调用一次。"""
    if sim.map is None:
        raise ValueError("navigation requires a map")
    planning_map = sim.map
    sim.set_planning_map(planning_map)
    return NavigationContext(
        planning_map=planning_map,
        planner=PathPlanner(planning_map, sim),
        controller=PathController(planning_map, sim),
    )


def nav_run(sim: Simulator, context: NavigationContext) -> None:
    """自动导航周期回调：读取状态、规划路径并设置控制量。"""
    goal = sim.get_goal()
    if goal is None:
        sim.set_display_path([])
        sim.set_control(0.0, 0.0)
        return
    sim.set_planning_map(context.planning_map)
    context.planner.update_goal(goal)
    path = context.planner.plan()
    sim.set_display_path(path)

    vx, vy = follow_path(sim, path, context.controller)
    sim.set_control(vx, vy)


def follow_path(
    sim: Simulator,
    path: list[tuple[float, float]],
    controller: PathController,
) -> tuple[float, float]:
    """根据路径计算下一周期的地图坐标系速度 ``(vx, vy)``。"""
    controller.set_path(path)
    return controller.plan()
