import math
import os
import time
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from sim.controller import ControlLimits, Controller
from sim.kinematics import Kinematics
from sim.map import Map, MapInput
from sim.robot import Robot


ControlCallback = Callable[["Simulator"], None]
WorldPoint = tuple[float, float]
GOAL_TOLERANCE = 0.25
GOAL_CLOSE_DELAY = 10.0


@dataclass
class _RobotAgent:
    """Internal pairing of a robot and its controller."""

    robot: Robot
    controller: Controller


class Simulator:
    """提供给上层控制软件使用的仿真环境。

    pygame 窗口、地图、机器人和控制器均由该类管理。上层软件主要通过
    ``set_control``、``get_feedback``、``get_pose`` 和 ``step`` 交互。
    """

    def __init__(
        self,
        time_step: float,
        map_data: MapInput | Map | None = None,
        map_resolution: float = 1.0,
        map_origin: tuple[float, float] = (0.0, 0.0),
        speed_noise_std: float = 0.01,
        seed: int | None = None,
        kinematics: Kinematics | None = None,
        control_config: dict[str, Any] | None = None,
        robot_radius: float = 0.05,
        render: bool = True,
        show_lidar: bool = True,
        show_planning_map: bool = False,
        window_scale: int | None = None,
        goal: tuple[float, float] | tuple[float, float, float] | None = None,
        display_only: bool = False,
    ):
        """创建仿真环境。

        ``map_data`` 可以是 ``Map``、``np.ndarray`` 或地图文件路径；
        ``map_resolution`` 为米/像素，``map_origin`` 为地图左下角世界坐标。
        """
        if time_step <= 0:
            raise ValueError("time_step must be greater than zero")
        if window_scale is not None and window_scale <= 0:
            raise ValueError("window_scale must be greater than zero")

        self.time_step = float(time_step)
        self.control_limits = self._parse_control_limits(control_config)
        self.map = (
            map_data
            if isinstance(map_data, Map)
            else Map(map_data, resolution=map_resolution, origin=map_origin)
            if map_data is not None
            else None
        )
        self.robots: dict[str, Robot] = {}
        self.controllers: dict[str, Controller] = {}
        self._agents: dict[str, _RobotAgent] = {}
        self.add_robot(
            name="robot",
            pose=(0.0, 0.0, 0.0),
            radius=robot_radius,
            speed_noise_std=speed_noise_std,
            seed=seed,
            kinematics=kinematics,
        )
        self.robot = self.robots["robot"]
        self.controller = self.controllers["robot"]
        self.render_enabled = render
        self.show_lidar = bool(show_lidar)
        self.show_planning_map = bool(show_planning_map)
        self.display_only = bool(display_only)
        self.window_scale = window_scale
        self.is_running = False
        self._pygame = None
        self._screen = None
        self._font = None
        self._clock = None
        self._map_surface = None
        self._grid_layer = None
        self._draw_frame = 0
        self._lidar_display_points: dict[str, list[tuple[int, int]]] = {}
        self._display_path: list[WorldPoint] = []
        self._display_map_override: Map | None = None
        self._planning_map: Map | None = None
        self._goal: tuple[float, float, float] | None = None
        self._goal_reached = False
        self._goal_reached_at: float | None = None
        self._goal_close_deadline: float | None = None
        self._manual_keys: set[int] = set()
        if goal is not None:
            if len(goal) not in (2, 3):
                raise ValueError("goal must contain (x, y) or (x, y, yaw)")
            yaw = float(goal[2]) if len(goal) == 3 else 0.0
            self._goal = (float(goal[0]), float(goal[1]), yaw)

    def add_robot(
        self,
        name: str | None = None,
        pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
        radius: float = 0.05,
        speed_noise_std: float = 0.01,
        seed: int | None = None,
        kinematics: Kinematics | None = None,
    ) -> str:
        """添加机器人并返回其名称。

        每个机器人拥有独立的 ``Robot`` 和 ``Controller``。名称用于之后
        调用 ``set_control``、``get_feedback``、``get_pose`` 和 ``get_lidar``。
        第一个默认机器人名称为 ``"robot"``。
        """
        if name is None:
            index = 1
            while f"robot_{index}" in self._agents:
                index += 1
            name = f"robot_{index}"
        if not name or name in self._agents:
            raise ValueError(f"robot name is invalid or already exists: {name!r}")

        robot = Robot(
            pose=pose,
            radius=radius,
            time_step=self.time_step,
            speed_noise_std=speed_noise_std,
            seed=seed,
            kinematics=kinematics,
            limits=self.control_limits,
        )
        self.robots[name] = robot
        self.controllers[name] = robot.controller
        self._agents[name] = _RobotAgent(robot, robot.controller)
        return name

    @staticmethod
    def _parse_control_limits(
        control_config: dict[str, Any] | None,
    ) -> ControlLimits | None:
        if control_config is None:
            return None
        profile = control_config.get("dynamics")
        if not isinstance(profile, dict):
            raise ValueError("control config must define a dynamics mapping")

        try:
            maximum = (float(profile["vx_max"]), float(profile["vy_max"]))
            minimum = (float(profile["vx_min"]), float(profile["vy_min"]))
            acceleration_max = float(profile["acc_max"])
            acceleration_min = float(profile["acc_min"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("invalid control limits in control.dynamics") from error
        return ControlLimits(
            maximum=maximum,  # type: ignore[arg-type]
            minimum=minimum,  # type: ignore[arg-type]
            acceleration_max=acceleration_max,
            acceleration_min=acceleration_min,
        )

    def remove_robot(self, name: str) -> None:
        """移除指定名称的机器人；默认机器人不能被移除。"""
        if name == "robot":
            raise ValueError("the default robot cannot be removed")
        if name not in self._agents:
            raise KeyError(f"unknown robot: {name}")
        del self._agents[name]
        del self.robots[name]
        del self.controllers[name]

    def get_robot(self, name: str = "robot") -> Robot:
        """获取指定名称的机器人对象。"""
        try:
            return self.robots[name]
        except KeyError as error:
            raise KeyError(f"unknown robot: {name}") from error

    def robot_names(self) -> tuple[str, ...]:
        """返回当前环境中的所有机器人名称。"""
        return tuple(self._agents)

    def set_control(self, first: float, second: float, robot_id: str = "robot") -> None:
        """设置机器人当前运动学模型所需的两个控制量。"""
        self.controllers[robot_id].set_control(first, second)

    def get_control(self, robot_id: str = "robot") -> tuple[float, float]:
        """获取当前设置的两个期望控制量。"""
        return self.controllers[robot_id].get_control()

    def get_feedback(self, robot_id: str = "robot") -> tuple[float, float]:
        """获取最近一个仿真周期的两个实际控制量。"""
        return self.controllers[robot_id].get_feedback()

    def get_pose(self, robot_id: str = "robot") -> tuple[float, float, float]:
        """获取机器人当前世界位姿 ``(x, y, yaw)``。"""
        return self.robots[robot_id].pose

    def get_goal(self) -> tuple[float, float, float] | None:
        """获取启动时从配置文件读取的固定目标点。"""
        return self._goal

    def set_display_path(self, path: Iterable[WorldPoint]) -> None:
        """设置要在 pygame 中显示的世界坐标路径。

        路径可以来自任意导航算法，不要求继承或绑定 PathPlanner；
        每个点格式为 ``(x, y)``，单位为米。
        """
        self._display_path = [(float(point[0]), float(point[1])) for point in path]

    def set_planning_map(self, planning_map: Map | None) -> None:
        """设置仅用于显示的规划地图，不影响碰撞检测或雷达。"""
        if planning_map is not None and not isinstance(planning_map, Map):
            raise TypeError("planning_map must be a Map or None")
        self._planning_map = planning_map

    def set_display_map(self, display_map: Map | None) -> None:
        """设置固定的显示地图，不影响碰撞检测、雷达或 C++ 地图数据。"""
        if display_map is not None and not isinstance(display_map, Map):
            raise TypeError("display_map must be a Map or None")
        self._display_map_override = display_map

    def _display_map(self) -> Map:
        if self.map is None:
            raise RuntimeError("display requires map_data")
        if self.show_planning_map and self._planning_map is not None:
            return self._planning_map
        if self._display_map_override is not None:
            return self._display_map_override
        return self.map

    def get_display_path(self) -> list[WorldPoint]:
        """获取当前显示的世界坐标路径副本。"""
        return list(self._display_path)

    def get_manual_control(
        self,
        speed: float = 1.0,
        lateral_speed: float = 1.0,
    ) -> tuple[float, float]:
        """读取 WASD 键盘输入并返回地图坐标系下的 ``(vx, vy)``。

        该函数只读取输入，不会自动设置控制量；调用方仍需把返回值传给
        :meth:`set_control`。A/D 控制地图 x 轴，W/S 控制地图 y 轴。
        """
        if self._pygame is None:
            raise RuntimeError("manual control requires the pygame renderer")
        x_velocity = (
            -lateral_speed
            if self._pygame.K_a in self._manual_keys
            else lateral_speed
            if self._pygame.K_d in self._manual_keys
            else 0.0
        )
        y_velocity = (
            speed
            if self._pygame.K_w in self._manual_keys
            else -speed
            if self._pygame.K_s in self._manual_keys
            else 0.0
        )
        return x_velocity, y_velocity

    def get_lidar(
        self,
        robot_id: str = "robot",
        count: int = 360,
        max_range: float = 3.0,
        fov: float = 2 * math.pi,
    ) -> list[float]:
        """获取二维雷达距离数组，角度相对机器人朝向均匀分布。

        返回数组第 ``i`` 项对应的角度为
        ``-fov / 2 + i * fov / count``，单位为弧度；距离单位为米。
        当前雷达读取静态地图，不读取其他机器人。
        """
        if self.map is None:
            raise RuntimeError("lidar requires map_data")
        if count <= 0 or max_range < 0 or fov <= 0:
            raise ValueError("count must be positive, max_range non-negative, fov positive")
        x, y, yaw = self.robots[robot_id].pose
        return [
            self.map.raycast(
                x,
                y,
                yaw - fov / 2 + index * fov / count,
                max_range,
            )
            for index in range(count)
        ]

    def step(self, robot_id: str = "robot") -> tuple[float, float]:
        """推进一个仿真周期，并返回两个实际控制量。"""
        if robot_id not in self._agents:
            raise KeyError(f"unknown robot: {robot_id}")
        feedback: dict[str, tuple[float, float]] = {}
        for name, agent in self._agents.items():
            feedback[name] = agent.robot.controller.step(self._can_move(name))
        return feedback[robot_id]

    def _can_move(self, robot_id: str) -> Callable[[tuple[float, float, float]], bool] | None:
        """创建单个机器人的地图和其他机器人碰撞检查函数。"""
        if self.map is None and len(self._agents) == 1:
            return None
        robot = self.robots[robot_id]

        def can_move(pose: tuple[float, float, float]) -> bool:
            if self.map is not None and not self.map._is_free_circle(
                pose[0], pose[1], robot.radius
            ):
                return False
            for other_id, other in self.robots.items():
                if other_id == robot_id:
                    continue
                other_x, other_y, _ = other.pose
                distance = math.hypot(pose[0] - other_x, pose[1] - other_y)
                if distance < robot.radius + other.radius:
                    return False
            return True

        return can_move

    def run(self, callback: ControlCallback | None = None, duration: float | None = None) -> None:
        """在当前线程运行仿真环境。

        每个周期结束后调用一次 ``callback(simulator)``。回调中可以读取本周期
        反馈，并设置下一周期控制量。开启渲染时，关闭 pygame 窗口会停止仿真。
        """
        if duration is not None and duration < 0:
            raise ValueError("duration must be non-negative or None")
        if self.map is None and self.render_enabled:
            raise ValueError("rendering requires map_data")

        if self.render_enabled:
            self._init_renderer()
        self.is_running = True
        elapsed = 0.0
        try:
            while self.is_running and (duration is None or elapsed < duration):
                if self.render_enabled:
                    self._handle_events()
                if not self.is_running:
                    break

                if not self._goal_reached and self._has_reached_goal():
                    self._mark_goal_reached()

                if self._goal_reached:
                    self.set_control(0.0, 0.0)
                else:
                    self.step()
                    if self._has_reached_goal():
                        self._mark_goal_reached()
                elapsed += self.time_step
                if callback is not None and not self._goal_reached:
                    callback(self)
                if self.render_enabled:
                    self._draw()
                    if self._clock is not None:
                        self._clock.tick(max(1, round(1.0 / self.time_step)))
                else:
                    time.sleep(self.time_step)
                if (
                    self._goal_close_deadline is not None
                    and time.monotonic() >= self._goal_close_deadline
                ):
                    self.stop()
        finally:
            self.is_running = False
            if self.render_enabled:
                self._close_renderer()

    def stop(self) -> None:
        """停止持续运行的仿真循环。"""
        self.is_running = False

    def _has_reached_goal(self) -> bool:
        if self._goal is None:
            return False
        x, y, _ = self.robot.pose
        goal_x, goal_y, _ = self._goal
        return math.hypot(x - goal_x, y - goal_y) <= GOAL_TOLERANCE

    def _mark_goal_reached(self) -> None:
        if self._goal_reached:
            return
        self._goal_reached = True
        self._goal_reached_at = time.monotonic()
        self._goal_close_deadline = self._goal_reached_at + GOAL_CLOSE_DELAY
        self.set_control(0.0, 0.0)

    def _init_renderer(self) -> None:
        import warnings

        with warnings.catch_warnings():
            # pygame 2.6.1 imports pkg_resources internally; this warning is
            # from pygame itself, not from the simulator code.
            warnings.filterwarnings(
                "ignore",
                message="pkg_resources is deprecated as an API.*",
                category=UserWarning,
                module="pygame.pkgdata",
            )
            import pygame

        world_map = self._display_map()
        pygame.init()
        height, width = world_map.shape
        panel_width = 280
        if self.window_scale is None:
            display_info = pygame.display.Info()
            horizontal_margin = 40
            vertical_margin = 80
            available_width = max(1, display_info.current_w - panel_width - horizontal_margin)
            available_height = max(1, display_info.current_h - vertical_margin)
            self.window_scale = max(
                1,
                min(available_width // width, available_height // height),
            )
        self._pygame = pygame
        self._screen = pygame.display.set_mode(
            (width * self.window_scale + panel_width, height * self.window_scale)
        )
        self._map_surface = pygame.Surface(
            (width * self.window_scale, height * self.window_scale)
        )
        for row in range(height):
            for column in range(width):
                value = int(world_map.display_data[row, column])
                if value <= 1:
                    value = 242 if world_map.data[row, column] else 20
                color = (value, value, value)
                pygame.draw.rect(
                    self._map_surface,
                    color,
                    (
                        column * self.window_scale,
                        row * self.window_scale,
                        self.window_scale,
                        self.window_scale,
                    ),
                )

        self._grid_layer = pygame.Surface(self._map_surface.get_size(), pygame.SRCALPHA)
        for column in range(width + 1):
            pygame.draw.line(
                self._grid_layer,
                (150, 150, 150, 55),
                (column * self.window_scale, 0),
                (column * self.window_scale, height * self.window_scale),
            )
        for row in range(height + 1):
            pygame.draw.line(
                self._grid_layer,
                (150, 150, 150, 55),
                (0, row * self.window_scale),
                (width * self.window_scale, row * self.window_scale),
            )
        pygame.display.set_caption("PyRobo Simulator")
        self._font = pygame.font.Font(None, 24)
        self._clock = pygame.time.Clock()

    def _handle_events(self) -> None:
        for event in self._pygame.event.get():
            if event.type == self._pygame.QUIT:
                self.stop()
            elif event.type == self._pygame.KEYDOWN:
                self._manual_keys.add(event.key)
                if event.key == self._pygame.K_ESCAPE:
                    self.stop()
            elif event.type == self._pygame.KEYUP:
                self._manual_keys.discard(event.key)

    def _world_to_screen(self, point: WorldPoint) -> tuple[int, int]:
        """将米制世界坐标转换为 pygame 窗口像素坐标。"""
        world_map = self._display_map()
        x, y = point
        height, _ = world_map.shape
        return (
            round((x - world_map.origin[0]) / world_map.resolution * self.window_scale),
            round((height - (y - world_map.origin[1]) / world_map.resolution) * self.window_scale),
        )

    def _draw(self) -> None:
        pygame = self._pygame
        world_map = self._display_map()
        screen = self._screen
        cell = self.window_scale
        height, width = world_map.shape
        screen.fill((35, 40, 48))
        screen.blit(self._map_surface, (0, 0))

        if len(self._display_path) >= 2:
            pygame.draw.lines(
                screen,
                (40, 120, 230),
                False,
                [self._world_to_screen(point) for point in self._display_path],
                4,
            )
        for point in self._display_path:
            pygame.draw.circle(screen, (40, 120, 230), self._world_to_screen(point), 4)

        if self._goal is not None:
            pygame.draw.circle(screen, (255, 140, 0), self._world_to_screen(self._goal[:2]), 5)

        map_width_m, map_height_m = world_map.size_meters
        lidar_count = 360
        lidar_max_range = 3.0
        if self.show_lidar and self._draw_frame % 2 == 0:
            self._lidar_display_points = {}
            for robot_id, robot in self.robots.items():
                x, y, yaw = robot.pose
                lidar_ranges = self.get_lidar(
                    robot_id=robot_id,
                    count=lidar_count,
                    max_range=lidar_max_range,
                )
                points = []
                for index, distance in enumerate(lidar_ranges):
                    if distance >= lidar_max_range:
                        continue
                    angle = yaw - math.pi + index * 2 * math.pi / lidar_count
                    hit_point = (
                        x + distance * math.cos(angle),
                        y + distance * math.sin(angle),
                    )
                    hit_screen = self._world_to_screen(hit_point)
                    if 0 <= hit_screen[0] < width * cell and 0 <= hit_screen[1] < height * cell:
                        points.append(hit_screen)
                self._lidar_display_points[robot_id] = points

        if self.show_lidar:
            for points in self._lidar_display_points.values():
                for point in points:
                    pygame.draw.circle(screen, (60, 220, 240), point, 2)

        for index, (robot_id, robot) in enumerate(self.robots.items()):
            x, y, yaw = robot.pose
            screen_x = (x - world_map.origin[0]) / map_width_m * width * cell
            screen_y = height * cell - (y - world_map.origin[1]) / map_height_m * height * cell
            if 0 <= screen_x < width * cell and 0 <= screen_y < height * cell:
                center = (round(screen_x), round(screen_y))
                radius_pixels = max(1, round(robot.radius / world_map.resolution * cell))
                color = (220, 55, 55) if index == 0 else (230, 130, 35)
                pygame.draw.circle(screen, color, center, radius_pixels)
                heading_length = max(10, round(radius_pixels * 1.4))
                heading_end = (
                    round(center[0] + heading_length * math.cos(yaw)),
                    round(center[1] - heading_length * math.sin(yaw)),
                )
                heading_width = max(2, min(6, round(radius_pixels * 0.18)))
                pygame.draw.line(
                    screen,
                    (255, 245, 180),
                    center,
                    heading_end,
                    heading_width,
                )

        # Keep grid lines as a low-contrast visual aid instead of the main map.
        screen.blit(self._grid_layer, (0, 0))

        if self._goal is not None:
            goal_screen = self._world_to_screen(self._goal[:2])
            goal_label = self._font.render("GOAL", True, (255, 140, 0))
            screen.blit(goal_label, (goal_screen[0] + 12, goal_screen[1] - 12))
        for robot_id, robot in self.robots.items():
            robot_screen = self._world_to_screen(robot.pose[:2])
            robot_label = self._font.render(robot_id.upper(), True, (255, 130, 130))
            screen.blit(robot_label, (robot_screen[0] + 12, robot_screen[1] - 12))

        panel_x = width * cell
        pygame.draw.rect(screen, (35, 40, 48), (panel_x, 0, 280, screen.get_height()))
        x, y, yaw = self.robot.pose
        robot_column, robot_row = world_map.world_to_grid(x, y)
        goal_lines = []
        if self._goal is not None:
            goal_x, goal_y, goal_yaw = self._goal
            goal_column, goal_row = world_map.world_to_grid(goal_x, goal_y)
            goal_lines = [
                "",
                "GOAL (orange)",
                f"map x: {goal_x:.3f} m",
                f"map y: {goal_y:.3f} m",
                f"grid column: {goal_column}",
                f"grid row: {goal_row}",
                f"goal yaw: {goal_yaw:.3f} rad",
            ]
        if self.display_only:
            lines = [
                "PYROBO DISPLAY",
                "mode: display only",
                "",
                "DISPLAY MAP",
                f"resolution: {world_map.resolution:.3f} m/cell",
                f"size: {world_map.size_meters[0]:.2f} x {world_map.size_meters[1]:.2f} m",
                "",
                "ROBOT (red)",
                f"map x: {x:.3f} m",
                f"map y: {y:.3f} m",
                f"grid column: {robot_column}",
                f"grid row: {robot_row}",
                f"yaw: {yaw:.3f} rad",
                f"radius: {self.robot.radius:.3f} m",
                *goal_lines,
                "",
                "planning: disabled",
                "control: disabled",
                "ESC  close",
            ]
        else:
            expected_vx, expected_vy = self.get_control()
            feedback_vx, feedback_vy = self.get_feedback()
            lines = [
                "PYROBO SIMULATOR",
                "mode: active",
                "",
                "MAP",
                f"resolution: {world_map.resolution:.3f} m/cell",
                f"size: {world_map.size_meters[0]:.2f} x {world_map.size_meters[1]:.2f} m",
                "",
                "ROBOT (red)",
                f"map x: {x:.3f} m",
                f"map y: {y:.3f} m",
                f"grid column: {robot_column}",
                f"grid row: {robot_row}",
                f"yaw: {yaw:.3f} rad",
                f"radius: {self.robot.radius:.3f} m",
                *goal_lines,
                "",
                f"command vx: {expected_vx:.3f} m/s",
                f"command vy: {expected_vy:.3f} m/s",
                f"feedback vx: {feedback_vx:.3f} m/s",
                f"feedback vy: {feedback_vy:.3f} m/s",
                f"path points: {len(self._display_path)}",
                "ESC  close",
            ]
        for index, line in enumerate(lines):
            color = (255, 220, 120) if index == 0 else (235, 235, 235)
            screen.blit(self._font.render(line, True, color), (panel_x + 18, 18 + index * 24))

        if self._goal_reached and self._goal_close_deadline is not None:
            remaining = max(0, math.ceil(self._goal_close_deadline - time.monotonic()))
            overlay = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
            overlay.fill((0, 0, 0, 150))
            screen.blit(overlay, (0, 0))
            message_font = self._load_message_font(pygame, 48)
            detail_font = self._load_message_font(pygame, 24)
            message = message_font.render("到达终点", True, (255, 140, 0))
            detail = detail_font.render(
                f"程序将在 {remaining} 秒后关闭", True, (255, 255, 255)
            )
            screen.blit(
                message,
                message.get_rect(center=(screen.get_width() // 2, screen.get_height() // 2 - 24)),
            )
            screen.blit(
                detail,
                detail.get_rect(center=(screen.get_width() // 2, screen.get_height() // 2 + 30)),
            )

        pygame.display.flip()
        self._draw_frame += 1

    @staticmethod
    def _load_message_font(pygame: Any, size: int) -> Any:
        """Load a CJK-capable font without pygame's Windows font enumeration."""
        windows_fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
        candidates = (
            windows_fonts / "msyh.ttc",
            windows_fonts / "simhei.ttf",
            windows_fonts / "simsun.ttc",
            Path("/System/Library/Fonts/PingFang.ttc"),
            Path("/System/Library/Fonts/Hiragino Sans GB.ttc"),
            Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
            Path("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"),
        )
        for candidate in candidates:
            if candidate.is_file():
                try:
                    return pygame.font.Font(str(candidate), size)
                except pygame.error:
                    continue
        return pygame.font.Font(None, size)

    def _close_renderer(self) -> None:
        self._pygame.quit()
        self._pygame = None
        self._screen = None
        self._font = None
        self._clock = None
        self._map_surface = None
        self._grid_layer = None
        self._draw_frame = 0
        self._lidar_display_points.clear()
        self._manual_keys.clear()
