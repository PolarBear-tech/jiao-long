from pathlib import Path
from typing import TypeAlias

import numpy as np

MapInput: TypeAlias = np.ndarray | str | Path


class Map:
    """带现实尺寸信息的二维黑白栅格地图。

    白色区域为可行区域，黑色区域为障碍物。``resolution`` 是米/像素，
    ``origin`` 是图片左下角像素对应的世界坐标。世界坐标 x 向右、y 向上。
    """

    def __init__(
        self,
        source: MapInput,
        resolution: float = 1.0,
        origin: tuple[float, float] = (0.0, 0.0),
        threshold: int = 128,
    ):
        """创建地图。

        ``source`` 可以是 ``np.ndarray``、PGM/PNG 图片路径；灰度值大于等于
        ``threshold`` 的像素会被视为可行区域。``data[row, column]`` 为
        ``True`` 表示该像素可行。
        """
        if resolution <= 0:
            raise ValueError("resolution must be greater than zero")
        if not 0 <= threshold <= 255:
            raise ValueError("threshold must be between 0 and 255")
        if len(origin) != 2:
            raise ValueError("origin must contain exactly two values")

        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))

        data = self._read(source)
        if data.ndim == 3:
            if data.shape[2] not in (3, 4):
                raise ValueError("map image must have 3 or 4 channels")
            data = data[..., :3].mean(axis=2)
        if data.ndim != 2:
            raise ValueError("map must be a 2D grayscale array or an image")
        if data.size == 0:
            raise ValueError("map must not be empty")

        self.display_data = np.asarray(data, dtype=np.uint8).copy()
        self.data = (
            data.astype(bool, copy=True)
            if data.dtype == bool
            else np.asarray(data >= threshold, dtype=bool)
        )

    @staticmethod
    def _read(source: MapInput) -> np.ndarray:
        if isinstance(source, np.ndarray):
            return source
        path = Path(source)
        if path.suffix.lower() == ".pgm":
            return Map._read_pgm(path)
        try:
            from PIL import Image
        except ImportError as error:
            raise ImportError("reading map images requires Pillow") from error
        with Image.open(source) as image:
            return np.asarray(image.convert("L"))

    @staticmethod
    def _read_pgm(path: Path) -> np.ndarray:
        """Read an ASCII PGM image without requiring an image package."""
        tokens = []
        for line in path.read_text(encoding="ascii").splitlines():
            tokens.extend(line.split("#", 1)[0].split())
        if len(tokens) < 4 or tokens[0] != "P2":
            raise ValueError("only ASCII PGM (P2) images are supported without Pillow")
        width, height, max_value = map(int, tokens[1:4])
        values = np.asarray([int(value) for value in tokens[4:]], dtype=np.float32)
        if width <= 0 or height <= 0 or max_value <= 0 or values.size != width * height:
            raise ValueError("invalid PGM image dimensions or pixel data")
        return np.rint(values * (255.0 / max_value)).astype(np.uint8).reshape(height, width)

    @property
    def shape(self) -> tuple[int, int]:
        """返回地图数组形状 ``(height, width)``，单位为像素。"""
        return self.data.shape

    @property
    def size_meters(self) -> tuple[float, float]:
        """返回地图现实尺寸 ``(width, height)``，单位为米。"""
        height, width = self.data.shape
        return width * self.resolution, height * self.resolution

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """将世界坐标米转换为图片索引 ``(column, row)``。"""
        height, _ = self.data.shape
        column = int(np.floor((x - self.origin[0]) / self.resolution))
        from_bottom = int(np.floor((y - self.origin[1]) / self.resolution))
        row = height - 1 - from_bottom
        return column, row

    def grid_to_world(self, column: int, row: int) -> tuple[float, float]:
        """返回栅格中心的世界坐标 ``(x, y)``，单位为米。"""
        height, width = self.data.shape
        if not 0 <= row < height or not 0 <= column < width:
            raise IndexError("grid coordinate is outside the map")
        x = self.origin[0] + (column + 0.5) * self.resolution
        y = self.origin[1] + (height - row - 0.5) * self.resolution
        return x, y

    def is_free(self, x: float, y: float) -> bool:
        """查询世界坐标 ``(x, y)`` 所在的栅格是否可行。越界返回 False。"""
        column, row = self.world_to_grid(x, y)
        return self._is_free_cell(column, row)

    def raycast(
        self,
        x: float,
        y: float,
        angle: float,
        max_range: float,
        step: float | None = None,
    ) -> float:
        """沿世界坐标射线查询到障碍物或地图边界的距离。

        ``angle`` 使用世界坐标系弧度，``max_range`` 和返回值单位为米。
        该接口是二维雷达的基础查询，命中障碍物时返回首次命中距离，
        没有命中时返回 ``max_range``。
        """
        if max_range < 0:
            raise ValueError("max_range must not be negative")
        sample_step = step if step is not None else self.resolution / 4.0
        if sample_step <= 0:
            raise ValueError("step must be greater than zero")
        distance = 0.0
        while distance <= max_range:
            sample_x = x + distance * np.cos(angle)
            sample_y = y + distance * np.sin(angle)
            if not self.is_free(sample_x, sample_y):
                return distance
            distance += sample_step
        return float(max_range)

    def _is_free_circle(self, x: float, y: float, radius: float) -> bool:
        """供 Simulator 内部进行圆形机器人碰撞检查。"""

        if radius < 0:
            raise ValueError("radius must not be negative")

        height, width = self.data.shape
        map_width, map_height = self.size_meters
        min_x, max_x = self.origin[0], self.origin[0] + map_width
        min_y, max_y = self.origin[1], self.origin[1] + map_height
        if x - radius < min_x or x + radius > max_x or y - radius < min_y or y + radius > max_y:
            return False

        min_column = int(np.floor((x - radius - self.origin[0]) / self.resolution))
        max_column = int(
            np.floor(np.nextafter((x + radius - self.origin[0]) / self.resolution, -np.inf))
        )
        min_row_from_bottom = int(np.floor((y - radius - self.origin[1]) / self.resolution))
        max_row_from_bottom = int(
            np.floor(np.nextafter((y + radius - self.origin[1]) / self.resolution, -np.inf))
        )

        for from_bottom in range(min_row_from_bottom, max_row_from_bottom + 1):
            row = height - 1 - from_bottom
            for column in range(min_column, max_column + 1):
                cell_min_x = self.origin[0] + column * self.resolution
                cell_max_x = cell_min_x + self.resolution
                cell_min_y = self.origin[1] + from_bottom * self.resolution
                cell_max_y = cell_min_y + self.resolution
                closest_x = min(max(x, cell_min_x), cell_max_x)
                closest_y = min(max(y, cell_min_y), cell_max_y)
                touches_cell = (closest_x - x) ** 2 + (closest_y - y) ** 2 <= radius**2
                if touches_cell and not self._is_free_cell(column, row):
                    return False
        return True

    def _is_free_cell(self, column: int, row: int) -> bool:
        """查询图片索引对应的栅格是否可行；越界返回 False。"""
        height, width = self.data.shape
        return 0 <= row < height and 0 <= column < width and bool(self.data[row, column])
