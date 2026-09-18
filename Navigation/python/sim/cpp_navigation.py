"""Load the compiled contestant navigation library."""

from __future__ import annotations

import ctypes
import os
import shutil
from pathlib import Path
from typing import Callable

import numpy as np

from sim.map import Map
from sim.simulator import Simulator


ROOT_DIR = Path(__file__).resolve().parents[2]
DEFAULT_BUILD_DIR = ROOT_DIR / "cpp" / "build"


class _CPoint(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double)]


_GetMap = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8)),
    ctypes.POINTER(ctypes.c_size_t),
)
_GetPose = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.c_char_p,
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
)
_GetRobot = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.c_char_p,
    ctypes.POINTER(ctypes.c_double),
)
_GetGoal = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
)
_GetFeedback = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.c_char_p,
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
)
_SetControl = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.c_char_p,
    ctypes.c_double,
    ctypes.c_double,
)
_GetControl = _GetFeedback
_GetLidar = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.c_char_p,
    ctypes.c_int,
    ctypes.c_double,
    ctypes.c_double,
    ctypes.POINTER(ctypes.c_double),
    ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_size_t),
)
_SetDisplayPath = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.POINTER(_CPoint),
    ctypes.c_size_t,
)


_SetPlanningMap = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_double,
    ctypes.c_double,
    ctypes.c_double,
    ctypes.POINTER(ctypes.c_uint8),
    ctypes.c_size_t,
)


class _CCallbacks(ctypes.Structure):
    _fields_ = [
        ("user_data", ctypes.c_void_p),
        ("get_map", _GetMap),
        ("get_pose", _GetPose),
        ("get_robot", _GetRobot),
        ("get_goal", _GetGoal),
        ("get_feedback", _GetFeedback),
        ("set_control", _SetControl),
        ("get_control", _GetControl),
        ("get_lidar", _GetLidar),
        ("set_display_path", _SetDisplayPath),
        ("set_planning_map", _SetPlanningMap),
    ]


def find_library(build_dir: Path = DEFAULT_BUILD_DIR) -> Path:
    """Find the CMake shared library on Windows, macOS or Linux."""
    candidates = [
        build_dir / "pyrobo_contestant.dll",
        build_dir / "libpyrobo_contestant.dll",
        build_dir / "Release" / "pyrobo_contestant.dll",
        build_dir / "Release" / "libpyrobo_contestant.dll",
        build_dir / "Debug" / "pyrobo_contestant.dll",
        build_dir / "libpyrobo_contestant.dylib",
        build_dir / "libpyrobo_contestant.so",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    expected = ", ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        "compiled C++ navigation library was not found; run build.bat/build.sh first. "
        f"Checked: {expected}"
    )


class CppNavigation:
    """Python-side owner of one C++ navigation context."""

    def __init__(
        self,
        sim: Simulator,
        library_path: Path | None = None,
        exposed_map: Map | None = None,
    ):
        if sim.map is None:
            raise ValueError("C++ navigation requires a map")
        if exposed_map is not None and not isinstance(exposed_map, Map):
            raise TypeError("exposed_map must be a Map or None")

        self._sim = sim
        self._callback_error: str | None = None
        map_for_cpp = exposed_map if exposed_map is not None else sim.map
        self._map_data = map_for_cpp.data.astype("uint8", copy=True, order="C")
        self._map_resolution = map_for_cpp.resolution
        self._map_origin = map_for_cpp.origin
        self._callback_refs: list[Callable[..., object]] = []
        self._dll_directory = None
        if os.name == "nt" and hasattr(os, "add_dll_directory"):
            compiler = shutil.which("c++") or shutil.which("g++")
            if compiler:
                self._dll_directory = os.add_dll_directory(str(Path(compiler).parent))
        self._library = ctypes.CDLL(str(library_path or find_library()))
        self._configure_library()
        self._callbacks = self._make_callbacks()

        error = ctypes.create_string_buffer(4096)
        self._handle = self._library.pyrobo_create_navigation(
            ctypes.byref(self._callbacks), error, len(error)
        )
        if not self._handle:
            raise RuntimeError(error.value.decode("utf-8", errors="replace"))

    def _configure_library(self) -> None:
        self._library.pyrobo_create_navigation.argtypes = [
            ctypes.POINTER(_CCallbacks),
            ctypes.c_char_p,
            ctypes.c_size_t,
        ]
        self._library.pyrobo_create_navigation.restype = ctypes.c_void_p
        self._library.pyrobo_run_navigation.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_size_t,
        ]
        self._library.pyrobo_run_navigation.restype = ctypes.c_int
        self._library.pyrobo_destroy_navigation.argtypes = [ctypes.c_void_p]
        self._library.pyrobo_destroy_navigation.restype = None

    def _safe(self, function: Callable[..., int], default: int = 0) -> Callable[..., int]:
        def wrapped(*args: object) -> int:
            try:
                return int(function(*args))
            except Exception as error:  # ctypes cannot propagate Python exceptions.
                self._callback_error = str(error)
                return default

        self._callback_refs.append(wrapped)
        return wrapped

    @staticmethod
    def _robot_id(value: bytes) -> str:
        return value.decode("utf-8")

    def _make_callbacks(self) -> _CCallbacks:
        def get_map(
            _user: object,
            height: ctypes.POINTER(ctypes.c_int),
            width: ctypes.POINTER(ctypes.c_int),
            resolution: ctypes.POINTER(ctypes.c_double),
            origin_x: ctypes.POINTER(ctypes.c_double),
            origin_y: ctypes.POINTER(ctypes.c_double),
            data: ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8)),
            data_size: ctypes.POINTER(ctypes.c_size_t),
        ) -> int:
            height[0], width[0] = self._map_data.shape
            resolution[0] = self._map_resolution
            origin_x[0], origin_y[0] = self._map_origin
            data[0] = self._map_data.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
            data_size[0] = self._map_data.size
            return 1

        def get_pose(
            _user: object,
            robot_id: bytes,
            x: ctypes.POINTER(ctypes.c_double),
            y: ctypes.POINTER(ctypes.c_double),
            yaw: ctypes.POINTER(ctypes.c_double),
        ) -> int:
            x[0], y[0], yaw[0] = self._sim.get_pose(self._robot_id(robot_id))
            return 1

        def get_robot(_user: object, robot_id: bytes, radius: ctypes.POINTER(ctypes.c_double)) -> int:
            radius[0] = self._sim.get_robot(self._robot_id(robot_id)).radius
            return 1

        def get_goal(
            _user: object,
            x: ctypes.POINTER(ctypes.c_double),
            y: ctypes.POINTER(ctypes.c_double),
            yaw: ctypes.POINTER(ctypes.c_double),
        ) -> int:
            goal = self._sim.get_goal()
            if goal is None:
                return 0
            x[0], y[0], yaw[0] = goal
            return 1

        def get_feedback(
            _user: object,
            robot_id: bytes,
            first: ctypes.POINTER(ctypes.c_double),
            second: ctypes.POINTER(ctypes.c_double),
        ) -> int:
            first[0], second[0] = self._sim.get_feedback(self._robot_id(robot_id))
            return 1

        def set_control(_user: object, robot_id: bytes, first: float, second: float) -> int:
            self._sim.set_control(first, second, self._robot_id(robot_id))
            return 1

        def get_control(
            _user: object,
            robot_id: bytes,
            first: ctypes.POINTER(ctypes.c_double),
            second: ctypes.POINTER(ctypes.c_double),
        ) -> int:
            first[0], second[0] = self._sim.get_control(self._robot_id(robot_id))
            return 1

        def get_lidar(
            _user: object,
            robot_id: bytes,
            count: int,
            max_range: float,
            fov: float,
            output: ctypes.POINTER(ctypes.c_double),
            capacity: int,
            output_size: ctypes.POINTER(ctypes.c_size_t),
        ) -> int:
            if capacity < count:
                return 0
            ranges = self._sim.get_lidar(self._robot_id(robot_id), count, max_range, fov)
            for index, value in enumerate(ranges):
                output[index] = value
            output_size[0] = len(ranges)
            return 1

        def set_display_path(_user: object, points: object, count: int) -> int:
            self._sim.set_display_path(
                [(points[index].x, points[index].y) for index in range(count)]
            )
            return 1

        def set_planning_map(
            _user: object,
            height: int,
            width: int,
            resolution: float,
            origin_x: float,
            origin_y: float,
            data: ctypes.POINTER(ctypes.c_uint8),
            data_size: int,
        ) -> int:
            if height <= 0 or width <= 0 or data_size != height * width:
                return 0
            values = np.ctypeslib.as_array(data, shape=(data_size,)).copy()
            planning_map = Map(
                values.reshape((height, width)).astype(bool),
                resolution=resolution,
                origin=(origin_x, origin_y),
            )
            self._sim.set_planning_map(planning_map)
            return 1

        functions = [
            (_GetMap, get_map, 0),
            (_GetPose, get_pose, 0),
            (_GetRobot, get_robot, 0),
            (_GetGoal, get_goal, -1),
            (_GetFeedback, get_feedback, 0),
            (_SetControl, set_control, 0),
            (_GetControl, get_control, 0),
            (_GetLidar, get_lidar, 0),
            (_SetDisplayPath, set_display_path, 0),
            (_SetPlanningMap, set_planning_map, 0),
        ]
        callbacks = [
            callback_type(self._safe(function, default))
            for callback_type, function, default in functions
        ]
        self._callback_refs.extend(callbacks)
        return _CCallbacks(None, *callbacks)

    def run(self) -> None:
        self._callback_error = None
        error = ctypes.create_string_buffer(4096)
        if not self._library.pyrobo_run_navigation(self._handle, error, len(error)):
            message = self._callback_error or error.value.decode("utf-8", errors="replace")
            raise RuntimeError(message or "C++ navigation failed")

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._library.pyrobo_destroy_navigation(self._handle)
            self._handle = None

    def __del__(self) -> None:
        self.close()
