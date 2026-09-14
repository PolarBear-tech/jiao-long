import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PART1 = ROOT / "assets" / "rmuc2026-part1.stl"
PART2 = ROOT / "assets" / "rmuc2026-part2.stl"
PART1_PCD = ROOT / "assets" / "rmuc2026-part1.pcd"
PART2_PCD = ROOT / "assets" / "rmuc2026-part2.pcd"
OUTPUT = ROOT / "assets" / "rmuc2026.stl"
OUTPUT_PCD = ROOT / "assets" / "rmuc2026.pcd"


def rot_x(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def compute_transform() -> np.ndarray:

    # TODO: 补全最关键的坐标变换部分，返回 T^W_B3。

    deg = np.pi / 180.0
    t_w_b0 = np.array([1.0, 2.0, 0.5])
    yaw, pitch, roll = 30.0 * deg, 10.0 * deg, 5.0 * deg

    # TODO: 用 yaw/pitch/roll 计算 B0 在 W 下的姿态。
    r_w_b0 = np.eye(3) @ rot_z(yaw) @ rot_y(pitch) @ rot_x(roll) 
    # 这里是将3x3单位矩阵分别与三个旋转矩阵相乘，得到B0在W下的姿态。

    # TODO: 先绕 B0 自身 Z 轴旋转 +45 度。
    r_w_b2 = r_w_b0 @ rot_z(45.0 * deg)

    # TODO: 再沿 B1 自身 X 轴平移 2 米。
    t_w_b2 = t_w_b0 + r_w_b2 @ np.array([2.0, 0.0, 0.0])
    # r_w_b2 @ np.array([2.0, 0.0, 0.0]) 是把在r_w_b2下的x轴的两米变换成W下的两米 添加到t_w_b0上

    # TODO: 最后绕世界坐标系 W 的 Y 轴旋转 -30 度。
    r_w_b3 = rot_y(-30 * deg) @ r_w_b2
    t_w_b3 = rot_y(-30 * deg) @ t_w_b2

    t_w_b3_matrix = np.eye(4)
    t_w_b3_matrix[:3, :3] = r_w_b3
    t_w_b3_matrix[:3, 3] = t_w_b3
    return t_w_b3_matrix


def triangle_count(stl) -> int:
    stl.seek(80)
    count_bytes = stl.read(4)
    if len(count_bytes) != 4:
        raise ValueError("无效的二进制 STL 文件")
    return struct.unpack("<I", count_bytes)[0]


def transform_triangle(record: bytes, transform: np.ndarray) -> bytes:
    if len(record) != 50:
        raise ValueError("STL 三角形数据不完整")

    vectors = np.frombuffer(record[:48], dtype="<f4").astype(np.float64).reshape(4, 3)
    vectors[0] = transform[:3, :3] @ vectors[0]
    vectors[1:] = vectors[1:] @ transform[:3, :3].T + transform[:3, 3]
    return vectors.astype("<f4").tobytes() + record[48:]


def write_complete_stl(transform: np.ndarray, output: Path = OUTPUT) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with PART1.open("rb") as part1, PART2.open("rb") as part2, output.open(
        "wb"
    ) as result:
        count1, count2 = triangle_count(part1), triangle_count(part2)
        result.write((b"RMUC 2026 reconstructed field" + b" " * 80)[:80])
        result.write(struct.pack("<I", count1 + count2))

        for _ in range(count1):
            result.write(transform_triangle(part1.read(50), transform))
        for _ in range(count2):
            record = part2.read(50)
            if len(record) != 50:
                raise ValueError("STL 三角形数据不完整")
            result.write(record)


def read_xyz_pcd(path: Path) -> np.ndarray:
    """Read the binary XYZ PCD files supplied as P1 auxiliary material."""
    with path.open("rb") as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD 文件缺少 DATA binary 头")
            header.append(line)
            if line.strip().upper() == b"DATA BINARY":
                break
        fields = next(line for line in header if line.upper().startswith(b"FIELDS"))
        if fields.split()[1:] != [b"x", b"y", b"z"]:
            raise ValueError("只支持 x y z 二进制 PCD")
        points_line = next(line for line in header if line.upper().startswith(b"POINTS"))
        count = int(points_line.split()[1])
        data = stream.read(count * 12)
    if len(data) != count * 12:
        raise ValueError("PCD 点数据不完整")
    return np.frombuffer(data, dtype="<f4").reshape(count, 3).copy()


def write_xyz_pcd(points: np.ndarray, output: Path = OUTPUT_PCD) -> None:
    points = np.asarray(points, dtype="<f4").reshape(-1, 3)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    ).encode("ascii")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        stream.write(header)
        stream.write(np.ascontiguousarray(points).tobytes())


def write_complete_pcd(transform: np.ndarray, output: Path = OUTPUT_PCD) -> None:
    part1 = read_xyz_pcd(PART1_PCD)
    part2 = read_xyz_pcd(PART2_PCD)
    transformed_part1 = part1 @ transform[:3, :3].T + transform[:3, 3]
    write_xyz_pcd(np.concatenate((transformed_part1, part2)), output)


if __name__ == "__main__":
    np.set_printoptions(precision=8, suppress=True)
    transform = compute_transform()
    print(transform)
    write_complete_stl(transform)
    print(f"完整场地已写入：{OUTPUT}")
    write_complete_pcd(transform)
    print(f"完整场地点云已写入：{OUTPUT_PCD}")
