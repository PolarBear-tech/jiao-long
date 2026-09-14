# TODO: Finish P2
import open3d as o3d
import numpy as np

def main():
    # ========= 1.读取点云 =========
    source = o3d.io.read_point_cloud("../assets/rmuc2026-sentry.pcd")   # S_S扫描坐标系
    target = o3d.io.read_point_cloud("../assets/rmuc2026.pcd")         # W_W世界参考坐标系

    # ========= 2.预处理：体素下采样 =========
    voxel_size = 0.05
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)

    # ========= 3.【重点】题目给的初值，消除180°歧义 =========
    # pS = R^T(-t) ≈ [-4.71, -6.81, -0.05]，R初始为单位阵
    t_init = np.array([4.71, 6.81, 0.05])
    R_init = np.eye(3)
    # 构造4*4初始齐次变换矩阵
    T_init = np.eye(4)
    T_init[:3, :3] = R_init
    T_init[:3, 3] = t_init
    print("=== 初始猜测变换矩阵 T_init ===")
    print(T_init)

    # ========= 4.ICP精配准 =========
    max_correspondence_distance = 0.2
    reg_result = o3d.pipelines.registration.registration_icp(
        source_down,
        target_down,
        max_correspondence_distance,
        T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
    )

    T_SW = reg_result.transformation # T_S^W，S坐标系 → W世界坐标系
    print("\n==== 最终求得 T_S^W 变换矩阵 ====")
    print(T_SW)

    # ========= 5.对点云施加变换并保存 =========
    source_aligned = source.transform(T_SW)
    o3d.io.write_point_cloud("../assets/rmuc2026-sentry-aligned.pcd", source_aligned)
    print("\n✅ 对齐后的点云保存至 assets/rmuc2026-sentry-aligned.pcd")

    # ========= 6.校验旋转矩阵 =========
    R = T_SW[:3, :3]
    print("\n==== 旋转矩阵校验 ====")
    print("R^T @ R = \n", R.T @ R)
    print("det(R) = ", np.linalg.det(R))

    # ========= 7.反推：世界原点在扫描S坐标系下坐标，核对题目给的值 =========
    # pS = R.T @ (-t)
    t = T_SW[:3,3]
    pS = R.T @ (-t)
    print("\n==== 反推：世界原点在扫描坐标系S中的坐标 ====")
    print(pS)

if __name__ == "__main__":
    main()
