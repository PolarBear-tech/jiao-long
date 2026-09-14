from pathlib import Path
import open3d as o3d


# 两个文件的名字
SENTRY_NAME = "rmuc2026-sentry.pcd"
TARGET_NAME = "rmuc2026.pcd"

# 这个函数用于读取点云文件，并返回两个点云对象
def get_pcd_source(sentry_name, target_name):
    root = Path(__file__)
    assets = root.parent.parent / "assets"
    sentry = assets / sentry_name
    target = assets / target_name
    sentry_soure = o3d.io.read_point_cloud(str(sentry))
    target_source = o3d.io.read_point_cloud(str(target))
    return sentry_soure, target_source

# 对于两个源进行预处理
def process_pcd(*pcds, voxel_size=0.05, nb_neighbors=20, std_ratio=2.0, max_nn=30):
    res = []
    for pcd in pcds:
        # 体素下采样 -> 为了较少采样点，否则算法速度太慢
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        # 移除离群点 -> 为了去掉一些噪声点
        pcd_clean, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        # 法线估计   -> 为了后续的配准算法使用
        pcd_clean.estimate_normals(
            # 这里的voxel_size * 2是一个经验值，表示搜索半径为体素大小的两倍
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=max_nn)
        )
        res.append(pcd_clean)
    return res

# 得到FPFH的特征描述子，用于匹配两片点云对应的区域
# Fast Point Feature Histogram
def get_fpfh(*pcds, voxel_size=0.05, max_nn=100):
    res = []
    for pcd in pcds:
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd,
            # FPFH 要统计邻域内多对点法向量之间的夹角，构造局部特征直方图，
            # 它需要比法向量估计更大的邻域，才能提取有区分度的几何描述子
            # 所以这里的voxel_size会更大一些，是proccess_pcd函数中voxel_size的5倍
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=max_nn)
        )
        res.append(fpfh)
    return res


# RANSAC算法的初始配准，得到可能的结果
def get_canditate(sentry_down, target_down, sentry_fpfh, target_fpfh, mutual_filter=True, voxel_size=0.05, ransac_n=3):
    ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        sentry_down, target_down,
        sentry_fpfh, target_fpfh,
        # 这里mutual_filter参数表示是否使用互相验证的方式来过滤匹配点对，强烈建议给到True，可以提高准确率
        mutual_filter=mutual_filter,
        # 这里的max_correspondence_distance又是一个经验值，表示匹配点对之间的最大距离，超过这个距离的点对会被认为是错误匹配
        max_correspondence_distance=voxel_size * 3,
        # 由于是刚体变换，所以使用点到点的估计方法，False表示不使用权重，直接写死了
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        # 表示随机选取多少组对应点对，用来求解变换矩阵，三是因为刚体变换需要至少三个点对来确定
        ransac_n=ransac_n,
        # checkers里面包含了两个检查器，用来过滤匹配点对
        # TODO
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size*3)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
    )


if __name__ == "__main__":
    sentry_soure, target_source = get_pcd_source(SENTRY_NAME, TARGET_NAME)
    # print("sentry_soure:", sentry_soure)    # -> 7256164
    # print("target_source:", target_source)  # -> 1000000
    sentry_down, target_down = process_pcd(sentry_soure, target_source)
    # print("sentry_down:", sentry_down)    # -> 2402771
    # print("target_down:", target_down)  # -> 453660
