# p0
$$
R_B^W=
\begin{bmatrix}
0 & -1 & 0\\
1 & 0 & 0\\
0 & 0 & 1\\
\end{bmatrix}\quad
$$
$$
T_B^W=
\begin{bmatrix}
0 & -1 & 0 &1\\
1 & 0 & 0 & 2\\
0 & 0 & 1 &0.5\\
0 & 0 & 0 & 1\\
\end{bmatrix}\quad
$$
$$\boldsymbol p_W= \boldsymbol R_B^W \tilde{\boldsymbol p}_W$$
***answer is***
$$
\begin{bmatrix}
1\\
3\\
1.5\\
1\\
\end{bmatrix}\quad
$$
## 原因
如果先平移再旋转，旋转时会绕W的原点转动，得到完全不一样的坐标

# p1

## 推导

我写在下面代码的注释里了

```py
def compute_transform() -> np.ndarray:
    deg = np.pi / 180.0  # 1度对应的弧度
    t_w_b0 = np.array([1.0, 2.0, 0.5])  # 当前视角所在的世界坐标
    yaw, pitch, roll = 30.0 * deg, 10.0 * deg, 5.0 * deg  # 当前视角分别沿z、y、x的视角旋转

    # 用 yaw/pitch/roll 计算 B0 在 W 下的姿态。
    r_w_b0 = np.eye(3) @ rot_z(yaw) @ rot_y(pitch) @ rot_x(roll) 
    # 这里是将3x3的单位矩阵分别与三个旋转矩阵相乘，得到B0在W下的姿态。

    # 先绕 B0 自身 Z 轴旋转 +45 度。
    r_w_b2 = r_w_b0 @ rot_z(45.0 * deg)
    # 在当前的局部坐标系下绕自己的轴旋转需要把旋转矩阵乘在右边

    # 再沿 B1 自身 X 轴平移 2 米。
    t_w_b2 = t_w_b0 + r_w_b2 @ np.array([2.0, 0.0, 0.0])
    # r_w_b2 @ np.array([2.0, 0.0, 0.0]) 是把在r_w_b2下的x轴的两米变换成W下的两米，添加到t_w_b0上

    # 最后绕世界坐标系 W 的 Y 轴旋转 -30 度。
    r_w_b3 = rot_y(-30 * deg) @ r_w_b2
    t_w_b3 = rot_y(-30 * deg) @ t_w_b2
    # 这里旋转矩阵就在左边了，因为是沿着世界坐标系旋转的

    t_w_b3_matrix = np.eye(4)
    # 把旋转矩阵和位移矩阵拼成一个完整的齐次坐标系
    t_w_b3_matrix[:3, :3] = r_w_b3
    t_w_b3_matrix[:3, 3] = t_w_b3
    return t_w_b3_matrix
```

## 结果

$$
T_{B_3}^W=
\begin{bmatrix}
0.25632617 & -0.91100799 & -0.32305004 &1.12867775\\
0.96357512 & 0.26721088 & 0.01101461 & 3.92715025\\
0.07628809 & -0.31410632 & 0.94631778 &1.08558888\\
0 & 0 & 0 & 1\\
\end{bmatrix}\quad
$$
## 检查方式
在cloudcompare里rmuc2026.stl与rmuc2026-part2.stl正好拼成了一个场地
同时，矩阵最后一行是0，0，0，1