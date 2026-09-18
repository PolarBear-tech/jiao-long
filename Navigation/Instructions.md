# 交龙视觉招新答题说明

## 题目背景

你正在操控一个robomaster赛场上的小车，你的目标非常明确，让它正确抵达目标点。

## 答题规范

1. 请在[cpp/src/contestant.cpp](cpp/src/contestant.cpp)和[cpp/src/planner.cpp](cpp/src/planner.cpp)中完成代码，不需要，也不应该修改`python/`目录
2. 如果需要，允许对cpp文件的实现进行一定的修改，包括`nav_init`和`nav_run`函数的流程等，但请注意不要随意修改接口，尽量只修改实现，否则可能导致程序运行失败

## 提交规范

1. 完成每道题后，请在本仓库目录下分别创建 `ex1`、`ex2`、`ex3` 和 `ex4` 文件夹，并将本仓库的完整项目文件手动复制进去。
2. 每个 `ex<n>` 都必须是一个可以独立运行的完整项目副本，不能只复制自己修改过的 C++ 文件，并且要确认`ex<n>`是你刚完成`第n题`的状态。
3. 最终只提交一个压缩包，例如 `submission.zip`。压缩包结构如下：
   ```text
   submission.zip
   ├── README.md
   ├── ex1/
   ├── ex2/
   ├── ex3/
   └── ex4/
   ```
4. 每个 `ex<n>/` 文件夹内部都应包含完整项目文件，结构如下：
   ```text
   ex<n>/
       ├── .clangd
       ├── .editorconfig
       ├── .envrc
       ├── .gitattributes
       ├── .gitignore
       ├── Env.md
       ├── Instructions.md
       ├── run.bat
       ├── run.sh
       ├── requirements.txt
       ├── config/
       │   └── sim.yaml
       ├── cpp/
       │   ├── CMakeLists.txt
       │   ├── include/pyrobo/
       │   │   ├── c_api.h
       │   │   ├── interface.hpp
       │   │   └── planner.h
       │   └── src/
       │       ├── c_api.cpp
       │       ├── contestant.cpp
       │       └── planner.cpp
       ├── map/
       │   ├── map.png
       │   ├── unknown.png
       │   └── unknown_real.png
       └── python/
           ├── ex.py
           ├── main.py
           ├── unknown.py
           ├── planner/
           ├── scripts/
           └── sim/
   ```
5. 请复制项目中的隐藏文件，例如 `.clangd`、`.editorconfig`、`.gitattributes` 和 `.gitignore`。
6. 不需要复制 `.git/`、`cpp/build/`、`build/`、`.cache/`、`.mypy_cache/`、`__pycache__/`、`.idea/` 和 `.vscode/` 等 Git 信息、构建产物和缓存目录。
7. 压缩包根目录的 `README.md` 用于说明你的实现方法、每道题的完成情况以及运行方式，不需要在每个 `ex<n>/` 中重复创建 README。

## 项目基础知识说明

1. 本项目所使用地图全部以左下角原点，向右x轴，向上y轴为坐标系。
2. 单位使用: 长度单位(米)，分辨率(米/像素)，角度单位(弧度)，时间(秒)
3. 本项目仿真控制频率为`20Hz`，分辨率为`0.05m/px`
4. 本项目使用动力学模型是全向轮小车，行驶时不用在意车身朝向，只需输入地图系下的`x方向速度vx`和`y方向速度vy`即可控制小车运动。
5. 你应该在控制小车运动时注意动力学约束：具体数值（限速、加速度上下限）见 `python/ex.py` 中的 `control.dynamics`，**`config/sim.yaml` 里同名数值不会被读取**。
6. 本项目所有地图存储在`map`目录下，黑色为障碍物，无法通行；白色为自由通行区域；灰色为未知区域。
7. 本项目默认起始点在左下角，终点在右上角，当小车中心点距离终点在`0.25m`以内时，会认为自己到达了终点
8. 你可以在[配置文件](config/sim.yaml)里面修改仿真可视化相关的参数（这是该文件中**唯一**会被读取的部分，其余数值都是硬编码的，改了不起作用）
9.  你可以参考[环境配置说明](Env.md)来配置本仓库需要的环境

## 地图查询和修改接口

所有坐标都是地图坐标，长度单位是米。

### 查询地图

```cpp
const pyrobo::Map& map = sim.map();
```

最常用的查询只有一个：

```cpp
if (map.is_free(x, y)) {
    // (x, y) 是地图坐标，可以通行
}
```

`is_free` 返回 `false` 表示这里是障碍物或已经越界。

需要遍历地图时使用：

```cpp
const auto shape = map.shape();
for (int row = 0; row < shape.height; ++row) {
    for (int column = 0; column < shape.width; ++column) {
        if (map.is_free_cell(column, row)) {
            // (column, row) 是栅格坐标
            const auto point = map.grid_to_world(column, row);
            // point.x 和 point.y 是这个格子中心的地图坐标
        }
    }
}
```

`map.world_to_grid(x, y)` 的输入 `(x, y)` 是地图坐标，返回值是栅格坐标。
`map.grid_to_world(column, row)` 的输入是栅格坐标，返回值是格子中心的地图坐标。

### 显示路径

规划出路径后，直接交给仿真器显示：

```cpp
pyrobo::Path path{
    // 每个点都是地图坐标
    {robot_pose.x, robot_pose.y},
    {goal_pose.x, goal_pose.y},
};
sim.set_display_path(path);
```

路径中的每个点都是地图坐标。这个接口只负责显示，不会自动控制小车。

### 设置规划地图

如果你处理了地图，可以在初始化时设置一份规划地图：

```cpp
sim.set_planning_map(map);
```

它只影响显示，不影响真实碰撞检测和雷达数据。碰撞检测和雷达始终使用 `sim.map()`。

### 机器人状态和控制

```cpp
const pyrobo::Pose pose = sim.get_pose();
// pose.x 和 pose.y 是地图坐标，pose.yaw 是弧度

const auto goal = sim.get_goal();
// goal->x 和 goal->y 是地图坐标，goal->yaw 是弧度

const auto feedback = sim.get_feedback();
// feedback.first 和 feedback.second 是地图坐标系下实际执行的 vx、vy

sim.set_control(vx, vy);
// vx、vy 是地图坐标系下的速度，单位是 m/s
```

## 规划器接口

你只需要修改 `cpp/src/planner.cpp` 中的两个函数，仿真器会自动调用它们。

### 路径规划

`PathPlanner::plan` 返回一组地图坐标点。保留函数名和参数，只把函数体写成：

```cpp
return {
    {robot_pose.x, robot_pose.y},
    {goal_pose.x, goal_pose.y},
};
```

每个点按 `{x, y}` 写，路径至少包含起点和终点。
`robot_pose` 和 `goal_pose` 中的 `x、y` 是地图坐标，`map` 中的 `column、row` 是栅格坐标。

### 速度控制

`ControlPlanner::control_plan` 返回 `{vx, vy}`，分别是地图坐标系下的 x、y 方向速度。函数体可以写成：

```cpp
const float vx = static_cast<float>(goal_pose.x - robot_pose.x);
const float vy = static_cast<float>(goal_pose.y - robot_pose.y);
return {vx, vy};
```

不需要计算车身坐标系，也不需要设置角速度。实际速度会受到 `python/ex.py` 中 `control.dynamics` 的限速与加速度约束。
`robot_pose` 和 `goal_pose` 中的 `x、y` 是地图坐标，`feedback` 是地图坐标系下的实际 `vx、vy`。

## 获取雷达数据接口

直接调用 `sim.get_lidar()`：

```cpp
const std::vector<double> lidar = sim.get_lidar();
```

雷达调用不需要坐标输入。默认返回 360 个距离值，单位是米，最大距离是 3 米。第 `i` 个数据对应的角度为：

```text
-pi + i * 2 * pi / 360
```

这个角度是相对于小车当前朝向的角度，`i = 0` 是车身后方，`i = 180` 是车身前方。

需要指定数量或范围时：

```cpp
const auto lidar = sim.get_lidar("robot", 360, 3.0);
```

雷达读取的是实际地图，不读取规划地图。

## 题目内容

---

### 第一题

请你在已知先验地图和目标点的情况下，为它规划出一条可行路线。(20 points)

你可以通过运行

```text
Windows:
run.bat

Linux/macOS:
./run.sh
```

来查看自己的结果

你可以搜索文本 `Question 1` 来找到推荐的答题位置

---

### 第二题

上一题没有考虑小车的半径，接下来请尝试自己的算法能否在机器人有体积 (半径) 的情况下依然能运行，如果不行，请重新设计你的方法。(20 points)

你可以通过运行

```text
Windows:
run.bat

Linux/macOS:
./run.sh
```

并打开[配置文件](config/sim.yaml)中的`pyrobo.display.show_planning_map`来查看自己的结果

你可以搜索文本 `Question 2` 来找到推荐的答题位置

---

### 第三题

请自己规划一个方法，让小车真正开起来，开到终点，希望你能在1分钟内完成这个任务。(30 points)

你可以通过运行

```text
Windows:
run.bat

Linux/macOS:
./run.sh
```

来查看自己的结果，小车到达终点后会提示到达终点。

你可以搜索文本 `Question 3` 来找到推荐的答题位置

---

### 第四题

新的地图出现了，但是小明很粗心，不小心在它上面滴了一点墨水，好在墨水不是黑色的，它在地图上不黑不白的，你可以靠自己的雷达探测它的缺口在哪里！

灰色区域是未知区域，它存在一处缺口，你需要自己发现它的位置并规划出一条正确的路径，小车上装有二维雷达，你可以使用360度角内的距离数据，我可以保证你的数据全都有效且正确。(30 points)

PS: 
1. `灰色区域`会被识别成`障碍物`，但是雷达可以正确识别
2. 你看到的`unknown.png`是你的先验地图，但是`unknown_real.png`才是真正的地图样貌
3. 提醒出题人会变换缺口位置来评判你的代码水平，请不要做硬编码

你可以通过运行

```text
Windows:
run.bat --scenario unknown

Linux/macOS:
./run.sh --scenario unknown
```

来查看自己的结果

你可以搜索文本 `Question 4` 来找到推荐的答题位置

---

## 提醒

1. 你的代码提交后需确保编译正常
2. 出题人会使用不同的地图配置来检测你的算法，以评判分数
