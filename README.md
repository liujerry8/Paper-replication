# 近似流形概率路线图的机器人受约束轨迹规划

**Approximate Manifold Probabilistic Roadmap (AM-PRM) for Constrained Robot Trajectory Planning**

This repository contains a ROS implementation of the constrained trajectory planning method described in Chapter 2 of the paper:

> *基于近似流形概率路线图的机器人受约束轨迹规划*  
> (Robot Constrained Trajectory Planning Based on Approximate Manifold Probabilistic Roadmap)

---

## 论文解读 / Paper Summary

The paper addresses trajectory planning for a robot arm (UR5e) in an **automated chemistry laboratory**, where the arm must manipulate reagent bottles and test tube racks while maintaining specific end-effector pose constraints (keeping the held vessel upright).

### 核心问题 / Problem

The robot must plan collision-free trajectories in a cluttered environment while simultaneously satisfying:

- **Equality or inequality constraints** on the end-effector orientation:  
  `r_x² ≤ 0.01, r_y² ≤ 0.01`  
  (end-effector may not tilt beyond a threshold, or the reagent will spill)
- **Short planning time** (target < 300 ms online)
- **Consistent trajectory quality** across repeated plans

Existing methods (RRT*, PRM*, BIT*) struggle to satisfy both time and quality requirements simultaneously in complex constrained scenes.

### 方法概述 / Method Overview

The proposed method has **three phases**:

```
        Offline                           Online
  ┌─────────────────────┐         ┌───────────────────────┐
  │ 1. Build Approx.    │         │ 3. LazyPRM Query      │
  │    Manifold Graph G │────────▶│    (< 300 ms)         │
  │    (V, E)           │         │                       │
  └─────────┬───────────┘         │  - Load roadmap M     │
            │                     │  - Add start/goal     │
            ▼                     │  - Lazy collision     │
  ┌─────────────────────┐         │    checking           │
  │ 2. Build PRM*       │         │  - IPTP for vel/acc   │
  │    Roadmap M        │────────▶│    profile            │
  │    (offline, once)  │         └───────────────────────┘
  └─────────────────────┘
```

#### Phase 1: Approximate Manifold Graph (近似图) [Section 2.1]

The constraint manifold `X = {x ∈ A | F(x) = 0}` is approximated by an undirected graph `G = (V, E)`.

**Vertex set construction (Algorithm 2.1):**
1. Sample a random configuration `x₀` in the full joint space.
2. Project `x₀` onto the manifold using Newton-Raphson (eq. 2.5):  
   `x_{i+1} = x_i - J(x_i)⁺ F(x_i)`
3. Validate diversity against existing vertices using three criteria (eqs. 2.6–2.8):
   - Distance to tangent plane > ε (eq. 2.6)
   - Tangent spaces not too aligned: ‖Φᵢᵀ Φⱼ‖ < cos(α) (eq. 2.7)
   - Tangent-coordinate distance > ρ (eq. 2.8)
4. Repeat until `n_c` vertices are collected.

**Edge set construction (Algorithm 2.2):**
1. For each vertex, find nearest neighbours.
2. Connect by straight line if all interpolated points satisfy `‖F(x_l)‖ ≤ ζ` (eq. 2.11).

#### Phase 2: PRM* Offline Roadmap (路线图) [Section 2.2.4]

PRM* [Karaman & Frazzoli, 2011] is applied **on the approximate graph** to build a roadmap containing near-optimal collision-free trajectories.

- k nearest neighbours: `k = ⌈2e · log(n)⌉`  
- Terminates when the top-5 path costs converge (< 10% change) or time limit is reached.
- Roadmap is **reused across queries** without recomputation.

#### Phase 3: LazyPRM Online Planning [Section 2.2.5]

- Loads the offline roadmap.
- Connects start/goal to the roadmap.
- Runs graph search (Dijkstra), then **lazily** validates only the edges on the returned path.
- Invalid edges are removed and search is re-run.
- Post-processes the joint path using MoveIt!'s **IPTP** (Iterative Parabolic Time Parameterization).

### 实验结果 / Experimental Results

| Setup | Method | Success Rate | Planning Time |
|-------|--------|-------------|---------------|
| With approx. graph | **Ours (AM-PRM)** | **100%** | **67 ms** |
| With approx. graph | LazyPRM | 85% | 286 ms |
| With approx. graph | BiTRRT | 85% | 128 ms |
| With approx. graph | PRM* | 100% | 5525 ms |

The proposed method achieves 100% success with the shortest planning time while maintaining trajectory quality and consistency.

---

## 复现步骤 / Replication Steps

### 1. 系统要求 / Prerequisites

- Ubuntu 20.04
- ROS Noetic (`ros-noetic-desktop-full`)
- MoveIt! (`ros-noetic-moveit`)
- UR Robot Driver (`ros-noetic-ur-robot-driver`)
- UR5e MoveIt! config (`ur5e_moveit_config`) — from the [`universal_robot`](https://github.com/ros-industrial/universal_robot) meta-package
- Python 3.8+, with packages: `numpy`, `scipy`, `networkx`, `scikit-learn`, `pyyaml`

**方式 A — 通过 apt 安装 / Install via apt (recommended):**

```bash
sudo apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-ur-robot-driver \
    ros-noetic-ur5e-moveit-config \
    ros-noetic-tf2-geometry-msgs

pip3 install numpy scipy networkx scikit-learn pyyaml
```

> **注意 / Note:** `ros-noetic-ur5e-moveit-config` is provided by the
> [`universal_robot`](https://github.com/ros-industrial/universal_robot)
> meta-package.  If the apt package is not available in your sources, use
> 方式 B below.

**方式 B — 从源码安装 / Install from source:**

If `ros-noetic-ur5e-moveit-config` is not found via apt, clone the
`universal_robot` repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

This provides both `ur_gazebo` and `ur5e_moveit_config` (among other
packages).  You can verify the package is available with:

```bash
rospack find ur5e_moveit_config
```

---

## 场景文件 / Scene Files

### 场景文件格式 / Scene File Format

Obstacles are described in a YAML file.  Three shape types are supported:
`box`, `sphere`, and `cylinder`.

```yaml
name: "my_lab_scene"
description: "Optional description"
obstacles:
  - name: table
    type: box
    size: [1.2, 0.8, 0.05]        # [x, y, z] extents in metres
    pose:
      xyz: [0.55, 0.0, 0.36]      # position in world frame [m]
      rpy: [0.0,  0.0, 0.0]       # roll/pitch/yaw [rad]

  - name: column
    type: cylinder
    radius: 0.05                  # metres
    height: 0.30                  # metres
    pose:
      xyz: [0.60, 0.10, 0.55]
      rpy: [0.0, 0.0, 0.0]

  - name: ball_valve
    type: sphere
    radius: 0.03                  # metres
    pose:
      xyz: [0.30, -0.10, 0.60]
      rpy: [0.0, 0.0, 0.0]
```

Two ready-made scene files (from the paper's experiments) are included in `scenes/`:

| File | Description |
|------|-------------|
| `scenes/sim_scene_1.yaml` | 仿真场景1: powder dispenser, liquid dispenser, UV spectrometer |
| `scenes/sim_scene_2.yaml` | 仿真场景2: liquid dispenser + liquid extractor |

### 快速使用（无需 ROS）/ Quick Start — No ROS Required

`run_planner.py` runs the full pipeline (build graph → build roadmap → LazyPRM query) using geometric collision checking against a scene file. **No ROS or MoveIt! installation needed.**

```bash
cd approx_manifold_prm

# Plan a path using sim_scene_1
python3 scripts/run_planner.py \
    --scene  scenes/sim_scene_1.yaml \
    --start  "0,0,0,0,0,0" \
    --goal   "0.3,-0.2,0.1,-0.1,0.1,0" \
    --n_c    500 \
    --n_e    3 \
    --output /tmp/path.json
```

**Reuse a pre-built graph and roadmap** (much faster for repeated queries):

```bash
# First run: build and save
python3 scripts/run_planner.py \
    --scene         scenes/sim_scene_1.yaml \
    --start         "0,0,0,0,0,0" \
    --goal          "0.3,-0.2,0.1,-0.1,0.1,0" \
    --save_graph    /tmp/approx_graph.pkl \
    --save_roadmap  /tmp/roadmap.pkl

# Subsequent runs: load pre-built files
python3 scripts/run_planner.py \
    --scene    scenes/sim_scene_1.yaml \
    --graph    /tmp/approx_graph.pkl \
    --roadmap  /tmp/roadmap.pkl \
    --start    "0,0,0,0,0,0" \
    --goal     "0.3,-0.2,0.1,-0.1,0.1,0"
```

**Available arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--scene` | *(required)* | Path to YAML scene file |
| `--start` | *(required)* | Start joints (6 comma-separated radians) |
| `--goal` | *(required)* | Goal joints (6 comma-separated radians) |
| `--graph` | *(auto-build)* | Pre-built approximate graph `.pkl` |
| `--roadmap` | *(auto-build)* | Pre-built PRM* roadmap `.pkl` |
| `--save_graph` | — | Save newly-built graph to this path |
| `--save_roadmap` | — | Save newly-built roadmap to this path |
| `--n_c` | 500 | Graph vertex count when building |
| `--n_e` | 3 | Max edges per vertex when building |
| `--graph_time` | 30 s | PRM* roadmap build time budget |
| `--plan_time` | 10 s | LazyPRM online query time limit |
| `--output` | — | Save planned path as JSON |

**Expected output:**
```
Scene 'sim_scene_1': 10 obstacle(s) loaded
  • workbench (box)
  • powder_dispenser_body (box)
  ...
Start and goal configurations are collision-free ✓
Building approximate manifold graph (n_c=500, n_e=3) …
  Done in 35.2s: 500 vertices, 742 edges
Building PRM* roadmap (max_time=30.0s) …
  Done in 30.0s: 287 nodes, 1453 edges

[SUCCESS] Path found in 45.3 ms, 7 waypoints:
   WP        j1        j2        j3        j4        j5        j6
  ──────────────────────────────────────────────────────────────────
    0    0.0000    0.0000    0.0000    0.0000    0.0000    0.0000
    1    0.0523   -0.0314    0.0178   -0.0214    0.0312    0.0000
    ...
```

---

### ROS 环境下完整实验复现流程 / Full ROS Experiment Replication Guide

> **对照论文第 2 章 / Paper Reference: Chapter 2**
>
> 本节按照论文的三阶段方法（Phase 1 → Phase 2 → Phase 3）逐步说明如何在
> ROS + MoveIt! 环境下复现论文中的全部实验。每一步都标注了对应的论文章节号和
> 关键公式，方便与原文对照。
>
> This section provides a step-by-step ROS + MoveIt! guide for replicating
> the paper's experiments, following the three-phase method. Each step is
> annotated with the corresponding paper section and key equations.

```
  总览 / Overview  (论文 Section 2.2, Fig. 2.1)
  ───────────────────────────────────────────────────────────────────
            Offline (执行一次)                    Online (每次查询)
  ┌──────────────────────────────┐      ┌─────────────────────────────┐
  │ Phase 1: 构建近似流形图 G     │      │ Phase 3: LazyPRM 在线规划    │
  │   (Algorithm 2.1 + 2.2)     │      │   (Section 2.2.5)           │
  │   G = (V, E)                │      │   • 加载路线图 M              │
  └──────────┬───────────────────┘      │   • 连接起/终点到 M          │
             │                          │   • Dijkstra 最短路径        │
             ▼                          │   • 惰性碰撞检测             │
  ┌──────────────────────────────┐      │   • IPTP 速度/加速度参数化   │
  │ Phase 2: 构建 PRM* 路线图 M  │─────▶│                             │
  │   (Section 2.2.4)           │      │ 目标: 规划时间 < 300 ms      │
  │   M = PRM*(G, scene)        │      │   (论文 Table 2.2: 67 ms)   │
  └──────────────────────────────┘      └─────────────────────────────┘
  ───────────────────────────────────────────────────────────────────
```

#### 步骤 0：环境准备 / Step 0 — Prerequisites & Workspace Setup

> **对照 / Paper Ref:** Section 2.3.1 — 实验平台：UR5e + 深度相机 + MoveIt!

**0a. 安装依赖 / Install System Dependencies**

```bash
# ROS Noetic (Ubuntu 20.04)
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-ur-robot-driver \
    ros-noetic-ur5e-moveit-config \
    ros-noetic-tf2-geometry-msgs

pip3 install numpy scipy networkx scikit-learn pyyaml
```

> 如果 `ros-noetic-ur5e-moveit-config` 无法通过 apt 安装，请在步骤 0b 中
> 通过 `git clone` 从源码安装 `universal_robot` 元包。

**0b. 创建 Catkin 工作空间 / Create Catkin Workspace**

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 将本仓库复制到 src 目录 / Copy this package into src/
cp -r /path/to/approx_manifold_prm .

# (可选) 如果 apt 中没有 ur5e_moveit_config，从源码安装 universal_robot
# git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git

cd ~/catkin_ws

# 安装所有声明的依赖 / Install all declared dependencies
rosdep install --from-paths src --ignore-src -r -y

catkin_make
source devel/setup.bash
```

验证安装是否成功 / Verify the install:

```bash
# 应输出包路径，例如 /home/<user>/catkin_ws/src/approx_manifold_prm
rospack find approx_manifold_prm

# 验证 MoveIt! 配置包存在
rospack find ur5e_moveit_config
```

---

#### 步骤 1：启动机器人与 MoveIt! / Step 1 — Launch Robot & MoveIt!

> **对照 / Paper Ref:** Section 2.3.1 — 仿真平台使用 UR5e Gazebo + MoveIt!；
> 实物实验使用 UR5e + `ur_robot_driver`。

每个终端 (Terminal) 的操作分别在独立的 shell 中运行。
请确保每个终端都先执行 `source ~/catkin_ws/devel/setup.bash`。

**方式 A：仿真 (Gazebo) / Simulation**

```bash
# ── Terminal 1: 启动 Gazebo 仿真 ────────────────────────────────────
# 此命令会在 Gazebo 中生成 UR5e 机械臂和控制器。
# 对应论文 Section 2.3.1 中描述的仿真平台。
source ~/catkin_ws/devel/setup.bash
roslaunch ur_gazebo ur5e_bringup.launch
```

```bash
# ── Terminal 2: 启动 MoveIt! 运动规划框架 ───────────────────────────
# MoveIt! 提供碰撞检测 (FCL)、正运动学 (FK/IK)、运动规划接口。
# sim:=true 表示使用仿真控制器，而非真实机器人驱动。
# 启动后会加载 SRDF 运动学模型和 OMPL 规划器插件。
source ~/catkin_ws/devel/setup.bash
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
```

> **提示:** 等待 Gazebo 窗口中出现 UR5e 模型，以及 Terminal 2 出现
> `You can start planning now!` 后再继续后续步骤。

**方式 B：实物机器人 / Real Robot**

```bash
# ── Terminal 1: 连接实物 UR5e ────────────────────────────────────────
# 将 <ROBOT_IP> 替换为 UR5e 控制器的 IP 地址（如 192.168.1.100）。
# 此驱动通过 RTDE 协议与控制器通信，频率为 500 Hz。
source ~/catkin_ws/devel/setup.bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<ROBOT_IP>
```

```bash
# ── Terminal 2: 启动 MoveIt! (不带 sim 标志) ────────────────────────
source ~/catkin_ws/devel/setup.bash
roslaunch ur5e_moveit_config moveit_planning_execution.launch
```

**验证连接 / Verify:**

```bash
# 检查关节话题是否有数据 (应以 500 Hz 发布)
rostopic hz /joint_states

# 检查 MoveIt! 规划服务是否就绪
rosservice list | grep move_group
# 应输出 /move_group/plan, /move_group/execute 等
```

---

#### 步骤 2 (Phase 1)：离线构建近似流形图 G / Step 2 — Offline: Build Approximate Manifold Graph

> **对照 / Paper Ref:** Section 2.1 (Algorithm 2.1 + Algorithm 2.2)
>
> 本步骤在约束流形 `X = {x ∈ A | F(x) = 0}` 上采样 n_c 个顶点，通过
> Newton-Raphson 投影 (eq. 2.5) 和多样性检查 (eqs. 2.6–2.8) 构建近似图
> G = (V, E)。这一步**只需执行一次**，不依赖于具体的场景环境。
>
> **约束函数 (Section 2.3):** 末端执行器保持竖直约束
> `r_x² ≤ 0.01, r_y² ≤ 0.01`（持瓶时防止试剂溢出）。

```bash
# ── Terminal 3: 构建近似图 ──────────────────────────────────────────
source ~/catkin_ws/devel/setup.bash

# 快速测试: n_c=1000 (约 30 秒)
roslaunch approx_manifold_prm build_approx_graph.launch \
    n_c:=1000 n_e:=5 output:=/tmp/approx_graph.pkl

# 论文级别实验: n_c=10000 (约 6 分钟)
# roslaunch approx_manifold_prm build_approx_graph.launch \
#     n_c:=10000 n_e:=5 output:=/tmp/approx_graph.pkl
```

**参数说明 / Parameter Details (对照 Algorithm 2.1):**

| 参数 | 默认值 | 论文值 | 说明 / Paper Reference |
|------|--------|--------|------------------------|
| `n_c` | 1000 | 10³–10⁵ | 图中顶点数量。更多顶点 → 更好覆盖流形 (Section 2.1) |
| `n_e` | 5 | 0–30 | 每个顶点的最大边数。控制图的连通性 (Algorithm 2.2) |
| `delta` | 0.01 | 0.01 | 约束阈值 `r_x², r_y² ≤ δ` (Section 2.3, eq. 2.1) |
| `eps` | 0.05 | 0.05 | 多样性参数 ε：到切平面最小距离 (eq. 2.6) |
| `rho` | 0.05 | 0.05 | 多样性参数 ρ：切坐标最小距离 (eq. 2.8) |
| `alpha` | 30.0 | 30° | 多样性参数 α：切空间最大对齐角度 (eq. 2.7) |

**关键算法步骤 (Algorithm 2.1 — 顶点集构建):**
1. 在关节空间随机采样 `x₀`
2. 使用 Newton-Raphson 将 `x₀` 投影到约束流形：`x_{i+1} = x_i − J(x_i)⁺ F(x_i)` (eq. 2.5)
3. 用三个多样性准则验证候选点：
   - 到切平面距离 > ε (eq. 2.6)
   - 切空间不过度对齐：`‖Φᵢᵀ Φⱼ‖ < cos(α)` (eq. 2.7)
   - 切坐标距离 > ρ (eq. 2.8)
4. 重复直至收集 n_c 个有效顶点

**关键算法步骤 (Algorithm 2.2 — 边集构建):**
1. 对每个顶点查找 k 近邻
2. 沿直线插值，验证所有中间点满足 `‖F(x_l)‖ ≤ ζ` (eq. 2.11)
3. 满足条件则添加边

**预期构建时间 / Expected Build Time (论文 Table 2.1):**

| n_c / n_e | 构建时间 (s) | 内存 (MB) |
|-----------|-------------|-----------|
| 1,000 / 0 | 30.76 | 0.07 |
| 1,000 / 1 | 30.96 | 0.15 |
| 10,000 / 5 | 348.51 | 7.31 |
| 100,000 / 30 | 5750.66 | 454.36 |

**预期输出 / Expected Output:**

```
Building approximate manifold graph: n_c=1000, n_e=5
Graph built: 1000 vertices, 2347 edges
Graph saved to /tmp/approx_graph.pkl
```

> **重要:** 近似图 G 仅依赖于约束类型 (末端执行器竖直)，**不依赖于特定场景**。
> 构建一次后，可在不同场景 (sim_scene_1, sim_scene_2) 中复用。

---

#### 步骤 3 (Phase 2)：离线构建 PRM* 路线图 M / Step 3 — Offline: Build PRM* Roadmap

> **对照 / Paper Ref:** Section 2.2.4 — PRM* 路线图构建
>
> 在近似图 G 上运行 PRM* 算法 [Karaman & Frazzoli, 2011]，结合场景碰撞检测，
> 构建包含近最优无碰撞轨迹的路线图 M。
>
> - k 近邻数: `k = ⌈2e · log(n)⌉` (渐近最优性保证)
> - 收敛准则: 前 5 条最优路径的代价变化 < 10%，或达到时间上限 (Section 2.3)
> - 路线图 M 可**跨查询复用**，无需重新构建。

论文中使用两种仿真场景进行实验。分别对每个场景构建路线图：

**场景 1（sim_scene_1）/ Scene 1:**

> 论文 Section 2.3.1 "仿真场景 1"：粉末进样器 + 液体进样器 + 紫外光谱检测器。
> 机械臂需从试管架取试剂瓶放至液体进样器，全程保持瓶体竖直。

```bash
# ── Terminal 3 (续): 为场景1构建路线图 ──────────────────────────────
# Option A — 使用 YAML 场景文件的几何碰撞检测 (无需额外 MoveIt! 场景加载)
roslaunch approx_manifold_prm build_roadmap.launch \
    graph:=/tmp/approx_graph.pkl \
    output:=/tmp/roadmap_scene1.pkl \
    max_time:=300 \
    scene_file:=$(rospack find approx_manifold_prm)/scenes/sim_scene_1.yaml
```

```bash
# Option B — 使用 MoveIt! Planning Scene (碰撞检测由 MoveIt!/FCL 执行)
# 适用于实际实验中使用 NeRF-SLAM 重建场景后加载到 MoveIt! 的情况。
# 需要 Terminal 1+2 中 MoveIt! 已启动并加载了碰撞场景。
roslaunch approx_manifold_prm build_roadmap.launch \
    graph:=/tmp/approx_graph.pkl \
    output:=/tmp/roadmap_scene1.pkl \
    max_time:=300
```

**场景 2（sim_scene_2）/ Scene 2:**

> 论文 Section 2.3.1 "仿真场景 2"：液体进样器 + 液体抽取器。
> 机械臂需将试管架从平台搬运到液体抽取器平台，保持试管架竖直。

```bash
roslaunch approx_manifold_prm build_roadmap.launch \
    graph:=/tmp/approx_graph.pkl \
    output:=/tmp/roadmap_scene2.pkl \
    max_time:=300 \
    scene_file:=$(rospack find approx_manifold_prm)/scenes/sim_scene_2.yaml
```

**路线图构建参数 / Roadmap Parameters:**

| 参数 | 默认值 | 论文值 | 说明 / Paper Reference |
|------|--------|--------|------------------------|
| `max_time` | 300 s | 300 s | PRM* 构建时间上限 (Section 2.2.4) |
| `convergence_threshold` | 0.10 | 0.10 | 前 5 最优路径代价变化率阈值 (Section 2.3) |

**预期输出 / Expected Output:**

```
Loaded graph: 1000 vertices
[scene] Loaded 10 obstacle(s) from '.../sim_scene_1.yaml'
Building PRM* roadmap (max_time=300.0s)...
Roadmap built: 287 nodes, 1453 edges
Roadmap saved to /tmp/roadmap_scene1.pkl
```

---

#### 步骤 4 (Phase 3)：在线 LazyPRM 规划 / Step 4 — Online: LazyPRM Query

> **对照 / Paper Ref:** Section 2.2.5 — LazyPRM 在线规划
>
> 加载离线路线图 M，将起点/终点连接到路线图，使用 Dijkstra 搜索最短路径，
> 然后**惰性地**仅验证返回路径上的边。无效边被移除后重新搜索。
> 最后用 MoveIt! 的 IPTP (Iterative Parabolic Time Parameterization)
> 计算速度/加速度参数，生成可直接执行的关节轨迹。
>
> **目标:** 论文 Table 2.2 中报告的在线规划时间为 **67 ms**（100% 成功率）。

**4a. 启动在线规划节点 / Launch Planner Node:**

```bash
# ── Terminal 3: 启动在线规划服务 ────────────────────────────────────
source ~/catkin_ws/devel/setup.bash

# 使用场景1的路线图
roslaunch approx_manifold_prm trajectory_planner.launch \
    graph_file:=/tmp/approx_graph.pkl \
    roadmap_file:=/tmp/roadmap_scene1.pkl \
    scene_file:=$(rospack find approx_manifold_prm)/scenes/sim_scene_1.yaml
```

> 此命令会：
> 1. 将 YAML 场景中的障碍物加载到 MoveIt! Planning Scene (通过 `scene_loader.py` 的 `load_into_moveit`)
> 2. 加载预构建的近似图 G 和路线图 M
> 3. 初始化 LazyPRM 规划器并注册 ROS 服务 `~/plan_trajectory`
> 4. 等待来自 ROS 服务的规划请求

**4b. 发送规划请求 / Send Planning Request:**

```bash
# ── Terminal 4: 发送规划请求 ────────────────────────────────────────
source ~/catkin_ws/devel/setup.bash

# 示例: 从零位到场景1中液体进样器附近的位姿
# start_joints: UR5e 零位 (所有关节角度为 0)
# goal_joints: 液体进样器附近的关节构型
rosservice call /trajectory_planner/plan_trajectory \
    "start_joints: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
     goal_joints:  [1.0, -1.0, 0.5, -1.0, 0.5, 0.0]
     time_limit:   10.0"
```

**预期响应 / Expected Response:**

```yaml
success: True
planning_time_ms: 45.3        # 论文 Table 2.2 报告的平均值: 67 ms
trajectory:
  joint_trajectory:
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                  wrist_1_joint, wrist_2_joint, wrist_3_joint]
    points:
      - positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        time_from_start: {secs: 0, nsecs: 0}
      - positions: [0.052, -1.54, 0.017, -1.55, 0.031, 0.0]
        time_from_start: {secs: 0, nsecs: 200000000}
      ...                         # IPTP 时间参数化后的轨迹点
message: "Path found (7 waypoints, 45.3 ms)"
```

**4c. 多次查询与统计 / Multiple Queries (对照论文 Section 2.3.2):**

论文中对每种方法进行多次独立试验来计算统计量。可连续发送多次请求来验证规划
时间的一致性：

```bash
# 连续发送 20 次规划请求以统计平均规划时间
for i in $(seq 1 20); do
    echo "=== Trial $i ==="
    rosservice call /trajectory_planner/plan_trajectory \
        "start_joints: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
         goal_joints:  [1.0, -1.0, 0.5, -1.0, 0.5, 0.0]
         time_limit:   10.0" 2>&1 | grep planning_time_ms
done
```

---

#### 步骤 5：基线方法对比 / Step 5 — Baseline Comparisons via MoveIt!

> **对照 / Paper Ref:** Section 2.3.2, Table 2.2 — 与 LazyPRM / BiTRRT / PRM*
> 等基线方法的对比
>
> 论文在相同的仿真场景下，分别使用以下 MoveIt!/OMPL 规划器进行对比实验。
> 测量指标：成功率、规划时间、路径质量。

**5a. 通过 MoveIt! Commander 运行基线规划器:**

```bash
# ── Terminal 4: 使用 MoveIt! 内置规划器 ─────────────────────────────
source ~/catkin_ws/devel/setup.bash
rosrun moveit_commander moveit_commander_cmdline.py
```

在 MoveIt! Commander 交互式终端中：

```
# 选择规划组 (UR5e 的运动学链)
> use manipulator

# === 基线 1: LazyPRM (论文 Table 2.2: 85% 成功率, 286 ms) ===
> planner LazyPRM
> go [1.0, -1.0, 0.5, -1.0, 0.5, 0.0]

# === 基线 2: BiTRRT (论文 Table 2.2: 85% 成功率, 128 ms) ===
> planner BiTRRT
> go [1.0, -1.0, 0.5, -1.0, 0.5, 0.0]

# === 基线 3: PRM* (论文 Table 2.2: 100% 成功率, 5525 ms) ===
> planner PRMstar
> go [1.0, -1.0, 0.5, -1.0, 0.5, 0.0]

# === 其他基线 (论文 Section 2.3.2) ===
> planner RRTConnect
> go [1.0, -1.0, 0.5, -1.0, 0.5, 0.0]

> planner RRTstar
> go [1.0, -1.0, 0.5, -1.0, 0.5, 0.0]

> planner BITstar
> go [1.0, -1.0, 0.5, -1.0, 0.5, 0.0]
```

**5b. 对比指标 / Comparison Metrics (论文 Table 2.2):**

| 方法 | 类型 | MoveIt! ID | 成功率 | 规划时间 |
|------|------|------------|--------|----------|
| **AM-PRM (本文)** | 近似流形 + LazyPRM | — (自定义) | **100%** | **67 ms** |
| LazyPRM | 非渐近最优 | `LazyPRM` | 85% | 286 ms |
| BiTRRT | 非渐近最优 | `BiTRRT` | 85% | 128 ms |
| PRM* | 渐近最优 | `PRMstar` | 100% | 5525 ms |
| RRT-Connect | 非渐近最优 | `RRTConnect` | — | — |
| RRT* | 渐近最优 | `RRTstar` | — | — |
| BIT* | 渐近最优 | `BITstar` | — | — |
| ABIT* | 渐近最优 | `ABITstar` | — | — |
| LazyPRM* | 渐近最优 | `LazyPRMstar` | — | — |

---

#### 步骤 6：切换场景复现 / Step 6 — Repeat for Scene 2

> **对照 / Paper Ref:** Section 2.3.1 — 两个仿真场景的实验

要对场景 2 进行相同的实验流程，只需替换路线图和场景文件：

```bash
# 停止 Terminal 3 中运行的规划节点 (Ctrl+C)，然后重新启动:
roslaunch approx_manifold_prm trajectory_planner.launch \
    graph_file:=/tmp/approx_graph.pkl \
    roadmap_file:=/tmp/roadmap_scene2.pkl \
    scene_file:=$(rospack find approx_manifold_prm)/scenes/sim_scene_2.yaml
```

> **注意:** 近似图 G (`/tmp/approx_graph.pkl`) **无需重新构建**——它仅依赖于
> 约束函数，不依赖于场景。只需为新场景重新构建路线图 M。

然后用步骤 4b 的方式发送规划请求即可。

---

#### 步骤 7：完整实验流程（实物化学实验）/ Step 7 — Full Lab Experiment Sequence

> **对照 / Paper Ref:** Section 2.3.2 — 实际化学实验操作流程

在实际自动化化学实验室中，完整的端到端流程如下：

```
 步骤    操作                              论文章节        对应命令/组件
 ────    ──────────────────────────────    ──────────      ──────────────────
  1      机械臂移动到仪器附近                Section 2.3.2  MoveIt! execute
         (home → instrument vicinity)
  2      深度相机采集 RGB-D 数据流          Section 2.3.2  ROS depth camera driver
         (Intel RealSense / Azure Kinect)
  3      NeRF-SLAM 重建三维场景            Section 2.3.2  场景重建服务
         → 输出 collision objects
  4      加载场景 + 路线图到 MoveIt!        Section 2.2.5  scene_loader.py →
         → Planning Scene 更新                            load_into_moveit()
  5      LazyPRM 在线规划 (< 300 ms)       Section 2.2.5  /plan_trajectory service
         → 输出 joint-space path
  6      IPTP 计算速度/加速度参数           Section 2.2.5  MoveIt! retime_trajectory
         → 输出 RobotTrajectory
  7      机械臂执行轨迹                    Section 2.3.2  MoveIt! execute
  8      机械臂返回 home 位置              Section 2.3.2  MoveIt! go to home
```

在仿真中复现此流程，对应 ROS 命令的执行顺序为：

```bash
# 1. 确保 Terminal 1+2 中 Gazebo + MoveIt! 已启动 (步骤 1)
# 2. 确保近似图 G 已构建 (步骤 2)
# 3. 确保路线图 M 已构建 (步骤 3)
# 4. 启动在线规划节点 (步骤 4a)
roslaunch approx_manifold_prm trajectory_planner.launch \
    graph_file:=/tmp/approx_graph.pkl \
    roadmap_file:=/tmp/roadmap_scene1.pkl \
    scene_file:=$(rospack find approx_manifold_prm)/scenes/sim_scene_1.yaml

# 5. 发送规划请求 (步骤 4b)
rosservice call /trajectory_planner/plan_trajectory \
    "start_joints: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
     goal_joints:  [1.0, -1.0, 0.5, -1.0, 0.5, 0.0]
     time_limit:   10.0"

# 6. 响应中的 trajectory 字段包含 IPTP 参数化后的完整轨迹
#    可通过 RViz 的 DisplayTrajectory 话题可视化 (~/planned_path)

# 7. 在 RViz 中，观察 UR5e 执行规划轨迹
#    实物实验中，通过 MoveIt! execute 发送到机器人控制器
```

---

#### ROS 实验常见问题 / ROS Experiment Troubleshooting

**Q: `roslaunch ur5e_moveit_config moveit_planning_execution.launch` 报错找不到？**

A: 参见本文档末尾的常见问题章节。需安装 `ros-noetic-ur5e-moveit-config` 包。

**Q: `build_approx_graph.launch` 报错 `/compute_fk` 服务不可用？**

A: 确保 Terminal 2 中 MoveIt! 已完全启动。等待看到 `You can start planning now!`。

**Q: 规划时间远超 67 ms？**

A: 首次查询可能较慢（冷启动）。多次查询后时间会趋于稳定。
确保 `n_c ≥ 1000`、`n_e ≥ 5`，且路线图构建时间充足（默认 `max_time=300s`，
最低建议不少于 60s）。

**Q: 路线图构建时碰撞检测很慢？**

A: 使用 Option A（YAML 场景文件的几何碰撞检测）比 MoveIt! FCL 碰撞检测更快，
适合离线批量构建。MoveIt! 碰撞检测更精确，推荐在实物实验时使用。

---

## 包结构 / Package Structure

```
approx_manifold_prm/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── config/
│   └── robot_params.yaml          # All algorithm parameters
├── launch/
│   ├── build_approx_graph.launch  # Offline: build approximate graph
│   ├── build_roadmap.launch       # Offline: build PRM* roadmap (scene_file arg)
│   └── trajectory_planner.launch  # Online: start planning node (scene_file arg)
├── scenes/
│   ├── sim_scene_1.yaml           # Paper's simulation scene 1
│   └── sim_scene_2.yaml           # Paper's simulation scene 2
├── scripts/
│   ├── approx_manifold_graph.py   # Core: G=(V,E) construction
│   ├── prm_star.py                # PRM* offline roadmap
│   ├── lazy_prm.py                # LazyPRM online planner
│   ├── scene_loader.py            # Scene file parser + geometric collision checker
│   ├── run_planner.py             # End-to-end CLI (no ROS needed)
│   ├── build_approx_graph.py      # CLI: build graph (offline)
│   ├── build_roadmap.py           # CLI: build roadmap (offline, --scene arg)
│   ├── traditional_prm.py        # Baseline: traditional PRM (no manifold)
│   ├── benchmark.py              # Benchmark: AM-PRM vs Traditional PRM
│   └── trajectory_planner_node.py # ROS node: online planning service
├── srv/
│   └── PlanTrajectory.srv         # ROS service definition
├── src/
│   └── approx_manifold_prm/
│       └── __init__.py
└── tests/
    ├── test_approx_manifold_graph.py  # Unit tests (no ROS required)
    ├── test_scene_loader.py           # Scene loading & collision checker tests
    └── test_benchmark.py             # Benchmark & traditional PRM tests
```

---

## 关键参数说明 / Key Parameters

All parameters are in `config/robot_params.yaml`.

| Parameter | Description | Paper Value |
|-----------|-------------|-------------|
| `constraint_delta` | Upright tolerance (`r_x²,r_y² ≤ δ`) | 0.01 |
| `approx_graph.n_c` | Number of graph vertices | 10³–10⁵ |
| `approx_graph.n_e` | Max edges per vertex | 0–30 |
| `approx_graph.diversity_epsilon` | Min distance to tangent plane (eq. 2.6) | 0.05 |
| `approx_graph.diversity_rho` | Min tangent-coord distance (eq. 2.8) | 0.05 |
| `approx_graph.diversity_alpha_deg` | Max tangent-space alignment (eq. 2.7) | 30° |
| `prm_star.max_time` | Offline roadmap build budget | 300 s |
| `lazy_prm.time_limit` | Online planning time limit | 10 s |
| `lazy_prm.connect_k` | Neighbours for start/goal connection | 10 |

---

## 运行测试 / Running Tests

Unit tests do **not** require ROS or a robot:

```bash
cd ~/catkin_ws/src/approx_manifold_prm
python3 -m pytest tests/ -v
```

---

## 对比方法 / Baseline Comparisons

The paper compares against (all available in MoveIt!):

| Method | Type | MoveIt! planner ID |
|--------|------|--------------------|
| PRM | Non-asymptotically optimal | `PRM` |
| LazyPRM | Non-asymptotically optimal | `LazyPRM` |
| RRT-Connect | Non-asymptotically optimal | `RRTConnect` |
| BiTRRT | Non-asymptotically optimal | `BiTRRT` |
| PRM* | Asymptotically optimal | `PRMstar` |
| LazyPRM* | Asymptotically optimal | `LazyPRMstar` |
| RRT* | Asymptotically optimal | `RRTstar` |
| BIT* | Asymptotically optimal | `BITstar` |
| ABIT* | Asymptotically optimal | `ABITstar` |

To run a baseline:
```bash
rosrun moveit_commander moveit_commander_cmdline.py
> use manipulator
> planner PRMstar
> go [1.0 -1.0 0.5 -1.0 0.5 0.0]
```

---

## 速度基准测试 / Speed Benchmark

To verify the construction speed advantage of the proposed AM-PRM algorithm,
a benchmark script (`scripts/benchmark.py`) is provided that compares AM-PRM
against a **Traditional PRM** baseline (uniform C-space sampling, no manifold
approximation).  This follows the experimental methodology in Section 2.3 of
the paper.

### 实验方法 / Experimental Methodology

The benchmark implements the paper's comparison protocol:

1. **Control variable**: Both methods share the same scene, start/goal
   configurations, collision checker, and constraint function.
2. **AM-PRM pipeline** (proposed):
   - Phase 1: Build approximate manifold graph G = (V, E) with diversity checks
     (Algorithms 2.1 + 2.2) — timed as **graph build time**.
   - Phase 2: Build PRM* roadmap M on G — timed as **roadmap build time**.
   - Phase 3: Online LazyPRM query on M — timed as **planning time**.
3. **Traditional PRM baseline**:
   - Sample configurations uniformly in C-space, project onto the constraint
     manifold, and build a PRM roadmap directly — timed as **roadmap build time**.
   - Query via Dijkstra — timed as **planning time**.
4. Each method is run for multiple independent **trials** to compute statistics
   (mean ± std).
5. **Metrics reported** (matching Table 2.2 in the paper):
   - Offline construction time (graph + roadmap build)
   - Online planning time (ms)
   - Success rate (%)
   - Path length (joint-space L2 norm, as a quality proxy)

### 生成近似图后的实验流程 / Post-Graph Experiment Procedure

After building the approximate manifold graph G (Phase 1), the paper describes
the following experimental steps (Section 2.3.2):

```
 Phase 1 (offline, once)         Phase 2 (offline, once)          Phase 3 (online, per query)
┌───────────────────────┐       ┌───────────────────────┐       ┌───────────────────────────┐
│ Build Approx. Graph G │──────▶│ Build PRM* Roadmap M  │──────▶│ LazyPRM Query             │
│ (n_c vertices, n_e    │       │ on G with collision   │       │ • Connect start/goal to M │
│  edges per vertex)    │       │ checking against scene│       │ • Dijkstra shortest path  │
│                       │       │ (stop on convergence  │       │ • Lazy edge validation    │
│ ⏱ Record build time  │       │  or time limit)       │       │ • Remove invalid edges    │
│ 📏 Record |V|, |E|   │       │                       │       │ • Re-search if needed     │
│ 💾 Record memory      │       │ ⏱ Record build time  │       │                           │
└───────────────────────┘       │ 📏 Record |M_V|, |M_E│       │ ⏱ Record planning time   │
                                └───────────────────────┘       │ 📏 Record path length    │
                                                                │ ✅ Record success/fail   │
                                                                └───────────────────────────┘
```

For the **Traditional PRM** baseline, Phases 1 and 2 are replaced by a single
roadmap construction step that samples uniformly in C-space (with optional
constraint projection).  This makes the comparison fair: both methods produce
a roadmap, and the online query is measured separately.

### 快速运行 / Quick Start (No ROS Required)

**Toy benchmark** (2-D unit-circle constraint, fast):

```bash
cd approx_manifold_prm
python3 scripts/benchmark.py --mode toy --trials 5 --seed 42
```

**Scene benchmark** (UR5e + YAML scene file):

```bash
python3 scripts/benchmark.py --mode scene \
    --scene  scenes/sim_scene_1.yaml \
    --start  "0,0,0,0,0,0" \
    --goal   "0.3,-0.2,0.1,-0.1,0.1,0" \
    --n_c 200 --n_e 3 --trials 5
```

**Save results as JSON** for further analysis:

```bash
python3 scripts/benchmark.py --mode toy --trials 10 \
    --seed 42 --output /tmp/benchmark_results.json
```

### 基准参数 / Benchmark Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--mode` | `toy` | `toy` (2-D circle) or `scene` (UR5e + YAML) |
| `--scene` | — | Path to YAML scene file (required for `scene` mode) |
| `--start` | — | Start joints, comma-separated (required for `scene` mode) |
| `--goal` | — | Goal joints, comma-separated (required for `scene` mode) |
| `--n_c` | 200 | AM-PRM graph vertex count |
| `--n_e` | 3 | AM-PRM max edges per vertex |
| `--n_samples` | = n_c | Traditional PRM sample count |
| `--roadmap_time` | 10 s | AM-PRM roadmap build budget |
| `--trad_build_time` | 30 s | Traditional PRM build budget |
| `--plan_time` | 10 s | Online planning time limit |
| `--trials` | 5 | Number of independent trials per method |
| `--seed` | — | Base random seed (trial *i* uses seed + *i*) |
| `--output` | — | Save JSON results to this path |

### 预期输出 / Expected Output

```
========================================================================
  Benchmark Comparison: AM-PRM vs Traditional PRM
========================================================================
  Metric                                     AM-PRM    Traditional PRM
  --------------------------------------------------------------------
  Trials                                          5                  5
  Success Rate                               100.0%             100.0%
    Graph Build Time (s)               21.70 ± 0.08                N/A
    Roadmap Build Time (s)              0.04 ± 0.00        0.15 ± 0.00
  Offline Build Time (s)               21.74 ± 0.09        0.15 ± 0.00
  Online Plan Time (ms)                 2.72 ± 0.21        3.94 ± 0.18
  Path Length (rad)                     2.39 ± 0.22        3.10 ± 0.01
========================================================================
```

> **Note:** AM-PRM's offline graph build is a one-time cost; once built, the
> graph is reused across scenes and queries.  The key metric from the paper is
> **online planning time**, where AM-PRM achieves significantly faster query
> performance (67 ms vs 5525 ms for PRM* in the paper's Table 2.2).

---

## 常见问题 / Troubleshooting

### `RLException: [moveit_planning_execution.launch] is neither a launch file in package [ur5e_moveit_config] …`

This error means the `ur5e_moveit_config` ROS package is not installed or
not on ROS's package path.  It typically happens when `ur_gazebo` was
installed (e.g. via the `universal_robot` meta-package) but
`ur5e_moveit_config` was not.

`ur_gazebo` and `ur5e_moveit_config` are **separate packages** inside the
[`universal_robot`](https://github.com/ros-industrial/universal_robot)
meta-package.  `ur_gazebo` provides the Gazebo simulation (which is why
`roslaunch ur_gazebo ur5e_bringup.launch` succeeds), while
`ur5e_moveit_config` provides the MoveIt! configuration and launch files
needed for motion planning.

**Fix:**

1. Install `ur5e_moveit_config` via apt:

   ```bash
   sudo apt-get install ros-noetic-ur5e-moveit-config
   ```

   If the apt package is unavailable, install from source instead:

   ```bash
   cd ~/catkin_ws/src
   git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   ```

2. Source the workspace:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

3. Verify the package is found:

   ```bash
   rospack find ur5e_moveit_config
   ```

4. Retry:

   ```bash
   roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
   ```

---

## 引用 / References

- Karaman, S. & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. *IJRR*.
- Bohlin, R. & Kavraki, L. (2000). Path planning using lazy PRM. *ICRA*.
- MoveIt! documentation: https://moveit.ros.org/
- UR5e MoveIt! config: https://github.com/ros-industrial/universal_robot
