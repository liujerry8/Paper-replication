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

### 2. 创建工作空间 / Create Catkin Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# Copy this package into src/
cp -r /path/to/approx_manifold_prm .
cd ~/catkin_ws

# Install all declared dependencies (including ur5e_moveit_config, ur_robot_driver, etc.)
rosdep install --from-paths src --ignore-src -r -y

catkin_make
source devel/setup.bash
```

### 3. 启动机器人 / Launch Robot

**Simulation (Gazebo):**
```bash
roslaunch ur_gazebo ur5e_bringup.launch
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
```

**Real Robot:**
```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<ROBOT_IP>
roslaunch ur5e_moveit_config moveit_planning_execution.launch
```

### 4. 离线阶段：构建近似图 / Offline: Build Approximate Manifold Graph

Run once per constraint type. For the paper's upright constraint (`r_x²≤0.01, r_y²≤0.01`):

```bash
# n_c=1000 for quick test; use n_c=10000 for paper-quality results
roslaunch approx_manifold_prm build_approx_graph.launch \
    n_c:=1000 n_e:=5 output:=/tmp/approx_graph.pkl
```

Expected time (from Table 2.1 in the paper):

| n_c / n_e | Build Time (s) | Memory (MB) |
|-----------|---------------|-------------|
| 1,000 / 0 | 30.76 | 0.07 |
| 1,000 / 1 | 30.96 | 0.15 |
| 10,000 / 5 | 348.51 | 7.31 |
| 100,000 / 30 | 5750.66 | 454.36 |

### 5. 离线阶段：构建路线图 / Offline: Build PRM* Roadmap

**Option A – Using a scene file** (geometric collision checking, no extra setup):

```bash
roslaunch approx_manifold_prm build_roadmap.launch \
    graph:=/tmp/approx_graph.pkl \
    output:=/tmp/roadmap.pkl \
    max_time:=300 \
    scene_file:=$(rospack find approx_manifold_prm)/scenes/sim_scene_1.yaml
```

**Option B – Using MoveIt! planning scene** (requires scene reconstruction first):

```bash
# Load collision objects from YAML into MoveIt!
# (NeRF-SLAM output or manually added objects)
roslaunch approx_manifold_prm build_roadmap.launch \
    graph:=/tmp/approx_graph.pkl \
    output:=/tmp/roadmap.pkl \
    max_time:=300
```

### 6. 在线规划 / Online Planning

Start the trajectory planner node, optionally loading a scene file at startup:

```bash
roslaunch approx_manifold_prm trajectory_planner.launch \
    graph_file:=/tmp/approx_graph.pkl \
    roadmap_file:=/tmp/roadmap.pkl \
    scene_file:=$(rospack find approx_manifold_prm)/scenes/sim_scene_1.yaml
```

Send a planning request:

```bash
rosservice call /trajectory_planner/plan_trajectory \
    "start_joints: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
     goal_joints:  [1.0, -1.0, 0.5, -1.0, 0.5, 0.0]
     time_limit:   10.0"
```

### 7. 实验流程（实际化学实验）/ Full Lab Experiment Sequence

As described in Section 2.3.2 of the paper:

```
1. Robot moves from home to instrument vicinity
2. Depth camera captures RGB-D stream
3. NeRF-SLAM reconstructs the scene
4. MoveIt! loads scene + roadmap
5. LazyPRM plans trajectory (< 300 ms)
6. IPTP computes velocity/acceleration profile
7. Robot executes trajectory
8. Robot returns to home position
```

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
