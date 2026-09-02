# 工序自适，群臂协同
## 面向多工艺柔性产线的多机械臂自主调度与效能优化系统

> CR5 Assembly Team — 江科大学生参赛项目仓库
> 当前主线统一为：**八台 DOBOT CR5A（R1–R8）+ HXGN-12 高压控制柜装配 + 多订单生产工作流**（真实柜体 STL 50% 缩比、14 个零件 27 个实例、72 个工序目标点）。
> 早期五臂电控箱方案的历史模块与文档仍保留在仓库中（见「历史内容说明」）。

---

## 1. 当前项目状态

当前主线已形成「工艺自动拆解 → 场景构建 → 固定路径规划 → 确定性回放」的可运行链路：

| 部分 | 当前内容 | 状态 |
|---|---|---|
| 8 臂 CoppeliaSim 场景 | `scenes/compact_cell.ttt`：8 台 CR5A、流水线（上料 → WB1 → 交接 → WB2 → 暂存 → 成品带 → 成品筐）、340 个场景对象、72 个工序目标点 | 由场景构建链生成并保存，契约记录于 `configs/scene_contract.yaml` |
| 工艺自动拆解 | `test/decompose_assembly.py`：STL 解析 → 接触图 → 装配顺序 DAG → 工艺分类 → 8 臂能力映射（输入一个装配好的柜体模型，输出工艺链 JSON） | MVP 已可用（含 tkinter 界面 `test/import_cabinet_ui.py`） |
| 运动规划与执行 | `scripts/run_8arm_cabinet_assembly.py`：APP/TCP → simIK 关节空间固定路径 → 机间碰撞预检 → 确定性步进回放；拾取吸附工具、放置吸附装配基准 | HXGN-12 流程 R1–R6 走廊调优中 |
| 固定路径数据 | `data/fixed_paths/eight_arm_cabinet.json`（完整版）与 `.partial.json`（部分版本） | 与当前场景绑定 |
| 调度与编排 | `scheduler/`（订单解析、动态订单窗口、前后段重叠流水）+ `orchestration/cell_orchestrator.py` | 沿自五臂阶段，适配 8 臂中 |

机械臂分工（工艺拆解 MVP 的初始能力映射，权威分工见 `test/process_chain_hxgn.json` 与场景目标点）：

| 资源 | 任务 |
|---|---|
| R1 | 上料 |
| R2 / R3 | 平板与杆件装配 |
| R5 / R6 | 器件装配 |
| R7 | 锁付 |
| R8 | 分拣 |

---

## 2. 快速开始

推荐环境：Ubuntu 22.04、CoppeliaSim Edu 4.x（含 ZMQ Remote API 与 simIK 插件）、Python 3。

```bash
pip install -r requirements.txt
pip install coppeliasim-zmqremoteapi-client

# 启动场景（默认打开 scenes/compact_cell.ttt，可用环境变量 CR5_SCENE_PATH 覆盖）
bash scripts/start_coppelia_ubuntu.sh

# 仅重新规划，输出/更新固定路径 JSON
python3 scripts/run_8arm_cabinet_assembly.py --rebuild-plan --plan-only

# 规划 + 执行（确定性步进回放）
python3 scripts/run_8arm_cabinet_assembly.py --rebuild-plan
# 常用参数：--host/--port（默认 127.0.0.1:23000）、--speed 0.2..3.0、--skip-preflight
```

新柜体工艺拆解：

```bash
# 离线拆解：把新柜体的零件 STL 放入 test/models/<柜体名>/，然后：
python3 test/decompose_assembly.py test/models/<柜体名>

# 或图形界面（导入模型 → 解析工艺链 → 应用到场景）：
python3 test/import_cabinet_ui.py
```

单元测试：

```bash
python3 -m unittest discover -s tests -v
```

---

## 3. 场景构建链

当前 8 臂 HXGN-12 场景由以下脚本按序生成（幂等，可重复执行）：

```text
scripts/rename_scene_robots.py       → 生成 R1–R8 8 臂产线（传送带、工作台、交接、料筐）
scripts/preprocess_cabinet_models.py → 预处理柜体 STL，输出 models/cabinet/processed/
scripts/build_cabinet_product_scene.py → 导入 14 个零件（27 实例）、放置到各工位、
                                          创建 72 个工序目标点、保存场景
```

构建结果与基线记录在 `configs/scene_contract.yaml`（场景哈希、对象数量、目标点数量）。

---

## 4. 已知边界与注意事项

- 只控制 CoppeliaSim 内的模型，**不连接真实机械臂**；ROS2 工作空间（`src/`）当前未接入主线执行。
- 端口：每次启动前确认 2300x 端口已释放——旧实例残留会导致控制脚本连到错误实例；多次 start/stop 后实例可能污染（`getSimulationState()` 异常），测试请用新鲜实例。
- kinematic 关节下 `setJointTargetPosition` 不产生实际运动，运行时经 Lua 逐帧 `setJointPosition` 驱动；验证视觉运动用 Link6/tip 位置，不要用 Link1_visual（绕 joint1 旋转对称，位置不变是正常现象）。
- 工艺拆解待办：各机器人「朝下可达区」离线网格表、传送工位自动插入、与运动规划器/场景生成器全自动打通。

---

## 5. 仓库结构

```text
configs/          场景契约（scene_contract.yaml）与点位配置（points.yaml）
data/fixed_paths/ 8 臂固定关节路径（完整版 + partial 版）
scenes/           compact_cell.ttt（8 臂 HXGN-12 装配场景）
scripts/          场景构建链与运行/规划脚本
robot_control/    运动原语（motion_common：运行时桥接脚本等）
sim_bridge/       CoppeliaSim ZMQ Remote API 封装（scene_objects、coppelia_client）
scheduler/        调度层：订单解析、任务生成、动态订单窗口、流水重叠
orchestration/    统一编排器（cell_orchestrator）
test/             工艺自动拆解 MVP、可达性映射（reachability_map）、工艺链 JSON
tests/            单元测试（运动协调等）
models/           零件 STL 与预处理产物（models/cabinet/processed）
src/              DOBOT ROS2 工作空间
docs/             设计文档（含早期五臂方案记录）
app/ mock/ interfaces/ output/  早期演示、Mock 与接口模块（历史）
```

---

## 6. 历史内容说明

- 本仓库为 8 臂 HXGN-12 工作流的快照仓库（单次初始提交起步）；早期五臂电控箱方案的模块（`app/`、`mock/`、`interfaces/` 及 `docs/` 中部分文档）仍保留在树中。
- `docs/` 内各文件头部多有状态说明（如 `PROJECT_PLAN.md` 注明其为四臂方案草案）；与五臂场景相关的内容属历史方案，以本文档为当前状态基准。
- 团队协作与分支规则见 `docs/TEAM_WORKFLOW.md`；`build/`、`install/`、`log/` 及缓存、IDE 临时文件不应提交。
