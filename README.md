# 工序自适，群臂协同
## 面向多工艺柔性产线的多机械臂自主调度与效能优化系统

> CR5 Assembly Team — 江科大学生参赛项目仓库
> 当前主线统一为：**八台 DOBOT CR5A（R1–R8）+ 电控柜装配 + 多订单生产工作流**（真实柜体 STL 50% 缩比、14 个零件，含预装内安装板；30个动作、68个 PARK/APP/TCP 规划点；不装柜门）。
> 早期五臂电控箱方案的历史模块与文档仍保留在仓库中（见「历史内容说明」）。

---

## 1. 当前项目状态

**当前版本：** 无柜门、开口朝上、无圆台的八机械臂场景，30动作/68规划点、schema36。三柜三级流水线已完成真实回放：空柜从6米西侧上料带按1.5米安全节距排队，公共区1/2/3可同时执行不同柜体任务，同一公共区仍保持单臂互斥；`--speed` 同时作用于机械臂回放和各段传送带。

2026-09-08 审计纠正：旧版将形状包围盒坐标系误当作网格坐标系，导致真实柜壳/门板朝向与代理不一致。旧版 33/33 通过记录不能用于整线验收。目前已改为背面朝托盘、柜门开口朝上，正在按真实几何逐臂重规划；尚未完成整线验收。

| 部分 | 当前内容 | 状态 |
|---|---|---|
| 8 臂 CoppeliaSim 场景 | `scenes/compact_cell.ttt`：8 台 CR5A、远端排队上料带、加长中央索引输送线、末端成品料框、三个公共区、68 个规划点 | 开口朝上，无旧圆盘、右上展示柜或右下废弃输送带；三柜流水线回放通过 |
| 工艺自动拆解 | `test/decompose_assembly.py`：STL 解析 → 接触图 → 装配顺序 DAG → 工艺分类 → 8 臂能力映射（输入一个装配好的柜体模型，输出工艺链 JSON） | MVP 已可用（含 tkinter 界面 `test/import_cabinet_ui.py`） |
| 运动规划与执行 | `configs/motion_planning_policy.yaml` + `scripts/run_8arm_cabinet_assembly.py`：竖直 Π 形模板 → 分级回退 → 机间碰撞预检 → 确定性步进回放 | 按真实柜壳检查碰撞、实际抓取接触及释放误差；重验证中 |
| 固定路径数据 | `data/fixed_paths/eight_arm_cabinet.json`（历史正式计划）与 `.partial.json`（新检查点） | 当前规划器要求 schema 36；旧 schema 34/35 不可直接执行 |
| 调度与编排 | `scripts/run_8arm_pipeline_assembly.py`：三柜独立上下文、依赖图调度、供料区预抓、安装区单臂互斥 | 已完成最高五臂交错回放；瓶颈模块二在 speed=3 下由 1583 降至 1323 显示帧，约缩短 16.4% |

机械臂分工（当前权威分工见 `configs/assembly_task_assignment.yaml` 与场景目标点）：

| 资源 | 任务 |
|---|---|
| R1 | 柜壳上料并定位到托盘 |
| R2 | 磁吸安装横轨 |
| R3 | 夹持安装两根竖轨 |
| R4 | 安装 PSU、Servo、EDS |
| R5 | 吸附安装 PLC、DMA |
| R6 | 在公共区 2 安装 Contactor、Breaker |
| R7 | 所有器件装好后的四点锁紧动作（运动仿真，不代表定扭验收） |
| R8 | 在公共区 3 吸附安装 COM5、Filter，先于 R7 执行 |

所有新路径必须遵守 `vertical_pi_transfer_v1`：末端物理轴朝世界 `-Z`，按 PARK/高位/APP/TCP 完成抓取、抬升、平移、下探和释放；回退顺序固定为“直接 Π 形 → 抬高 → 单侧高位绕点 → 替代 IK 分支 → 受约束 OMPL”。三个公共区执行单机械臂互斥，只有公共区外的取料/预取可以并行。策略文件指纹写入路径文件，策略一旦修改，旧路径自动失效。

WB1 内采用一次短距离工艺步进：R3 安装竖轨 A 后，八臂必须全部回到 PARK，托盘和已装箱体沿输送方向 `+X` 移动 120 mm，再执行竖轨 B。R3-B 的 APP/TCP 和装配基准同步使用 `wb1_micro`，不是仅移动视觉模型。

WB2 同样采用一次 120 mm 工艺步进：R5 安装 DMA 后，八臂回到 PARK，托盘由 `wb2` 沿 `+X` 移到 `wb2_micro`，再由 R6 安装 Contactor 与 Breaker。随后托盘进入 staging，R8 使用小型真空吸具依次安装 COM5 与 Filter，最后 R7 锁紧。

柜体由 R1 上料后始终开口朝上，仅由托盘沿输送线移动。R8 使用12mm单吸盘从真实平面抓取滤波器，不再执行装门、闭锁或整柜搬运。

STL 导入后保留真实网格世界姿态并统一形状坐标原点，通过三角形顶点检查世界包围盒，不再根据独立包围盒轴系猜测旋转。柜壳直接使用真实三角网格做碰撞检查；其他器件的代理与真实 CAD 坐标一致。参考成品改用不可见基准点，机器人重复显示的碰撞网格已隐藏但仍参与碰撞检测。下文涉及旧回放结果及固定路径数值的历史说明须以新审计报告为准。

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

# 推荐：逐台机械臂增量规划到 partial 检查点，其他七臂保持碰撞检查后的 PARK
python3 scripts/run_8arm_cabinet_assembly.py --plan-robot R1 --rebuild-plan
python3 scripts/run_8arm_cabinet_assembly.py --plan-robot R2
python3 scripts/run_8arm_cabinet_assembly.py --plan-robot R3
python3 scripts/run_8arm_cabinet_assembly.py --plan-robot R4
python3 scripts/run_8arm_cabinet_assembly.py --plan-robot R5
python3 scripts/run_8arm_cabinet_assembly.py --plan-robot R6
python3 scripts/run_8arm_cabinet_assembly.py --plan-robot R7
python3 scripts/run_8arm_cabinet_assembly.py --plan-robot R8
# 若只需重算某一台，组合 --plan-robot Rx --rebuild-plan；此前机械臂检查点会保留

# 用当前 partial 累计审计 WB1 的 R1→R2→R3、真实工件挂接和 120 mm 小步进
python3 scripts/run_8arm_cabinet_assembly.py \
  --plan data/fixed_paths/eight_arm_cabinet.partial.json --preflight-wb1

# 累计审计到 R4（含 wb1_micro→wb2 输送及三个器件的真实带载装配）
python3 scripts/run_8arm_cabinet_assembly.py \
  --plan data/fixed_paths/eight_arm_cabinet.partial.json --preflight-r4

# 累计审计到 R5（含 PLC、DMA、Filter 的真实真空带载装配）
python3 scripts/run_8arm_cabinet_assembly.py \
  --plan data/fixed_paths/eight_arm_cabinet.partial.json --preflight-r5

# 累计审计到 R6（含 WB2 微移、Contactor/Breaker，并转入 staging）
python3 scripts/run_8arm_cabinet_assembly.py \
  --plan data/fixed_paths/eight_arm_cabinet.partial.json --preflight-r6

# 完整审计 R1→R8、三段跨工位索引、两次 120 mm 微移和最终输出
python3 scripts/run_8arm_cabinet_assembly.py \
  --plan data/fixed_paths/eight_arm_cabinet.json --preflight-only

# 规划 + 执行（确定性步进回放）
python3 scripts/run_8arm_cabinet_assembly.py --rebuild-plan
# 常用参数：--host/--port（默认 127.0.0.1:23000）、--speed 0.2..3.0、--skip-preflight

# 推荐演示：三个空柜排队上料、三个公共区流水并行、三个成品依次输出
python3 -u scripts/run_8arm_pipeline_assembly.py \
  --port 23000 \
  --plan data/fixed_paths/eight_arm_cabinet.partial.json \
  --jobs 3 \
  --speed 3.0
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
                                          创建 74 个 PARK/APP/TCP 规划点、保存场景
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
