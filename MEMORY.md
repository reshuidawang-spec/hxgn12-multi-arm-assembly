# CoppeliaSim 八机械臂电控柜装配：对话续接记忆

## 当前任务（用户确认无柜门；优先于全部历史记录）

用户明确要求：R1 搬开口朝上的柜壳到托盘，后续柜体始终躺在输送线上；不补建柜门。已实施 `cabinet_open_up_v2`：原 CAD 绕 X 翻转180°，真实背板在下、开口在上，中心射线检查通过。原 `door` 零件更名 `mounting_panel`，作为柜壳预装件随 R1 运动。R8 改为用12mm吸盘安装滤波器；R5 仅 PLC/DMA；流程 R1→R2→R3→R4→R5→R6→R8→R7→输出，R7最后锁紧。30动作/68规划点，schema36，旧34/35路径不适用。

场景已重建保存 `scenes/compact_cell.ttt`，配置/任务/点位注册已同步，69测试通过1跳过。R1新两动作规划已生成，正在做真实抓取/携件/释放前缀审计；尚未宣称整线通过。R7 TCP 已按真实折边表面从z=.412修到约.405，而非悬空。源文件备份在 `data/audits/open_top_no_door_v2/`。用户表示DDL临近，优先正确场景和可运行链路，不扩展柜门或无关功能。工作区可能有用户/其他进程的自动提交及新 `scripts/assembly_demo_ui/`，不要覆盖。

最新进度：R1已通过真实双侧橡胶垫接触、携件运动、释放姿态前缀审计，报告 `data/audits/open_top_no_door_v2/prefix_R1.json`。IK接触精度从3mm收紧到0.1mm；机械夹持用实际左右橡胶垫与真实工件网格的双侧碰撞接触检查，而非零厚度TCP射线（薄折边会被射线误判）。磁头压入量从1mm减至0.1mm，避免穿过0.5mm导轨底板碰到内安装板。正在连续规划其余动作。

## 2026-09-08 审计纠正（优先于下方历史完成记录）

### 最新阻断：开口面与“门板”身份此前仍被误认

真实网格射线和 `data/audits/20260908_geometry/cad_surfaces.png` 证明：`cabinet_front_up_v1` 的 +Z 面是连续背板，不是开口。中心多条射线均命中 z=0.14139429..0.14199430 m，R2 导轨目标顶面为 z=0.13479428 m，安装路径必穿背板。正确开口方向是原 CAD -Z，需要整体 Rx(180°) 等正交旋转并重算全部基准；不能仅修代理或豁免柜壳碰撞。

旧称 `door` 的原始文件 `2018081800220728` 位于 CAD z=264.8..282.8 mm，紧邻背板和导轨后侧，结构更符合内安装板，尚无零件图/BOM 证明它是柜门。不得继续把它放在柜体底部称作最后装门，也不能未经说明从真实 BOM 中删除它并编造完成。需用户确认保留它作为内安装板、补建简化柜门，或提供真实柜门模型。

本轮 R1 两动作通过真实抓取接触、携件路径和释放误差检查（仅对当前错误开口场景，不代表目标姿态验收）。R2 窄磁头旧四吸盘载体已清除；R5 已替换为 12 mm 单吸盘。R2 重规划仍因真实背板阻挡失败。运行入口现增加开口面强校验，在姿态/零件定义修正前拒绝继续规划和回放。正式 schema34 路径仍是历史文件。项目 tests 最后为68通过1跳过（新增屋顶回归测试后需重跑）。

真实三角网格审计证明旧版 STL 朝向修复错误：把 oriented BB 坐标系误当作 shape 坐标系，柜壳实际为 142×240.5×452.5 mm，碰撞代理却为 452.5×240.5×142 mm。R8 门板同样立起。历史“视觉/碰撞一致、33/33 已完成”不能作为实际几何验收依据。GUI 最后一次回放在 R8 ZMQ 超时，并未完成。

用户已授权全部代码、姿态和逐臂路径修正。正在采用 `cabinet_front_up_v1`（背面朝托盘、柜门所在正面朝 +Z），规划 schema 升级为 35；旧 schema 34 计划不可执行。已备份到 `data/audits/20260908_geometry/`。目前处于重建及逐臂验证阶段，不得宣称整线已验收。

卡顿审计：约 90.7 万三角形中，约 32.6 万为与视觉网格重复显示的机器人碰撞网格，参考成品约 11.2 万。正在隐藏前者但保留碰撞用途，把参考成品替换成轻量装配基准 dummy。

更新时间：2026-09-07（Asia/Shanghai）

工作目录：`/home/zhu/cr5_assembly_team`

## 用户最终确认的设计方向

公共工作区改为一条低矮的托盘式步进输送线。机械臂只执行取料、安装、锁紧等局部工艺，不再抓取完整电控柜跨工位搬运。路径原则依次为：真实几何避障、末端法兰尽可能朝下、多机械臂联动、到工件上方后垂直下探/抓取/上升、到目标上方后垂直下放。

2026-09-07 最新状态：中央输送带保持净空，并沿输送方向设置三个公共工作区。公共区 1 由 R1/R2/R3 共享，公共区 2 由 R4/R5/R6 共享，公共区 3 由 R6/R7/R8 共享（R6 是相邻区域的共享资源）。R1–R8 共33个动作及其 APP/TCP 均已完成；无圆盘场景、全流程携件碰撞预飞、两次 120 mm 工艺微移、R8 真空门板安装及最终输出均已通过。详细 STL 的导入轴系也已校正，视觉外形与碰撞代理处于同一世界位姿。

当前工艺分配：

- R1（公共区1）：抓取柜壳并在步进托盘上定位。
- R2（公共区1）：使用14 mm磁吸头抓取并安装横轨。
- R3（公共区1）：使用窄夹爪依次安装两根竖轨。
- R4（公共区2）：侧夹安装 PSU、Servo、EDS 三个高窄器件。
- R5（公共区2）：真空吸附安装 PLC、DMA、Filter 三个平面器件。
- R6（公共区2/3共享）：安装 Contactor、Breaker，并在公共区3安装 COM5。
- R7（公共区3）：完成四个内部安装点的定扭锁紧。
- R8（公共区3）：使用 50 mm 真空吸盘抓取柜门，完成铰链对准、装门和闭锁；不搬运完整柜体。
- 输送托盘负责公共区1 → 公共区2 → 公共区3 → OUTPUT 的整柜转运。
- R3 安装竖轨 A 后，托盘和箱体在 WB1 内沿输送方向 `+X` 小步进 120 mm 到 `wb1_micro`，随后安装竖轨 B。
- R5 安装 Filter 后，托盘和完整装配体在 WB2 内沿输送方向 `+X` 小步进 120 mm 到 `wb2_micro`，随后 R6 安装 Contactor 与 Breaker；托盘再进入 STAGING，R6 安装 COM5。
- 公共区执行规则：区外取料/预取可并行；同一公共区内一次只允许一台机械臂进入柜体上方作业。

## 已完成

### 场景结构

- 已在 `scenes/compact_cell.ttt` 中建立中央步进输送线 `Central_Indexing_Conveyor`。
- 输送线中心约为 `(-1.20, 0.25)`，长度 4.50 m，宽度 0.46 m。
- 已建立 `Indexing_Pallet_1`，尺寸 580 × 380 × 25 mm。
- 输送带顶面为 Z=0.245 m，托盘顶面保持 Z=0.270 m，因此原工艺 TCP 高度没有整体下移。
- 已建立 WB1、WB2、FASTEN、OUTPUT 四组站边锁止器。
- 工位坐标：
  - WB1 `[-3.15, 0.25, 0.27]`
  - WB1_MICRO `[-3.03, 0.25, 0.27]`
  - WB2 `[-1.20, 0.25, 0.27]`
  - WB2_MICRO `[-1.08, 0.25, 0.27]`
  - STAGING/FASTEN `[0.05, 0.25, 0.27]`
  - OUTPUT `[0.75, 0.25, 0.27]`
- 已移除旧 WB1/WB2 抬高夹具、handoff/staging 中转台和旧末端成品输送带的流程用途。
- 装配体现在真实父子绑定到 `Indexing_Pallet_1`，托盘移动时所有已装工件一起移动，不是只改视觉位置。
- 已删除 `RobotBases` 组和脱离机器人模型的 `R1_Base`～`R8_Base` 八个遗留圆盘；机器人模型自带的 `base_link_visual/base_link_respondable` 才是真实底座。
- 已删除 `WB1_Table`、`Damping_Table_Left`、`Damping_Table_Right` 三张直径 1.9 m 的旧圆台及 24 个橡胶垫。工件只由步进托盘承载。
- R6 最终位于 `[-0.50, -0.08, 0.19]`，R7 位于 `[0.10, 0.75, 0.19]`，R8 位于 `[0.35, -0.22, 0.19]`。R6/R8 已实时确认与中央输送线无碰撞。
- 三个公共区中心分别为 `[-3.15, 0.25]`、`[-1.20, 0.25]`、`[0.05, 0.25]`，场景层级中为 `Public_Workspace_1`～`Public_Workspace_3`。
- 参考成品和小型展示垫已移到东北角 `[1.65, 1.25, 0.27]`，不再占用公共工艺区。

### 工件碰撞代理

- 详细 STL 保留用于显示。
- 轨道和器件增加与真实外包络一致的保守盒式碰撞代理，别名前缀为 `COL_`。
- 柜壳使用四面开放框架代理，允许工具从上方进入，但不允许穿过柜体边框。
- 场景构建器会按每个 STL 的 CoppeliaSim 本地包围盒自动求解轴系校正，并对碰撞代理应用逆变换。门板与柜壳的视觉 STL/代理世界包围盒已精确重合，R8 吸盘 TCP 位于门板表面。
- 场景构建器已修复非递归删除问题：会显式删除中央输送/托盘完整子树以及中断后遗留在机械臂工具下的工件。
- 运行器启动时可从整个场景找回中断后仍挂在工具下的工件，并按固定路径文件中的初始父节点和矩阵复位。

### 运动流程和路径

- `PLAN_SCHEMA_VERSION = 34`；规划策略为 `configs/motion_planning_policy.yaml` 中的 `vertical_pi_transfer_v1`，场景/策略指纹不一致时旧路径与 partial 检查点都会自动失效。
- 默认路径严格采用竖直 Π 形闭环：PARK → SOURCE_HIGH → PICK_APP → PICK_TCP → 抓取 → PICK_APP → SOURCE_HIGH → TARGET_HIGH → PLACE_APP → PLACE_TCP → 释放 → PLACE_APP → READY/PARK。末端默认最大倾角 6°，偏航只能在安全高度改变。
- 回退顺序固定为：直接竖直 Π 形 → 每次抬高 50 mm → 单个侧向高位绕点 → 替代 IK 分支 → 最后才使用受高度/碰撞/竖直姿态约束的 OMPL。
- 公共区动作在执行层强制单机械臂互斥；只允许公共区外取料/预取并行。APP↔TCP 接触段可仅豁免工件与指定夹具的预期接触，机械臂连杆永不豁免。
- 当前动作图共有 33 个实际工艺动作（旧固定路径仍只有旧版24条，禁止执行）：
  - R1 2 个
  - R2 2 个
  - R3 4 个
  - R4 6 个
  - R5 6 个
  - R6 6 个
  - R7 4 个
  - R8 3 个
- 场景现有 74 个规划点：8 个 PARK/HOME_REF，加上33个动作各一对 APP/TCP。
- `scripts/audit_task_tcp_reachability.py` 已对33/33动作完成本臂+固定环境的APP IK及APP→TCP逐采样垂直下探检查。R3-B使用约5°补偿（低于6°上限），其余目标工具轴严格朝世界-Z。
- 新增 `--plan-robot R1..R8` 增量规划入口：始终保留八台机械臂并验证全部 PARK，但只生成指定机械臂的动作到 schema 34 partial 检查点；未完成33个动作前不会覆盖正式路径文件。
- 在 `--plan-robot` 模式下加 `--rebuild-plan` 只重建所选机械臂的 stow/动作，保留此前已通过机械臂的检查点；完整规划模式的 `--rebuild-plan` 语义不变。
- 普通放置动作现在在生成阶段挂载真实被搬工件，并把“工件↔自身连杆/其他机器人/环境”的碰撞对送入直接 Π 形、抬高、侧向绕点、替代 IK 和受约束 OMPL 各层回退。R1 柜壳放置因此自动选择了 `alternate-IK high corridor`，没有豁免 Link2。
- R2 的磁吸工具没有左右夹指，带载规划辅助函数已支持无夹指工具；R2 PARK 固定在横轨取料 APP 正上方 `[-3.76,-0.725,0.50]`，避免长横轨回缩到 Link2。横轨安装采用0.65 m直接竖直 Π 形高位通道。
- 已从动作图中删除 8 个完整柜体搬运动作：
  `WB1_PICK`、`HANDOFF_PLACE`、`HANDOFF_PICK`、`WB2_PLACE`、`WB2_PICK`、`STAGING_PLACE`、`STAGING_PICK`、`OUTPUT_PLACE`。
- R4、R8 使用已验证的法兰朝下安全停靠位。
- 三次跨工位步进及 WB1/WB2 内两次 120 mm 小步进前都会检查八台机器人均回到各自 stow，随后逐帧检查“托盘 + 完整装配体”与夹具、参考柜和所有机器人碰撞。
- R6 的 Contactor、Breaker、COM5 六条路径均采用“高位源点 → 高位目标点 → APP → 垂直 TCP”的直接竖直 Π 形走廊；Contactor/Breaker 在 `wb2_micro` 安装，COM5 在 STAGING 安装。为避开相邻机械臂与环境，R6 stow 及 COM5 高位转运高度使用 0.48 m。
- R3-B 的目标已随小步进从 x=-3.35145 m 更新为 x=-3.23145 m；APP/TCP 分别为 z=0.400/0.29725 m。新路径使用 `alternate-IK high corridor`，共675帧、TCP位于第337帧，末端约5°补偿且低于6°上限。

### 预检性能和稳定性修复

- 详细 STL 对 STL 的全场景碰撞极慢，因此预检改用上述保守碰撞代理。
- 停止态预检已改为 CoppeliaSim 内部批量扫帧，不再由 Python 每帧发一次远程调用。
- 最新批量实现每批最多 120 帧。运行态批扫会保存并恢复 `sim.setStepping(true)` 层级；停止态（bareLua）不调用 stepping，避免 `attempt to yield from outside a coroutine`。修复后 R1→R6 累计流程已完整重跑通过。
- 远程 API 的 RCVTIMEO/SNDTIMEO 已提高到 900000 ms。
- 新增参数：
  - `--preflight-only`
  - `--preflight-wb1`（允许直接使用未完成的 partial 累计审计 R1–R3）
  - `--preflight-r4`（累计审计 R1–R4、WB1→WB2 输送及 R4 三器件装配）
  - `--preflight-r5`（累计审计 R1–R5 及 R5 三个真空吸附器件装配）
  - `--preflight-r6`（累计审计 R1–R6、WB2 微步进、WB2→STAGING 及 R6 三器件装配）
  - `--audit-existing-plan`（必须与 `--preflight-only` 同用）
- `--audit-existing-plan` 只有在完整预检通过后才会把当前场景指纹写入固定路径文件。

### 已通过的检查

- `python3 -m py_compile scripts/build_cabinet_product_scene.py scripts/run_8arm_cabinet_assembly.py` 通过。
- `python3 -m pytest tests -q`：63 passed，1 skipped。
- `python3 -m unittest tests.test_motion_coordination`：14/14 通过。
- 多次代理预检中已经确认：
  - R1/R2/R3 并行取料通过。
  - R1 柜壳落到当前托盘通过。
  - R2 横轨安装通过。
  - R3 第一根竖轨安装至少完整通过过一次。
- 曾发现 R1 在第 289/578 帧与 `Indexing_Pallet_Deck[0]` 碰撞，最终确认是历史旧托盘甲板孤儿对象；递归清理后同一帧已通过。
- 当前 R1 schema 34 检查点路径已独立复核通过：`SHELL_PICK` 85 帧、`WB1_PLACE` 717 帧，共802帧；取料回撤携带柜壳，放置转运使用带载替代 IK 高位通道，严格检查自身连杆、其他七台 PARK 机械臂、固定环境和输送线。
- 当前 R1→R2 累计工艺已通过：R2 `RAIL_PICK_H` 67帧、`RAIL_PLACE_H` 697帧；横轨在真实柜壳已位于WB1的条件下完成带载扫掠和接触段检查。累计四动作共1566帧。
- 当前 R1→R2→R3 WB1 累计工艺已通过：R3 A取/放为197/497帧，B取/放为153/675帧；A完成后真实托盘和已装柜体安全前移120 mm，随后B放置成功，累计8动作共3088帧。
- 当前 R1→R4 累计工艺已通过：`wb1_micro → wb2` 完整装配体输送无碰撞；R4 PSU取/放207/441帧、Servo取/放167/465帧、EDS取/放179/467帧，三器件按顺序在真实柜体内安装成功。
- 当前 R1→R5 累计工艺已通过：R5 PLC取/放165/401帧、DMA取/放189/439帧、Filter取/放225/467帧；六条路径全部使用直接竖直Π形走廊，并在保留全部前序器件的柜体内完成真空带载检查。
- 当前 R1→R6 累计工艺已通过：Filter 完成后托盘及完整装配体由 WB2 安全前移 120 mm；R6 Contactor取/放213/511帧、Breaker取/放211/487帧；随后装配体安全转入 STAGING，COM5取/放231/491帧。六条路径均完成真实带载和放置接触段检查，终止事件为 `COM5_DONE`。
- 2026-09-07：R7 四点锁紧和 R8 三个动作已加入正式 schema 34 计划，动作总数为33/33。R8 使用真空吸盘前向肘部支路；门板放置采用 `z=0.50 m` 的单个底座外缘高位绕点，共507帧，直线方案因门板与R8 Link2相交而被拒绝。
- 2026-09-07：R1→R8 完整顺序预检通过，包括真实工件附着/释放、其他七臂 PARK、两次120 mm工艺微移、WB1→WB2→STAGING→OUTPUT三段索引、R7四点锁紧、R8门板安装和闭锁；最终事件为 `OUTPUT_READY`，报告为 `full geometry and carried-workpiece sweep passed`。
- 2026-09-07：正式 schema 34 计划以 `--skip-preflight --speed 3.0` 完成一次确定性动画回放，依次产生全部工艺事件，最终输出 `cabinet assembly complete; simulation is paused at the output pallet stop`，退出码0。
- 2026-09-07：最终重新用 `bash scripts/start_coppelia_ubuntu.sh` 打开可见 GUI；Remote API 复核场景路径正确、停止态、875对象。R1–R8、中央输送线、托盘、三个公共区和R8真空工具均唯一；旧 `RobotBases`/`R1_Base`…`R8_Base`/桌台别名与所有 turntable/base-disc 类别名均为0。R6根位姿为 `[-0.50,-0.08,0.19]`，位于输送带侧边而非带面上。
- 视觉 STL 轴系校正后，33/33 APP/TCP 可达性再次全部通过；随后 R1→R8 完整携件几何预飞到达 `OUTPUT_READY`，报告 `full geometry and carried-workpiece sweep passed`。
- 正式与 partial 固定路径均为 schema 34、33/33，并绑定当前场景 size `8688315`、sha256 `cbe0c667adfae81dd716f12e71772358829ae7745672368d52c27162aa67fd55`。
- partial 现在包含14个零件的初始父级/矩阵，可直接用 `--preflight-wb1` 复位和审计，不再只保存关节路径。

## 验收边界 / 不得越界宣称

1. 数字样机的场景、任务、TCP、固定路径和自动几何验证已完成，当前没有已知的阻断项。
2. 用户仍可做展示性人工目视签收：释放前工件持续随 TCP、吸盘/夹爪接触、最终输出位构型。这不是当前自动运行的阻断项。
3. 当前仅为 CoppeliaSim 数字样机验证，不代表真实机械臂已连接或完成安全认证。

当前指纹信息：

- 正式与 schema 34 partial 路径记录：size `8688315`，sha256 `cbe0c667adfae81dd716f12e71772358829ae7745672368d52c27162aa67fd55`（当前可执行计划，33/33）
- 最新无圆台、八机械臂、R8 真空场景：size `8688315`，sha256 `cbe0c667adfae81dd716f12e71772358829ae7745672368d52c27162aa67fd55`

## 新对话应从这里继续

1. 当前 CoppeliaSim 已用 bareLua 模式打开 `/home/zhu/cr5_assembly_team/scenes/compact_cell.ttt`，ZMQ 端口 23000，仿真停止态；标准启动命令为 `bash scripts/start_coppelia_ubuntu.sh`。
2. 当前没有 `run_8arm_cabinet_assembly.py` 后台进程；只保留一个 CoppeliaSim 实例监听23000。
3. 33个动作的APP/TCP、固定路径和完整预检均已完成；不要再重算 R1–R8，除非场景、运动策略或任务点发生变化。
4. 当前正式路径可直接演示；不要手工修改路径中的场景/策略指纹。
5. 若后续改动导致碰撞，严格按“机器人/工具/携带工件/托盘/装配体”的对象对和帧号逐项修复，不要全局豁免柜体或输送线。
6. 下一步运行演示：

   ```bash
   python3 scripts/run_8arm_cabinet_assembly.py \
     --port 23000 --skip-preflight --speed 1.0
   ```

7. 演示后重点目视复核：夹爪/磁吸面是否真实接触工件、释放前工件是否随 TCP、托盘锁止时装配体是否保持不滑移、法兰是否持续朝下、各公共区待机姿态是否侵入输送通道。

## 关键文件

- 场景构建：`scripts/build_cabinet_product_scene.py`
- 规划、碰撞预检和执行：`scripts/run_8arm_cabinet_assembly.py`
- 固定路径：`data/fixed_paths/eight_arm_cabinet.json`
- 规划检查点：`data/fixed_paths/eight_arm_cabinet.partial.json`
- 当前场景：`scenes/compact_cell.ttt`
- 场景契约：`configs/scene_contract.yaml`
- 单元测试：`tests/test_motion_coordination.py`

## 工作区注意事项

当前工作区有未提交修改，至少包括：

- `configs/scene_contract.yaml`
- `data/fixed_paths/eight_arm_cabinet.json`
- `data/fixed_paths/eight_arm_cabinet.partial.json`
- `scenes/compact_cell.ttt`
- `scripts/build_cabinet_product_scene.py`
- `scripts/run_8arm_cabinet_assembly.py`

不要 reset、checkout 或覆盖这些修改。继续使用 `apply_patch` 修改源码，并在每次场景重建后检查是否产生同名/孤儿对象。
