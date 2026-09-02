# CR5 柔性装配单元 — 团队使用说明

## 环境要求

| 软件 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble |
| CoppeliaSim | Edu V4.10.0（**必须从这个版本启动**） |

## 一、获取场景

```bash
git clone https://github.com/reshuidawang-spec/cr5_assembly_team.git ~/cr5_assembly_team
cd ~/cr5_assembly_team
```

## 二、启动 CoppeliaSim

**必须从 source 过 ROS2 的终端启动**，否则 ROS2 通信不工作：

```bash
source /opt/ros/humble/setup.bash
cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
./coppeliaSim.sh
```

## 三、打开场景

**File → Open scene...** → 选择 `~/cr5_assembly_team/scenes/compact_cell.ttt`

▶️ 点击播放按钮启动仿真。

## 四、场景概览

```
FiveCR5A_Cell
├── R1: 箱体上料 + 端子排安装      (宽口可调夹爪 R1T)
├── R2: PCB 吸取安装               (吸盘 R2T)
├── R3: 控制模块安装 + 搬运到检测区 (宽口夹爪 R3T)
├── R4: 端子排螺钉锁付             (电动螺丝刀 R4T)
├── R5: 合格/缺陷品分拣到传送带    (宽口夹爪 R5T)
├── CartA / CartB: AGV 物料小车
├── Fixed_Vision_Camera: 视觉检测 OK/NG
├── Good_Conveyor: 良品传送带
├── Defect_Conveyor: 缺陷品传送带
├── Parts: A 型号产品
└── PartsB: B 型号产品
```

### 装配流程

```
CartA供料 → R1放箱体 → R2装PCB → R3装模块 → R1装端子排
→ R3搬运到检测区 → 相机检测 → R4锁付 → R5分拣到传送带
```

## 五、ROS2 命令

### 5.1 场景控制 `/compact_cell/cmd`

```bash
# 重置场景
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'RESET_CELL'"

# 切换 A 型号（默认）
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'PRODUCT_A'"

# 切换 B 型号
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'PRODUCT_B'"

# 颜色循环
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'COLOR_NEXT'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'COLOR_1'"

# 小车调度
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'CART_A_SUPPLY'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'CART_B_SUPPLY'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'CART_RESET'"

# 工艺步骤（逐阶段推进）
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'SHOW_ASSEMBLY_SHELL'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'SHOW_ASSEMBLY_PCB'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'SHOW_ASSEMBLY_MODULE'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'SHOW_ASSEMBLY_FULL'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'SHOW_INSPECTION_FULL'"

# 传送带
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'CONVEYOR_GOOD'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'CONVEYOR_DEFECT'"
```

### 5.2 工具动作 `/compact_cell/tool_cmd`

```bash
# R1: 宽口可调夹爪（箱体 + 端子排）
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R1_GRIPPER_OPEN'"
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R1_GRIPPER_CLOSE'"
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R1_ATTACH_BOX'"
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R1_ATTACH_TERMINAL'"

# R2: 吸盘
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R2_VACUUM_ON'"
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R2_ATTACH_PCB'"

# R3: 宽口夹爪（模块 + 搬运）
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R3_GRIPPER_OPEN'"
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R3_ATTACH_ASSEMBLY_PRODUCT'"

# R4: 电动螺丝刀
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R4_SCREW_START'"

# R5: 分拣夹爪
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R5_GRIPPER_OPEN'"
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R5_ATTACH_INSPECTION_PRODUCT'"
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R5_RELEASE_GOOD'"
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R5_RELEASE_DEFECT'"
```

### 5.3 关节控制 `/compact_cell/joint_cmd`

```bash
# 单关节点动（度）
ros2 topic pub --once /compact_cell/joint_cmd std_msgs/msg/String "data: 'R1 J1 +10'"
ros2 topic pub --once /compact_cell/joint_cmd std_msgs/msg/String "data: 'R1 J1 -10'"

# 回零
ros2 topic pub --once /compact_cell/joint_cmd std_msgs/msg/String "data: 'ALL HOME'"

# 设定关节角度
ros2 topic pub --once /compact_cell/joint_cmd std_msgs/msg/String "data: 'R1 SET 0 -30 0 -60 0 90 0'"
```

### 5.4 专用 Topic

```bash
# 每个机械臂独立命令
ros2 topic pub --once /compact_cell/r1_cmd std_msgs/msg/String "data: 'R1_READY'"

# 状态反馈
ros2 topic echo /compact_cell/status
```

## 六、完整工艺演示流程

```bash
# 1. 重置场景，切换 B 型号
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'RESET_CELL'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'PRODUCT_B'"

# 2. 调度 CartB 去供料位（B 型号用 CartB）
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'CART_B_SUPPLY'"

# 3. 逐步装配
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'R1_BOX_PLACED'"       # R1放箱体
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'R2_PCB_PLACED'"       # R2装PCB
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'R3_MODULE_PLACED'"    # R3装模块
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'R1_TERMINAL_PLACED'"  # R1装端子排
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'R3_PRODUCT_TO_INSPECTION'" # R3→检测

# 4. 检测 + 分拣
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'CAMERA_GOOD'"    # 合格
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R4_SCREW_START'" # 锁付
ros2 topic pub --once /compact_cell/tool_cmd std_msgs/msg/String "data: 'R5_RELEASE_GOOD'" # 放到良品传送带
```

## 七、开发和调试

### 查看场景对象
```bash
# 打开场景后查看目标点位置
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'RESET_CELL'"
```

### 手动微调目标点
在 CoppeliaSim 中点击 Dummy → 拖动 XYZ 箭头。目标点位于：
`/FiveCR5A_Cell/Targets/R1_Targets/` ~ `R5_Targets/`

### Agent 小车坐标
| 名称 | 坐标 | 用途 |
|------|------|------|
| CartA_WaitPose | (-2.3, -1.55, 0.05) | A 车等待位 |
| CartA_SupplyPose | (-2.3, -0.9, 0.05) | A 车供料位 |
| CartB_WaitPose | (-1.8, -1.55, 0.05) | B 车等待位 |
| CartB_SupplyPose | (-1.8, -0.9, 0.05) | B 车供料位 |

### 传送带坐标
| 名称 | 坐标 |
|------|------|
| Good_Conveyor | (0.48, -1.68, 0.18) |
| Defect_Conveyor | (-0.75, -1.12, 0.18) |

### R5 分拣点位
| 名称 | 坐标 |
|------|------|
| R5_GOOD_PLACE | (0.35, -1.10, 0.252) |
| R5_DEFECT_PLACE | (-0.15, -1.12, 0.252) |

## 八、常见问题

**Q: `ros2 topic list` 看不到 `/compact_cell/*`？**
必须从 `source /opt/ros/humble/setup.bash` 的终端启动 CoppeliaSim，且仿真必须正在运行（▶️）。

**Q: 打开场景后机械臂不见了？**
场景 23MB，URDF 模型已嵌入。检查 CoppeliaSim 版本必须是 V4.10。

**Q: CartA/CartB 不移动？**
确认小车目标点存在（`/CartA_SupplyPose` 等），且 Cart_Order_Controller 脚本已启用。

**Q: B 产品不显示？**
确认已运行过 Product_B_Create 或 PartsB 已存在于场景中。

**Q: 如何更新场景？**
```bash
cd ~/cr5_assembly_team
git pull origin main
```
