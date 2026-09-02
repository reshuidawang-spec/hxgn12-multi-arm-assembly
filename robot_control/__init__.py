# robot_control/ —— 机械臂控制模块（3号同学）
# 实现 interfaces/robot_interface.py 中的 IRobotExecutor
#
# 2026-08 产线重构：旧五臂运动栈（r1-r5_motion、coordinated_engine、
# runtime_cartesian、plans 等）已随旧工艺整体移除。新 8 臂（R1-R8）
# 运动逻辑将在新产线流程定义完成后基于新场景重新实现。
