# 区域级 CBS 与 R7/R8 协调快速验证

> 仅验证公共区时间窗冲突消解，不替代连续几何碰撞验证。

| 用例 | 是否求解 | 展开节点 | 附加等待 |
|---|---:|---:|---:|
| opposite_pass | True | 2 | 10.0 |
| crossing_two_zones | True | 22 | 14.0 |
| urgent_yield | True | 2 | 11.0 |
| three_arm_chain | True | 2 | 8.0 |

## R7/R8 三策略

| 策略 | 剩余冲突 | 附加等待 | 急单完成时刻 |
|---|---:|---:|---:|
| uncoordinated | 1 | 0.0 | 11.0 |
| fixed_normal_first_mutex | 0 | 12.0 | 23.0 |
| priority_cbs | 0 | 11.0 | 11.0 |

四类用例均在100个节点预算内得到无时间窗冲突解；优先级CBS让普通任务让行，避免固定普通任务优先造成的急单等待。
