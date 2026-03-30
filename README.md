# DecisionNode
The decision part for sentry using behaviour tree

## 快速开始

**手动发布数据进行测试**：
```bash
./run test.cpp:/mnt/d/decision_ws/devel/lib/decision_node/continuous_forwarder
```

**可视化行为树**：下载[Groot2](https://github.com/BehaviorTree/Groot2)，打开`config/strategy_tree.xml`文件即可自动识别和可视化

---

# 决策树节点完整说明

本项目包含 **42个自定义行为树节点**，分为7大类。详细信息见 [BehaviorTree_Nodes_Summary.md](BehaviorTree_Nodes_Summary.md)

## 1️⃣ 占位相关节点（Central Occupiable）

这组节点处理中央占领点的累计和触发逻辑。

| 节点 | 类型 | 功能 |
|------|------|------|
| **AccumulateCentralOccupiable** | Action | 累计中央占位的状态计数。当敌方未占领时累计，达到阈值后触发标记 |
| **TriggerOnThreshold** | Condition | 在达到阈值且未触发时返回SUCCESS，防止重复触发 |
| **ResetAccumulator** | Action | 重置累计计数器，将计数设为0 |
| **ResetCentralOccupiable** | Action | 完整重置占位数据，包括计数和触发标记 |

**使用示例**：
```xml
<AccumulateCentralOccupiable 
  occupy_status="{ref.occupy_status}"
  threshold="{occupy_threshold}"
  accumulated_count="{occupy_count}"
  reached_threshold="{occupy_reached}" />
<TriggerOnThreshold 
  reached_threshold="{occupy_reached}"
  reset_condition="{is_enemy_occupied}" />
```

---

## 2️⃣ 追逐相关节点（Chase）

用于实现追逐敌方机器人的功能。

| 节点 | 类型 | 功能 |
|------|------|------|
| **InitChase** | Action | 初始化追逐模式，读取建议目标ID并获取目标位置 |
| **UpdateChaseTarget** | Action | 每帧更新追逐目标的实时位置 |
| **PublishChaseGoal** | Action | 发布追逐目标位置到ROS话题 |
| **ResetChase** | Action | 重置追逐状态，清空目标信息 |

---

## 3️⃣ 运动控制节点（Motion Control）

控制机器人的运动状态和到达检测。

| 节点 | 类型 | 功能 | 参数 |
|------|------|------|------|
| **CheckArrived** | Condition | 检查导航是否到达目标点 | - |
| **CheckAttacked** | Condition | 检查是否正在受攻击（HP下降）| `attack_threshold` (默认5) |
| **SetMotionFlag** | Action | 设置运动状态标志（0-3）| `target_motion` |
| **PublishMotion** | Action | 发布motion_flag到ROS話題 | - |

**motion_flag 状态说明**：
- `0` - 停止
- `1` - 巡逻/占领
- `2` - 躲避（受攻击时）
- `3` - 推进/移动

**注意**：状态0-2之间切换需要5秒冷却时间，防止频繁切换

---

## 4️⃣ 恢复与补给节点（Recover & Supply）

管理血量恢复和弹药补放。

| 节点 | 类型 | 功能 | 参数 |
|------|------|------|------|
| **IsHealthFull** | Condition | 检查HP是否满血 | `max_hp` (默认400) |
| **SetRecover** | Action | 设置恢复标志（0/1） | `value` |
| **IsBulletFull** | Condition | 检查弹量是否满弹 | `max_bullet` (默认999) |
| **SetBulletUp** | Action | 设置补弹标志（0/1） | `value` |
| **PublishRecover** | Action | 发布恢复标志 | - |
| **PublishBulletUp** | Action | 发布补弹标志 | - |
| **SetBulletNum** | Action | 计算补弹数量 | `mode` (DELTA/FIXED)、`expected_bullet`、`fixed_supply` |
| **PublishBulletNum** | Action | 发布补弹数量 | - |

**SetBulletNum 两种模式**：
- `DELTA`：补弹到指定最大值（弹数 = max_bullet - 当前弹数）
- `FIXED`：固定补放数量

---

## 5️⃣ 状态更新节点（State Update）

持续更新黑板中的各类信息。

| 节点 | 类型 | 数据来源 | 功能 |
|------|------|--------|------|
| **UpdateRefereeBB** | Action | 裁判系统 | 游戏进度、HP、弹量、分数、占位状态、敌方位置 |
| **UpdateNavigationBB** | Action | 导航模块 | 到达状态 |
| **UpdateVisionBB** | Action | 视觉模块 | 占位符（待集成） |
| **UpdateTimersBB** | Action | 计时器 | 占位符（待迁移） |
| **UpdateDerivedFlags** | Action | 计算派生 | 危险状态、弹量充足、伤害历史 |

**UpdateDerivedFlags 参数**：
- `danger_hp` (默认200)：低于此值为危险
- `sufficient_bullet` (默认20)：高于此值为充足

---

## 6️⃣ 条件判断节点（Condition）

各类条件检查，返回SUCCESS或FAILURE。

| 节点 | 功能 | 参数 |
|------|------|------|
| **IsGameStarted** | 检查游戏是否开始 | `expect_started` (true=已开始, false=未开始) |
| **IsAction** | 检查当前动作是否为目标值 | `value` (如"PUSH"、"OCCUPY") |
| **IsSentryDead** | 检查机器人是否已死亡 | - |
| **IsSentryAlive** | 检查机器人是否还活着 | - |
| **IsSentryInDanger** | 检查HP是否危险（低血量） | - |
| **NotBulletSufficient** | 检查弹量是否不足 | - |
| **AggressiveAdvantage** | 检查分数优势（我方 - 敌方 ≥ 阈值） | `threshold` (默认50) |
| **IntenseHarm** | 检查是否受到激烈伤害 | `threshold_activate`、`threshold_deactivate` |

---

## 7️⃣ 行动执行节点（Action Execution）

执行具体的决策和导航动作。

| 节点 | 功能 | 参数 |
|------|------|------|
| **SetAction** | 设置当前动作 | `action` (INIT、PUSH、OCCUPY、SUPPLY等) |
| **ClearGoal** | 清空目标点 | - |
| **Wait** | 等待指定时间 | `duration` (秒数) |
| **SetGoalFromParams** | 从参数服务器读取单个目标点 | `ns` (命名空间) |
| **SetGoalFromParamsCyclic** | 循环读取多个目标点 | `ns`、`point_count` (默认4) |
| **AdvanceCycleIndex** | 推进循环索引 | `point_count` |
| **PublishGoalPoint** | 发布目标点到ROS topic | `topic` (默认"clicked_point") |

**SetGoalFromParams 参数示例**：
```xml
<!-- 从参数 /goals/occupy/point_2/{x,y} 读取第2个占领点 -->
<SetGoalFromParamsCyclic ns="occupy" point_count="4" />
```

---

## 黑板（Blackboard）关键字速查

| 类别 | 关键字 | 说明 |
|------|--------|------|
| **Referee** | `ref.remain_hp`、`ref.bullet_remain` | 自身HP、弹量 |
| **Referee** | `ref.occupy_status` | 占位状态 |
| **Referee** | `ref.friendly_score`、`ref.enemy_score` | 双方分数 |
| **Referee** | `ref.enemy_*_x/y` | 敌方位置 |
| **Navigation** | `nav.arrived` | 是否到达 |
| **Goal** | `goal.point`、`goal.valid` | 目标点和有效性 |
| **Motion** | `motion_flag` | 运动状态 |
| **Recovery** | `recover`、`bullet_up` | 恢复和补弹标志 |
| **Accumulate** | `occupy_count`、`occupy_reached` | 累计占位计数 |

---

## 🔗 相关资源

- [详细节点文档](BehaviorTree_Nodes_Summary.md)
- [Strategy Tree XML](config/strategy_tree.xml) - 行为树定义
- [Groot2](https://github.com/BehaviorTree/Groot2) - 行为树可视化工具