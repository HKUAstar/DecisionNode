# DecisionNode 行为树节点快速参考

共 43 个自定义节点。每个节点标注 XML 写法和 Launch 参数。

---

## 1. 占位相关节点

### 1.1 AccumulateCentralOccupiable
**XML**: `<AccumulateCentralOccupiable occupy_status="{ref.occupy_status}" threshold="{occupy_threshold}" accumulated_count="{occupy_count}" reached_threshold="{occupy_reached}" />`
**Launch**: `<arg name="occupy_threshold" default="50"/><param name="occupy_threshold" value="$(arg occupy_threshold)"/>`

### 1.2 TriggerOnThreshold
**XML**: `<TriggerOnThreshold reached_threshold="{occupy_reached}" reset_condition="{is_enemy_occupied}" />`
**Launch**: 无参数

### 1.3 ResetAccumulator
**XML**: `<ResetAccumulator />`
**Launch**: 无参数

### 1.4 ResetCentralOccupiable
**XML**: `<ResetCentralOccupiable />`
**Launch**: 无参数

### 1.5 IsOccupyStatusFavorable ⭐ NEW
**XML**: `<IsOccupyStatusFavorable />` (occupy_status: 0/1/3 返回SUCCESS，2返回FAILURE)
**Launch**: 无参数

---

## 2. 追逐相关节点

### 2.1 InitChase
**XML**: `<InitChase />`
**Launch**: 无参数

### 2.2 UpdateChaseTarget
**XML**: `<UpdateChaseTarget />`
**Launch**: 无参数

### 2.3 PublishChaseGoal
**XML**: `<PublishChaseGoal />`
**Launch**: `<param name="publish_on_change_only" value="true"/>`

### 2.4 ResetChase
**XML**: `<ResetChase />`
**Launch**: 无参数

---

## 3. 运动控制节点

### 3.1 CheckArrived
**XML**: `<CheckArrived />`
**Launch**: 无参数

### 3.2 CheckAttacked
**XML**: `<CheckAttacked attack_threshold="{attack_threshold}" />`
**Launch**: `<arg name="attack_threshold" default="30"/><param name="attack_threshold" value="$(arg attack_threshold)"/>`

### 3.3 SetMotionFlag
**XML**: `<SetMotionFlag target_motion="3" />` (0=停止, 1=巡逻, 2=躲避, 3=推进)
**Launch**: 无参数(状态值0-2间需5秒冷却)

### 3.4 PublishMotion
**XML**: `<PublishMotion />`
**Launch**: `<param name="publish_on_change_only" value="true"/>`

---

## 4. 恢复与补给节点

### 4.1 IsHealthFull
**XML**: `<IsHealthFull max_hp="{max_hp}" />`
**Launch**: 无参数

### 4.2 SetRecover
**XML**: `<SetRecover value="1" />`
**Launch**: 无参数

### 4.3 IsBulletFull
**XML**: `<IsBulletFull max_bullet="{max_bullet}" />`
**Launch**: 无参数

### 4.4 SetBulletUp
**XML**: `<SetBulletUp value="1" />`
**Launch**: 无参数

### 4.5 PublishRecover
**XML**: `<PublishRecover />`
**Launch**: `<param name="publish_on_change_only" value="true"/>`

### 4.6 PublishBulletUp
**XML**: `<PublishBulletUp />`
**Launch**: `<param name="publish_on_change_only" value="true"/>`

### 4.7 SetBulletNum
**XML**: `<SetBulletNum mode="DELTA" expected_bullet="{max_bullet}" fixed_supply="{fixed_supply}" />`
**Launch**: `<arg name="max_bullet" default="150"/><arg name="fixed_supply" default="50"/>`

### 4.8 PublishBulletNum
**XML**: `<PublishBulletNum />`
**Launch**: `<param name="publish_on_change_only" value="true"/>`

---

## 5. 状态更新节点

### 5.1 UpdateRefereeBB
**XML**: `<UpdateRefereeBB />`
**Launch**: 无参数 (自动订阅裁判系统话题)

### 5.2 UpdateNavigationBB
**XML**: `<UpdateNavigationBB />`
**Launch**: 无参数

### 5.3 UpdateVisionBB
**XML**: `<UpdateVisionBB />`
**Launch**: 无参数 (占位符)

### 5.4 UpdateTimersBB
**XML**: `<UpdateTimersBB />`
**Launch**: 无参数 (占位符)

### 5.5 UpdateDerivedFlags
**XML**: `<UpdateDerivedFlags danger_hp="{danger_hp}" sufficient_bullet="{sufficient_bullet}" />`
**Launch**: `<arg name="danger_hp" default="200"/><arg name="sufficient_bullet" default="20"/>`

### 5.6 IntenseHarm
**XML**: `<IntenseHarm threshold_activate="{harm_threshold_on}" threshold_deactivate="{harm_threshold_off}" />`
**Launch**: `<arg name="harm_threshold_on" default="50"/><arg name="harm_threshold_off" default="10"/>`

---

## 6. 条件判断节点

### 6.1 IsGameStarted
**XML**: `<IsGameStarted expect_started="true" />` (true=已开始, false=未开始)
**Launch**: 无参数

### 6.2 IsSentryDead
**XML**: `<IsSentryDead />`
**Launch**: 无参数

### 6.3 IsSentryAlive
**XML**: `<IsSentryAlive />`
**Launch**: 无参数

### 6.4 IsSentryInDanger
**XML**: `<IsSentryInDanger />`
**Launch**: 无参数 (使用danger_hp阈值)

### 6.5 NotBulletSufficient
**XML**: `<NotBulletSufficient />`
**Launch**: 无参数 (使用sufficient_bullet阈值)

### 6.6 AggressiveAdvantage
**XML**: `<AggressiveAdvantage threshold="50" />`
**Launch**: `<arg name="aggressive_threshold" default="50"/>`

### 6.7 IsAction
**XML**: `<IsAction value="PUSH" />` (支持: INIT, INITPUSH, PUSH, OCCUPY, SUPPLY, RESPAWN, RADICAL, WAITFOROP)
**Launch**: 无参数

---

## 7. 行动执行节点

### 7.1 SetAction
**XML**: `<SetAction action="PUSH" />`
**Launch**: 无参数

### 7.2 ClearGoal
**XML**: `<ClearGoal />`
**Launch**: 无参数

### 7.3 Wait
**XML**: `<Wait duration="2.0" />`
**Launch**: 无参数

### 7.4 SetGoalFromParams
**XML**: `<SetGoalFromParams ns="supply" />`
**Launch**: `<param name="/goals/supply/x" value="6.0"/><param name="/goals/supply/y" value="6.0"/>`

### 7.5 SetGoalFromParamsCyclic
**XML**: `<SetGoalFromParamsCyclic ns="occupy" point_count="4" />`
**Launch**: 
```xml
<param name="/goals/occupy/point_0/x" value="1.0"/>
<param name="/goals/occupy/point_0/y" value="1.0"/>
<!-- point_1, point_2, point_3 类似 -->
```

### 7.6 AdvanceCycleIndex
**XML**: `<AdvanceCycleIndex point_count="4" />`
**Launch**: 无参数

### 7.7 PublishGoalPoint
**XML**: `<PublishGoalPoint topic="clicked_point" />`
**Launch**: 无参数

---

## 快速对照表

| 节点 | 黑板读取 | 黑板写入 | 参数 |
|------|---------|---------|------|
| AccumulateCentralOccupiable | occupy_status | occupy_count | threshold |
| CheckAttacked | remain_hp | - | attack_threshold |
| SetMotionFlag | motion_flag | motion_flag | target_motion |
| IsHealthFull | remain_hp | - | max_hp |
| SetBulletNum | bullet_remain | bullet_num | mode, expected_bullet |
| UpdateDerivedFlags | remain_hp, bullet_remain | is_dead, is_in_danger | danger_hp, sufficient_bullet |
| AggressiveAdvantage | friendly_score, enemy_score | - | threshold |
| IsAction | action | - | value |
| SetGoalFromParamsCyclic | goal.cycle_index | goal.point, goal.valid | ns, point_count |
