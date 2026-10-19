# DecisionNode

RoboMaster 哨兵决策模块，基于 [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) 实现。

## 1. 为什么使用 BehaviorTree.CPP

| 原因 | 说明 |
|------|------|
| **Groot 辅助调试** | [Groot2](https://github.com/BehaviorTree/Groot2) 可以直接打开 `strategy_tree.xml`，可视化整棵行为树和当前执行路径，改完策略可立即检查结构是否合理 |
| **解耦式策略编写** | 节点代码只描述“能力”（判断/动作），策略全部写在 XML 里。调策略时只改 XML，不碰 C++、不重新编译 |
| **比赛期间快速修改** | 战术临时调整只需编辑 XML 并重启节点；出问题直接换回 `backup_tree.xml`，回滚成本极低，避免现场改代码引入风险 |

## 2. 结构

### 树状结构

整棵决策逻辑是一棵**行为树**，优先级完全由 BehaviorTree.CPP 自带的层级结构约束：

```
MainLoop (ReactiveSequence)
├── UpdateBlackboard           # 每个 tick 先刷新黑板数据
└── DecideAction (ReactiveFallback)
    ├── 未开始   → INIT
    ├── 刚开局   → PUSH
    ├── 已死亡   → RESPAWN
    ├── 复活后   → SUPPLY
    ├── 血量危险 → SUPPLY
    ├── ...
    └── 兜底     → WAITFOROP
```

- **`ReactiveFallback`（选择/优先级）**：每个 tick 从左到右重新评估，**越靠前的分支优先级越高**，第一个返回 SUCCESS 的分支胜出。
- **`ReactiveSequence`（顺序 + 中断）**：依次执行，前面的条件失败则整条分支被打断，下一个 tick 重新从根节点选择。
- **`Sequence`**：普通顺序执行，适合“先设置、再发布”这类必须走完的流程。
- 因此**调整优先级 = 调整 XML 中分支的先后顺序**，无需改动任何 C++ 代码。

### 目录结构

```
DecisionNode/
├── src/decision_node/
│   ├── config/
│   │   ├── strategy_tree.xml      # 当前策略（行为树定义）
│   │   └── backup_tree.xml        # 备份策略，用于快速回滚
│   ├── launch/
│   │   ├── decision_with_communicator.launch   # 决策 + 通讯（比赛用）
│   │   ├── strategy_decision_tmp.launch        # 仅决策
│   │   └── mcu_communicator.launch             # 仅通讯
│   ├── src/
│   │   ├── strategy_node.cpp      # 行为树执行节点 + 自定义节点注册
│   │   ├── central_occupiable.cpp # 占位累计
│   │   ├── chase.cpp              # 追逐
│   │   ├── motion_change.cpp      # 运动状态
│   │   ├── recover_change.cpp     # 恢复/补弹
│   │   └── mcu_communicator.cpp   # 与下位机串口通讯
│   └── CMakeLists.txt
└── devel/lib/decision_node/       # 编译产物
```

### 数据流

```
裁判系统/雷达/导航 ──串口──> mcu_communicator ──ROS topic──> strategy_node（写入黑板）
                                                              │
                                                         行为树 tick
                                                              │
                    导航 / 下位机 <──ROS topic── strategy_node（目标点、运动标志）
```

## 3. 使用

### 编译

```bash
cd ~/AstarTraining/DecisionNode      # 工作空间根目录
catkin_make
source devel/setup.bash
```

### 启动

```bash
# 比赛：决策 + 通讯一起启动
roslaunch decision_node decision_with_communicator.launch

# 只跑决策（调试策略，输出到 /clicked_point）
roslaunch decision_node strategy_decision_tmp.launch

# 只跑通讯（指定串口和波特率）
roslaunch decision_node mcu_communicator.launch serial_port:=/dev/ttyUSB0 baudrate:=921600
```

### 可视化调试

用 Groot2 打开 [strategy_tree.xml](src/decision_node/config/strategy_tree.xml)，可直接查看树结构和节点参数。

### 修改策略

1. 在 Groot2 中编辑或直接修改 XML；
2. 用 `roslaunch` 重启 `strategy_node` 生效；
3. 现场验证不通过时，把 [backup_tree.xml](src/decision_node/config/backup_tree.xml) 替换回 `strategy_tree.xml` 即可回滚。

### 无实车测试

```bash
./devel/lib/decision_node/continuous_forwarder
```

手动发布裁判系统/导航数据，观察决策输出是否符合预期。

## 4. 内容

### 策略

策略文件为 [strategy_tree.xml](src/decision_node/config/strategy_tree.xml)，核心思路是**按优先级从高到低排布分支**：

| 优先级 | 场景 | 动作 |
|--------|------|------|
| 1 | 比赛未开始 | `INIT` |
| 2 | 比赛开始，当前为 INIT | `PUSH` |
| 3 | 哨兵死亡 | `RESPAWN` |
| 4 | 复活后 / 血量危险 / 补给未完成 | `SUPPLY` |
| 5 | PUSH 已到达 | `OCCUPY`（循环占领点） |
| 6 | 分数优势且弹量充足 | `RADICAL` → 到达后 `RADICAL1` |
| 7 | 占位有利 | `PUSH` |
| 8 | 兜底 | `WAITFOROP` |

节点分四类：

- **状态更新**：`UpdateRefereeBB`、`UpdateNavigationBB`、`UpdateVisionBB`、`UpdateTimersBB`、`UpdateDerivedFlags`
- **条件**：`IsGameStarted`、`IsAction`、`IsSentryDead`、`IsSentryInDanger`、`AggressiveAdvantage`、`IntenseHarm` 等
- **动作**：`SetAction`、`SetGoalFromParams(Cyclic)`、`PublishGoalPoint`、`PublishMotion`、`PublishRecover` 等
- **控制流**：`Sequence`、`ReactiveSequence`、`Fallback`、`ReactiveFallback`，以及 `RetryUntilSuccessful` 等 Decorator

节点之间的数据通过**黑板**传递，常用的有 `ref.remain_hp`、`ref.bullet_remain`、`ref.occupy_status`、`nav.arrived`、`goal.point`、`motion_flag` 等。

### 通讯

通讯由 `mcu_communicator` 负责（USART6，默认 921600 bps）：

- **上行**：解析下位机串口数据，以 `/referee/*`、`/robot/*`、`/enemy/*`、`/radar/*` 等 topic 发布，供决策节点订阅。
- **下行**：订阅 `/navigation`、`/cmd_vel`、`/dstar_status` 等，按固定频率打包后通过串口发给下位机。

决策节点对外输出以下 topic：

| Topic | 类型 | 含义 |
|-------|------|------|
| `/clicked_point` | `geometry_msgs/PointStamped` | 目标点 |
| `motion` | `std_msgs/UInt8` | 运动状态（0 停止 / 1 巡逻占领 / 2 躲避 / 3 推进） |
| `recover` | `std_msgs/UInt8` | 恢复标志 |
| `bullet_up` | `std_msgs/UInt8` | 补弹标志 |
| `bullet_num` | `std_msgs/UInt8` | 补弹数量 |
