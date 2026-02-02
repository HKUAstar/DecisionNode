# 决策参数集中管理设计文档

## 概述

将所有决策相关的可配置参数集中在 `strategy_node.cpp` 中定义和初始化，通过结构体统一管理，并存储到黑板供行为树节点使用。这样做的优点是：

- ✅ **单一源**：所有参数定义在一个地方
- ✅ **易维护**：修改参数值不用改多个文件
- ✅ **避免冗余**：`max_bullet` 和 `expected_bullet` 统一使用
- ✅ **支持动态配置**：可通过 ROS 参数服务器或 launch 文件传入
- ✅ **日志记录**：启动时输出所有参数值

## 参数定义

### 参数结构体（在 strategy_node.cpp 中）

```cpp
struct DecisionParams {
    int danger_hp = 100;              // 血量危险阈值（低于此值进入危险状态）
    int sufficient_bullet = 10;       // 弹量充足阈值（低于此值弹量不足）
    int max_bullet = 150;             // 最大弹量（用于补弹的期望值和满弹判定）
    int fixed_supply = 50;            // 固定补弹量（FIXED 模式下使用）
    int occupy_threshold = 30;        // 占领阈值（连续占领计数器）
    int aggressive_threshold = 50;    // 攻击阈值（分差超过此值视为优势）
};
```

### 参数加载流程

```
ROS 参数服务器或 launch 文件
    ↓
pnh.param() 读取（如果存在，否则使用默认值）
    ↓
DecisionParams 结构体存储
    ↓
blackboard->set() 写入黑板
    ↓
行为树节点通过 getInput() 从黑板读取
```

## 使用方式

### 1. 代码中使用

在 `strategy_node.cpp` 中自动加载和初始化：

```cpp
struct DecisionParams {
  int danger_hp = 100;
  // ... 其他参数
} params;

// 从 ROS 参数服务器读取
pnh.param("danger_hp", params.danger_hp, params.danger_hp);
// ... 其他参数

// 写入黑板供行为树使用
blackboard->set("danger_hp", params.danger_hp);
// ... 其他参数
```

### 2. 行为树中使用

在 `strategy_tree.xml` 中通过 `{variable_name}` 形式引用：

```xml
<!-- 条件节点中使用 -->
<UpdateDerivedFlags danger_hp="{danger_hp}" sufficient_bullet="{sufficient_bullet}" />

<!-- 补弹节点中使用 -->
<SetBulletNum mode="DELTA" expected_bullet="{max_bullet}" fixed_supply="{fixed_supply}" />

<!-- 判断弹量满的条件 -->
<IsBulletFull max_bullet="{max_bullet}" />
```

### 3. 通过 launch 文件传入参数

```xml
<launch>
  <node name="strategy_node" pkg="decision_node" type="strategy_node" output="screen">
    <!-- 覆盖默认参数 -->
    <param name="danger_hp" value="80" />
    <param name="sufficient_bullet" value="20" />
    <param name="max_bullet" value="200" />
    <param name="fixed_supply" value="100" />
    <param name="occupy_threshold" value="40" />
    <param name="aggressive_threshold" value="60" />
  </node>
</launch>
```

### 4. 通过命令行传入参数

```bash
rosrun decision_node strategy_node \
  _danger_hp:=80 \
  _max_bullet:=200 \
  _fixed_supply:=100
```

### 5. 通过 ROS 参数服务器动态查看

```bash
# 查看参数
rosparam get /decision_node/danger_hp
rosparam get /decision_node/max_bullet
rosparam list | grep decision_node

# 修改参数（需要重启 strategy_node 节点生效）
rosparam set /decision_node/max_bullet 200
```

## 参数详解

| 参数 | 默认值 | 说明 | 使用场景 |
|------|--------|------|---------|
| `danger_hp` | 100 | 血量危险阈值 | 当 `remain_hp < danger_hp` 时触发补血 |
| `sufficient_bullet` | 10 | 弹量充足阈值 | 当 `bullet_remain < sufficient_bullet` 时弹量不足 |
| **`max_bullet`** | **150** | **最大/期望弹量** | **DELTA 补弹时的目标值；判断弹量是否满的基准** |
| **`fixed_supply`** | **50** | **固定补弹量** | **FIXED 补弹模式下的补充数量** |
| `occupy_threshold` | 30 | 占领累计阈值 | 中心占领点连续占领计数达到此值触发 |
| `aggressive_threshold` | 50 | 分差优势阈值 | `friendly_score - enemy_score >= threshold` 时为优势 |

## 新增参数说明

### max_bullet（最大弹量）

**改进点**：原先 XML 中有 `max_bullet` 和 `expected_bullet` 两个参数表示同一个概念，现已统一。

**使用**：
- 在 `IsBulletFull` 中判断弹量是否充满：`<IsBulletFull max_bullet="{max_bullet}" />`
- 在 `SetBulletNum (DELTA模式)` 中作为期望值：`<SetBulletNum mode="DELTA" expected_bullet="{max_bullet}" />`

**改 max_bullet 时**：只需在 launch 文件或命令行传入一个值，两处都会自动更新。

### fixed_supply（固定补弹量）

**新增**：原先硬编码在 XML 中（固定为 50），现改为可配置参数。

**使用**：
- 在 `SetBulletNum (FIXED模式)` 中使用：`<SetBulletNum mode="FIXED" ... fixed_supply="{fixed_supply}" />`

**改 fixed_supply 时**：不需要重新编译，只需重启节点即可。

## 配置示例

### 示例 1：默认配置
```xml
<!-- 不指定参数，使用 strategy_node.cpp 中的默认值 -->
<node name="strategy_node" pkg="decision_node" type="strategy_node" />
```

结果：
- `max_bullet = 150`
- `fixed_supply = 50`

### 示例 2：自定义配置（高血量）
```xml
<node name="strategy_node" pkg="decision_node" type="strategy_node" output="screen">
  <param name="danger_hp" value="50" />
  <param name="max_bullet" value="200" />
  <param name="fixed_supply" value="150" />
</node>
```

结果：
- 血量 < 50 才触发补血
- 补弹目标 200 发
- 固定补弹每次 150 发

### 示例 3：DELTA 模式（自适应）
在 strategy_tree.xml 中：
```xml
<SetBulletNum mode="DELTA" expected_bullet="{max_bullet}" fixed_supply="{fixed_supply}" />
```

当前弹量 → 补充数量：
- 100 → 补 100（期望 200 - 当前 100）
- 180 → 补 20（期望 200 - 当前 180）
- 200 → 补 0（期望 200 - 当前 200）

### 示例 4：FIXED 模式（固定）
在 strategy_tree.xml 中：
```xml
<SetBulletNum mode="FIXED" expected_bullet="{max_bullet}" fixed_supply="{fixed_supply}" />
```

无论当前弹量，始终补 50 发（取决于 `fixed_supply` 值）。

## 修改历史

| 日期 | 改动 | 文件 |
|------|------|------|
| 2026-02-02 | 初始实现补弹功能 | recover_change.cpp, strategy_tree.xml |
| 2026-02-02 | 参数集中管理 | strategy_node.cpp, strategy_tree.xml |
| 2026-02-02 | 合并 max_bullet 和 expected_bullet | strategy_tree.xml |
| 2026-02-02 | 新增 fixed_supply 可配置参数 | strategy_node.cpp, strategy_tree.xml |

## 相关文件

- [strategy_node.cpp](src/decision_node/src/strategy_node.cpp) - 参数定义和初始化
- [strategy_tree.xml](config/strategy_tree.xml) - 行为树配置（使用黑板变量）
- [recover_change.cpp](src/decision_node/src/recover_change.cpp) - SetBulletNum 节点实现
- [BULLET_SUPPLY_README.md](src/decision_node/BULLET_SUPPLY_README.md) - 补弹功能详细说明

## 扩展建议

如果未来需要更多参数，可以继续添加到 `DecisionParams` 结构体中：

```cpp
struct DecisionParams {
    // 原有参数...
    
    // 新增参数
    int some_new_param = 100;
    // ...
};
```

然后在黑板初始化中添加：

```cpp
pnh.param("some_new_param", params.some_new_param, params.some_new_param);
blackboard->set("some_new_param", params.some_new_param);
```

最后在 XML 中使用 `{some_new_param}` 即可。

