#include "decision_node/battle_field_status.hpp"

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>


class CompareValue : public BT::ConditionNode
{
public:
  CompareValue(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("key", "ref.remain_hp", "Blackboard key (int) to read"),
      BT::InputPort<int>("threshold", 1000, "Threshold: SUCCESS when value < threshold"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string key = "ref.remain_hp";
    int threshold = 1000;
    (void)getInput("key", key);
    (void)getInput("threshold", threshold);

    int value = 0;
    try
    {
      value = config().blackboard->get<int>(key);
    }
    catch (const std::exception& e)
    {
      ROS_WARN_THROTTLE(2.0, "CompareValue: failed to read key '%s': %s",
                        key.c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }

    return (value < threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class CheckIntStatus : public BT::ConditionNode
{
public:
  CheckIntStatus(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("key", "ref.trapezoidal_elevated_ground_status",
                                 "Blackboard key (int) to read"),
      BT::InputPort<int>("expected", 1,
                         "Expected integer value; SUCCESS when equal"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string key = "ref.trapezoidal_elevated_ground_status";
    int expected = 1;
    (void)getInput("key", key);
    (void)getInput("expected", expected);

    int value = 0;
    try
    {
      value = config().blackboard->get<int>(key);
    }
    catch (const std::exception& e)
    {
      ROS_WARN_THROTTLE(2.0, "CheckIntStatus: failed to read key '%s': %s",
                        key.c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }

    return (value == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class CheckVision : public BT::ConditionNode
{
public:
  CheckVision(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("key", "vision.detected",
                                 "Blackboard key (bool) to read"),
      BT::InputPort<bool>("expected", true,
                         "Expected boolean value; SUCCESS when equal"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string key = "vision.detected";
    bool expected = true;
    (void)getInput("key", key);
    (void)getInput("expected", expected);

    bool value = false;
    try
    {
      value = config().blackboard->get<bool>(key);
    }
    catch (const std::exception& e)
    {
      ROS_WARN_THROTTLE(2.0, "CheckVision: failed to read bool key '%s': %s",
                        key.c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }

    return (value == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsBaseInDanger : public BT::ConditionNode
{
public:
  IsBaseInDanger(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("danger_hp", 100, "HP threshold: below which is considered in danger"),
      BT::InputPort<std::string>("hp_key", "ref.current_HP", "Blackboard key for current HP"),
    };
  }

  BT::NodeStatus tick() override
  {
    int danger_hp = 100;
    (void)getInput("danger_hp", danger_hp);

    std::string hp_key = "ref.current_HP";
    (void)getInput("hp_key", hp_key);

    int remain_hp = 0;
    try
    {
      remain_hp = config().blackboard->get<int>(hp_key);
    }
    catch (...)
    {
      ROS_WARN_THROTTLE(1.0, "IsSentryInDanger: failed to read hp from key '%s'", hp_key.c_str());
      return BT::NodeStatus::FAILURE;
    }

    const bool in_danger = (remain_hp > 0 && remain_hp < danger_hp);
    return in_danger ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class CountInfantryDeaths : public BT::SyncActionNode
{
public:
  CountInfantryDeaths(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    // 1. 读取两个步兵血量
    int hp_3 = 0, hp_4 = 0;
    try
    {
      hp_3 = bb->get<int>("ref.ally_3_robot_HP");
      hp_4 = bb->get<int>("ref.ally_4_robot_HP");
    }
    catch (...)
    {
      ROS_WARN_THROTTLE(2.0, "[CountInfantryDeaths] ally_3/4 HP not in blackboard");
      return BT::NodeStatus::FAILURE;
    }

    // 2. 实时统计血量为 0 的机器人数量
    int infantry_dead = 0;
    if (hp_3 == 0) infantry_dead++;
    if (hp_4 == 0) infantry_dead++;

    // 3. 写回黑板
    bb->set("ref.infantry_dead", infantry_dead);

    return BT::NodeStatus::SUCCESS;
  }
};

class CountInfantryDeathsDynamic : public BT::SyncActionNode
{
public:
  CountInfantryDeathsDynamic(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    // 1. 读取两个步兵血量
    int hp_3 = 0, hp_4 = 0;
    try
    {
      hp_3 = bb->get<int>("ref.ally_3_robot_HP");
      hp_4 = bb->get<int>("ref.ally_4_robot_HP");
    }
    catch (...)
    {
      ROS_WARN_THROTTLE(2.0, "[CountInfantryDeathsDynamic] ally_3/4 HP not in blackboard");
      return BT::NodeStatus::FAILURE;
    }

    // 2. 检测"刚死亡"事件：上一帧 alive → 当前帧 dead
    int new_deaths = 0;
    if (hp_3 == 0 && last_hp_3_ > 0) new_deaths++;
    if (hp_4 == 0 && last_hp_4_ > 0) new_deaths++;

    // 3. 写入当前帧的新死亡数（非历史累加，与 CountInfantryDeaths 共用同一黑板变量）
    bb->set("ref.infantry_dead", new_deaths);

    // 4. 记录当前血量供下一帧比较
    last_hp_3_ = hp_3;
    last_hp_4_ = hp_4;

    return BT::NodeStatus::SUCCESS;
  }

private:
  int last_hp_3_ = -1;  // -1 表示首次运行，不会误计
  int last_hp_4_ = -1;
};

class WaitForVision : public BT::StatefulActionNode
{
public:
  WaitForVision(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", 10.0, "Time window in seconds"),
      BT::InputPort<std::string>("key", "vision.detected", "Blackboard key (bool) to read"),
    };
  }

  BT::NodeStatus onStart() override
  {
    auto bb = config().blackboard;

    // ---- 读取参数 ----
    (void)getInput("duration", duration_);
    (void)getInput<std::string>("key", vision_key_);

    // ---- 记录当前 action ----
    try {
      tracked_action_ = bb->get<std::string>("action");
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }

    // ---- 只在首次初始化时记录 start_time，后续重新触发时保留 ----
    if (!timer_initialized_)
    {
      start_time_ = ros::Time::now();
      vision_success_time_ = ros::Time(0);
      timer_initialized_ = true;
    }

    ROS_DEBUG("[WaitForVision] Started, duration=%.1f, key=%s",
              duration_, vision_key_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto bb = config().blackboard;
    ros::Time now = ros::Time::now();

    // ---- 检测 action 是否变化（状态切换则结束本次等待） ----
    std::string cur_action;
    try {
      cur_action = bb->get<std::string>("action");
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }

    if (cur_action != tracked_action_)
    {
      // action 切换，说明实际不满 duration → SUCCESS
      ROS_DEBUG("[WaitForVision] Action changed from '%s' to '%s' before timer expired, SUCCESS",
                tracked_action_.c_str(), cur_action.c_str());
      timer_initialized_ = false;  // 重置，下次 onStart 重新计时
      return BT::NodeStatus::SUCCESS;
    }

    // ---- 读取 vision ----
    bool vision_value = false;
    try {
      vision_value = bb->get<bool>(vision_key_);
    } catch (...) {
      // vision key 还不存在，忽略
    }

    if (vision_value)
    {
      // vision.detected 为 true → 记录成功时间，进入 grace period
      vision_success_time_ = now;
      ROS_DEBUG("[WaitForVision] Vision detected, entering grace period");
    }

    // ---- 检查是否在 grace period 内（上次 vision 成功后 duration 秒内） ----
    double grace_elapsed = (now - vision_success_time_).toSec();
    if (vision_success_time_ != ros::Time(0) && grace_elapsed < duration_)
    {
      // 上次 vision 成功后未满 duration，仍在 grace period 内 → SUCCESS
      ROS_DEBUG("[WaitForVision] Vision grace period (%.1fs remaining), SUCCESS",
                duration_ - grace_elapsed);
      timer_initialized_ = false;  // 重置，下次 onStart 重新计时
      return BT::NodeStatus::SUCCESS;
    }

    // ---- 检查是否在初始 duration 窗口内（进入节点后 duration 秒内） ----
    double elapsed = (now - start_time_).toSec();
    if (elapsed < duration_)
    {
      // duration 窗口内但 vision 从未为 true，也没有 grace period
      // 返回 SUCCESS 让 Sequence 执行 SetAction，下一帧重新 onStart
      // 但由于 timer_initialized_=true，onStart 不会重置计时器
      ROS_DEBUG("[WaitForVision] Within initial window (%.1fs remaining), SUCCESS",
                duration_ - elapsed);
      return BT::NodeStatus::SUCCESS;
    }

    // ---- 初始窗口到期，且不满足任何 SUCCESS 条件 → FAILURE ----
    ROS_DEBUG("[WaitForVision] Timeout after %.2fs without vision, FAILURE", elapsed);
    timer_initialized_ = false;  // 重置，下次 onStart 重新计时
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    ROS_DEBUG("[WaitForVision] Halted");
    timer_initialized_ = false;  // 节点被 halt 时也重置
  }

private:
  std::string tracked_action_;
  ros::Time   start_time_;
  ros::Time   vision_success_time_;  // 最近一次 vision 成功的时间戳
  double      duration_    = 10.0;
  std::string vision_key_  = "vision.detected";
  bool        timer_initialized_ = false;
};
void RegisterBattleFieldNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<CompareValue>("CompareValue");
  factory.registerNodeType<CheckIntStatus>("CheckIntStatus");
  factory.registerNodeType<CheckVision>("CheckVision");
  factory.registerNodeType<CountInfantryDeaths>("CountInfantryDeaths");
  factory.registerNodeType<CountInfantryDeathsDynamic>("CountInfantryDeathsDynamic");
  factory.registerNodeType<WaitForVision>("WaitForVision");
}
