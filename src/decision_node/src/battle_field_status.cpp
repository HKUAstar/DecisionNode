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

// =====================================================
// RegisterBattleFieldNodes: 统一注册入口
// =====================================================
void RegisterBattleFieldNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<CompareValue>("CompareValue");
  factory.registerNodeType<CheckIntStatus>("CheckIntStatus");
  factory.registerNodeType<CheckVision>("CheckVision");
}
