#include "decision_node/battle_field_status.hpp"

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

// =====================================================
// CheckIntStatus: 通用整型黑板条件节点
//
// XML 端口:
//   key      — 要读取的黑板 key（默认: ref.launch_ramp_elevated_ground）
//   expected — 期望值，相等则 SUCCESS，否则 FAILURE（默认: 1）
//
// 使用示例:
//   <CheckIntStatus key="ref.launch_ramp_elevated_ground" expected="1" />
//   <CheckIntStatus key="ref.game_progress" expected="4" />
// =====================================================
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
      BT::InputPort<std::string>("key", "ref.launch_ramp_elevated_ground",
                                 "Blackboard key (int) to read"),
      BT::InputPort<int>("expected", 1,
                         "Expected integer value; SUCCESS when equal"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string key = "ref.launch_ramp_elevated_ground";
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

// =====================================================
// RegisterBattleFieldNodes: 统一注册入口
// =====================================================
void RegisterBattleFieldNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<CheckIntStatus>("CheckIntStatus");
}
