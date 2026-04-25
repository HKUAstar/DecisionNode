#include "decision_node/battle_field.hpp"

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

// =====================================================
// IsLaunchRampElevated: 检查发射坡道是否已升起
// 读取黑板 "ref.launch_ramp_elevated_ground" (int)
// == 1 → 坡道已升起 → SUCCESS
// != 1 → 坡道未升起 → FAILURE
// =====================================================
class IsLaunchRampElevated : public BT::ConditionNode
{
public:
  IsLaunchRampElevated(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("key", "ref.launch_ramp_elevated_ground",
                                 "Blackboard key for launch ramp status"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string key = "ref.launch_ramp_elevated_ground";
    (void)getInput("key", key);

    int status = 0;
    try
    {
      status = config().blackboard->get<int>(key);
    }
    catch (const std::exception& e)
    {
      ROS_WARN_THROTTLE(2.0, "IsLaunchRampElevated: failed to read key '%s': %s",
                        key.c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }

    return (status == 1) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// =====================================================
// RegisterBattleFieldNodes: 统一注册入口
// =====================================================
void RegisterBattleFieldNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<IsLaunchRampElevated>("IsLaunchRampElevated");
}
