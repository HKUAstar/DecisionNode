#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include "decision_node/motion_change.hpp"

// Helper function to convert string to uppercase
static std::string toUpper(std::string s)
{
  for (auto& c : s)
  {
    c = static_cast<char>(::toupper(c));
  }
  return s;
}

class CheckArrived : public BT::ConditionNode
{
public:
  CheckArrived(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    bool arrived = bb->get<bool>("nav.arrived");
    // ROS_INFO("CheckArrived: arrived=%d", arrived);
    return arrived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// CheckAttacked: Checks if the robot is currently under attack
class CheckAttacked : public BT::ConditionNode
{
public:
  CheckAttacked(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    
    // Check if motion_flag is 1 (under attack state)
    try
    {
      int motion_flag = bb->get<int>("motion_flag");
      bool is_under_attack = (motion_flag == 1);
      // ROS_DEBUG("CheckAttacked: motion_flag=%d, under_attack=%d", motion_flag, is_under_attack);
      return is_under_attack ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    catch (...)
    {
      return BT::NodeStatus::FAILURE;
    }
  }
};

// SetMotionFlag: Universal motion flag setter with 5-second cooldown check
// Input port "target_motion" specifies the target motion_flag value (0, 1, or 2)
class SetMotionFlag : public BT::SyncActionNode
{
public:
  explicit SetMotionFlag(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("target_motion")};
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    
    auto target_motion = getInput<int>("target_motion");
    if (!target_motion)
    {
      ROS_WARN("SetMotionFlag: Failed to get target_motion input");
      return BT::NodeStatus::FAILURE;
    }
    
    int target = target_motion.value();
    
    // Check cooldown time
    ros::Time cooldown_end;
    try
    {
      cooldown_end = bb->get<ros::Time>("attack_cooldown_end_time");
    }
    catch (...)
    {
      cooldown_end = ros::Time(0);
      bb->set("attack_cooldown_end_time", cooldown_end);
    }
    
    bool cooldown_finished = (cooldown_end.toSec() == 0) || (ros::Time::now() >= cooldown_end);
    
    if (!cooldown_finished)
    {
      // Still in cooldown, don't change motion_flag
      // ROS_DEBUG("SetMotionFlag: target=%d, still in cooldown, skipping", target);
      return BT::NodeStatus::SUCCESS;
    }
    
    // Cooldown finished, set motion_flag to target value
    bb->set("motion_flag", target);
    ROS_DEBUG("SetMotionFlag: Cooldown finished, motion_flag set to %d", target);
    return BT::NodeStatus::SUCCESS;
  }
};

// SetMotion: Sets motion value from port input
class SetMotion : public BT::SyncActionNode
{
public:
  explicit SetMotion(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("input_port")};
  }

  BT::NodeStatus tick() override
  {
    auto motion_str = getInput<std::string>("input_port");
    if (!motion_str)
    {
      return BT::NodeStatus::FAILURE;
    }

    auto bb = config().blackboard;
    bb->set("motion", toUpper(motion_str.value()));
    // ROS_INFO("SetMotion: set motion to %s", toUpper(motion_str.value()).c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

// PublishMotion: Publishes motion_flag to ROS topic
class PublishMotion : public BT::SyncActionNode
{
private:
  ros::Publisher* publisher_;
  bool* publish_on_change_only_;

public:
  explicit PublishMotion(const std::string& name, const BT::NodeConfiguration& config,
                         ros::Publisher* publisher, bool* publish_on_change_only)
    : BT::SyncActionNode(name, config), publisher_(publisher), publish_on_change_only_(publish_on_change_only)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int motion_flag = bb->get<int>("motion_flag");

    if (*publish_on_change_only_)
    {
      // Check if motion_flag has changed since last publish
      int last_motion_flag = -1;
      bool have_last = false;
      try
      {
        last_motion_flag = bb->get<int>("motion.last_flag");
        have_last = true;
      }
      catch (...)
      {
        have_last = false;
      }

      if (have_last && motion_flag == last_motion_flag)
      {
        // ROS_DEBUG("PublishMotion: motion_flag unchanged (%d), skipping publish", motion_flag);
        return BT::NodeStatus::SUCCESS;
      }
      bb->set("motion.last_flag", motion_flag);
    }

    std_msgs::UInt8 msg;
    msg.data = (uint8_t)motion_flag;
    publisher_->publish(msg);
    // ROS_INFO("PublishMotion: published motion_flag = %d", motion_flag);
    return BT::NodeStatus::SUCCESS;
  }
};

void RegisterMotionChangeNodes(BT::BehaviorTreeFactory& factory, ros::Publisher* motion_pub, bool* publish_on_change_only)
{
  factory.registerNodeType<CheckArrived>("CheckArrived");
  factory.registerNodeType<CheckAttacked>("CheckAttacked");
  
  factory.registerBuilder<SetMotionFlag>(
      "SetMotionFlag", [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<SetMotionFlag>(name, config);
      });
  
  factory.registerBuilder<SetMotion>(
      "SetMotion", [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<SetMotion>(name, config);
      });

  factory.registerBuilder<PublishMotion>(
      "PublishMotion", [motion_pub, publish_on_change_only](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<PublishMotion>(name, config, motion_pub, publish_on_change_only);
      });
}

void RegisterSetMotionNode(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetMotion>("SetMotion");
}

void RegisterPublishMotionNode(BT::BehaviorTreeFactory& factory, ros::Publisher* publisher)
{
  // Note: PublishMotion requires publisher in constructor, needs custom registration
  // This will be handled in strategy_node.cpp
}
