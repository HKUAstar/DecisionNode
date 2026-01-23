#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
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

class SetMotionFlagToOne : public BT::SyncActionNode
{
public:
  SetMotionFlagToOne(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    
    // 检查冷却时间是否仍在进行
    ros::Time cooldown_end = bb->get<ros::Time>("attack_cooldown_end_time");
    bool cooldown_finished = (cooldown_end.toSec() == 0) || (ros::Time::now() >= cooldown_end);
    
    if (!cooldown_finished)
    {
      // ROS_INFO("SetMotionFlagToOne: Still in cooldown, NOT setting motion_flag to 1");
      return BT::NodeStatus::SUCCESS;  // 在冷却中，不改变motion_flag
    }
    
    bb->set("motion_flag", 1);
    // ROS_INFO("SetMotionFlagToOne: set motion_flag to 1");
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

    std_msgs::Int32 msg;
    msg.data = motion_flag;
    publisher_->publish(msg);
    // ROS_INFO("PublishMotion: published motion_flag = %d", motion_flag);
    return BT::NodeStatus::SUCCESS;
  }
};

void RegisterMotionChangeNodes(BT::BehaviorTreeFactory& factory, ros::Publisher* motion_pub, bool* publish_on_change_only)
{
  factory.registerNodeType<CheckArrived>("CheckArrived");
  factory.registerNodeType<SetMotionFlagToOne>("SetMotionFlagToOne");
  
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
