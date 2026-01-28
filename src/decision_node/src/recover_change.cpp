#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "decision_node/recover_change.hpp"

// IsHealthFull: Check if current HP equals max HP
class IsHealthFull : public BT::ConditionNode
{
public:
  IsHealthFull(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("max_hp", 400, "Maximum HP value"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int remain_hp = bb->get<int>("ref.remain_hp");
    
    int max_hp = 400;
    (void)getInput("max_hp", max_hp);
    
    bool is_full = (remain_hp >= max_hp);
    // ROS_INFO("IsHealthFull: remain_hp=%d, max_hp=%d, is_full=%d", remain_hp, max_hp, is_full);
    return is_full ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// SetRecover: Set recover flag to 0 or 1
class SetRecover : public BT::SyncActionNode
{
public:
  SetRecover(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("value", 0, "Recover value: 0 or 1")};
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int value = 0;
    (void)getInput("value", value);
    
    bb->set("recover", value);
    ROS_DEBUG("SetRecover: recover set to %d", value);
    return BT::NodeStatus::SUCCESS;
  }
};

// PublishRecover: Publish recover value to ROS topic
class PublishRecover : public BT::SyncActionNode
{
private:
  ros::Publisher* publisher_;
  bool* publish_on_change_only_;

public:
  PublishRecover(const std::string& name, const BT::NodeConfiguration& config, ros::Publisher* pub, bool* publish_on_change_only)
    : BT::SyncActionNode(name, config), publisher_(pub), publish_on_change_only_(publish_on_change_only)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int recover = 0;
    try
    {
      recover = bb->get<int>("recover");
    }
    catch (...)
    {
      recover = 0;
      bb->set("recover", recover);
    }

    if (*publish_on_change_only_)
    {
      int last_recover = 0;
      try
      {
        last_recover = bb->get<int>("recover_last");
      }
      catch (...)
      {
        last_recover = -1;
      }

      if (last_recover == recover)
      {
        return BT::NodeStatus::SUCCESS;  // No change, skip publish
      }
      bb->set("recover_last", recover);
    }

    std_msgs::Int32 msg;
    msg.data = recover;
    ROS_DEBUG("PublishRecover: Publishing recover=%d", recover);
    publisher_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }
};

void RegisterRecoverChangeNodes(BT::BehaviorTreeFactory& factory, ros::Publisher* recover_pub)
{
  factory.registerNodeType<IsHealthFull>("IsHealthFull");
  factory.registerNodeType<SetRecover>("SetRecover");
  
  // recover_pub 为 nullptr 时不注册 PublishRecover（可选）
  if (recover_pub != nullptr)
  {
    factory.registerBuilder<PublishRecover>(
      "PublishRecover", [recover_pub](const std::string& name, const BT::NodeConfiguration& config) {
        static bool publish_on_change = true;  // 可根据需要调整
        return std::make_unique<PublishRecover>(name, config, recover_pub, &publish_on_change);
      });
  }
}
