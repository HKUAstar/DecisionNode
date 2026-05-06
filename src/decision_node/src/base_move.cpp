#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <string>

#include "decision_node/base_move.hpp"

// Launch 参数 (nh 命名空间下):
//   pose.pose.orientation.x  float64  在2D中为 0.0
//   pose.pose.orientation.y  float64  在2D中为 0.0
//   pose.pose.orientation.z  float64  对应偏航角（Yaw）的正弦值
//   pose.pose.orientation.w  float64  对应偏航角（Yaw）的余弦值
class CalculateAngle : public BT::SyncActionNode
{
public:
  CalculateAngle(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle* nh)
    : BT::SyncActionNode(name, config), nh_(nh)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    // 从参数服务器读取目标四元数
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    nh_->param("pose_pose_orientation_x", qx, qx);
    nh_->param("pose_pose_orientation_y", qy, qy);
    nh_->param("pose_pose_orientation_z", qz, qz);
    nh_->param("pose_pose_orientation_w", qw, qw);

    // 从四元数计算服务器输入的yaw角（关于世界系的夹角）
    // yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2))
    // 返回范围: [-π, π] 弧度制
    double server_yaw = std::atan2(
      2.0 * (qw * qz + qx * qy),
      1.0 - 2.0 * (qy * qy + qz * qz)
    );

    // 获取当前云台的gimbal_angle
    double gimbal_angle = 0.0;
    try
    {
      gimbal_angle = bb->get<double>("odom.gimbal_angle");
    }
    catch (...)
    {
      ROS_WARN_THROTTLE(1.0, "CalculateAngle: odom.gimbal_angle not found in blackboard, using 0.0");
    }

    // 获取当前MCU的yaw_angle
    double yaw_angle = 0.0;
    try
    {
      yaw_angle = bb->get<double>("odom.yaw_angle");
    }
    catch (...)
    {
      ROS_WARN_THROTTLE(1.0, "CalculateAngle: odom.yaw_angle not found in blackboard, using 0.0");
    }

    // 目标角度取 server_yaw ± π/4，分别计算 result_yaw，
    // 选择与 yaw_angle 差值更小的那个
    double candidate1 = server_yaw + M_PI_4;
    double candidate2 = server_yaw - M_PI_4;

    double result1 = candidate1 - gimbal_angle;
    double result2 = candidate2 - gimbal_angle;

    double diff1 = std::abs(std::atan2(std::sin(result1 - yaw_angle), std::cos(result1 - yaw_angle)));
    double diff2 = std::abs(std::atan2(std::sin(result2 - yaw_angle), std::cos(result2 - yaw_angle)));

    double result_yaw = (diff1 <= diff2) ? result1 : result2;
    double target_yaw = (diff1 <= diff2) ? candidate1 : candidate2;

    // 归一化到 [-π, π]
    result_yaw = std::atan2(std::sin(result_yaw), std::cos(result_yaw));

    bb->set("odom.target_yaw", result_yaw);

    ROS_DEBUG("CalculateAngle: q=(%.3f, %.3f, %.3f, %.3f), server_yaw=%.3f, target_yaw=%.3f, gimbal=%.3f, yaw_angle=%.3f, result=%.3f",
              qx, qy, qz, qw, server_yaw, target_yaw, gimbal_angle, yaw_angle, result_yaw);

    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle* nh_;
};

class CompareYaw : public BT::ConditionNode
{
public:
  CompareYaw(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("threshold", 0.0167 * M_PI, "允许的角度误差阈值（弧度），默认0.0167π≈3°"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    // 读取阈值
    double threshold = 0.0167 * M_PI;
    (void)getInput("threshold", threshold);

    // 读取MCU云台当前的yaw角（取负）
    double yaw_angle = 0.0;
    try { yaw_angle = bb->get<double>("odom.yaw_angle"); }
    catch (...)
    {
      ROS_WARN_THROTTLE(1.0, "CompareYaw: odom.yaw_angle not found in blackboard");
      return BT::NodeStatus::FAILURE;
    }
    double current_yaw = -yaw_angle;

    // 读取目标yaw角
    double target_yaw = 0.0;
    try { target_yaw = bb->get<double>("odom.target_yaw"); }
    catch (...)
    {
      ROS_WARN_THROTTLE(1.0, "CompareYaw: odom.target_yaw not found in blackboard");
      return BT::NodeStatus::FAILURE;
    }

    // 计算角度差并归一化到 [-π, π]
    double diff = target_yaw - current_yaw;
    diff = std::atan2(std::sin(diff), std::cos(diff));

    bool within_threshold = (std::abs(diff) <= threshold);

    ROS_DEBUG("CompareYaw: yaw_angle=%.4f, neg_yaw=%.4f, target=%.4f, diff=%.4f, threshold=%.4f, result=%s",
              yaw_angle, current_yaw, target_yaw, diff, threshold, within_threshold ? "SUCCESS" : "FAILURE");

    return within_threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class PublishTargetYaw : public BT::SyncActionNode
{
public:
  PublishTargetYaw(const std::string& name, const BT::NodeConfiguration& config,
                   ros::Publisher* pub)
    : BT::SyncActionNode(name, config), pub_(pub)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    float target_yaw = 0.0f;
    try { target_yaw = static_cast<float>(bb->get<double>("odom.target_yaw")); }
    catch (...) { return BT::NodeStatus::SUCCESS; }

    float last_yaw = -999.0f;
    try { last_yaw = bb->get<float>("odom.last_target_yaw"); } catch (...) {}

    // 只在值变化时才发布
    if (std::abs(target_yaw - last_yaw) < 0.001f)
      return BT::NodeStatus::SUCCESS;

    bb->set("odom.last_target_yaw", target_yaw);

    std_msgs::Float32 msg;
    msg.data = target_yaw;
    pub_->publish(msg);

    ROS_DEBUG("PublishTargetYaw: published target_yaw = %.4f rad", target_yaw);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::Publisher* pub_;
};


class ResetNavArrived : public BT::SyncActionNode
{
public:
  ResetNavArrived(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    bb->set("nav.arrived", false);
    ROS_DEBUG("ResetNavArrived: nav.arrived set to false");
    return BT::NodeStatus::SUCCESS;
  }
};

void RegisterBaseMoveNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle* nh, ros::Publisher* target_yaw_pub)
{
  factory.registerBuilder<CalculateAngle>(
    "CalculateAngle", [nh](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<CalculateAngle>(name, config, nh);
    });

  factory.registerNodeType<CompareYaw>("CompareYaw");

  factory.registerBuilder<PublishTargetYaw>(
    "PublishTargetYaw", [target_yaw_pub](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<PublishTargetYaw>(name, config, target_yaw_pub);
    });

  factory.registerNodeType<ResetNavArrived>("ResetNavArrived");
}
