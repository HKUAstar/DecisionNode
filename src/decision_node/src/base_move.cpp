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
    nh_->param("pose.pose.orientation.x", qx, qx);
    nh_->param("pose.pose.orientation.y", qy, qy);
    nh_->param("pose.pose.orientation.z", qz, qz);
    nh_->param("pose.pose.orientation.w", qw, qw);

    // 从四元数计算yaw角（关于世界系的夹角）
    // yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2))
    // 返回范围: [-π, π] 弧度制
    double target_yaw = std::atan2(
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

    // 目标yaw角减去当前云台角度
    double result_yaw = target_yaw - gimbal_angle;

    // 归一化到 [-π, π]
    result_yaw = std::atan2(std::sin(result_yaw), std::cos(result_yaw));

    bb->set("odom.target_yaw", result_yaw);

    ROS_DEBUG("CalculateAngle: q=(%.3f, %.3f, %.3f, %.3f), target_yaw=%.3f, gimbal=%.3f, result=%.3f",
              qx, qy, qz, qw, target_yaw, gimbal_angle, result_yaw);

    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle* nh_;
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

// =====================================================
// 注册所有 base_move 节点
// =====================================================
void RegisterBaseMoveNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle* nh, ros::Publisher* target_yaw_pub)
{
  factory.registerBuilder<CalculateAngle>(
    "CalculateAngle", [nh](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<CalculateAngle>(name, config, nh);
    });

  factory.registerBuilder<PublishTargetYaw>(
    "PublishTargetYaw", [target_yaw_pub](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<PublishTargetYaw>(name, config, target_yaw_pub);
    });
}
