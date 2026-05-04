#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <cmath>
#include <string>

#include "decision_node/base_move.hpp"

// =====================================================
// CalculateAngle: 根据launch文件中配置的目标四元数计算目标yaw角
// =====================================================
// 从ROS参数服务器读取四元数 (pose.pose.orientation.x/y/z/w)
// 计算该位姿的yaw角（世界系夹角）
// 然后用该角度减去当前云台角度 (odom.gimbal_angle)
// 结果存入黑板 odom.target_yaw
//
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

// =====================================================
// 注册所有 base_move 节点
// =====================================================
void RegisterBaseMoveNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle* nh)
{
  factory.registerBuilder<CalculateAngle>(
    "CalculateAngle", [nh](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<CalculateAngle>(name, config, nh);
    });
}
