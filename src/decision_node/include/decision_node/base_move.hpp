#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

/**
 * 注册 base_move 相关节点到行为树工厂
 * @param factory  行为树工厂
 * @param nh       ROS NodeHandle，用于读取参数服务器上的四元数配置
 */
void RegisterBaseMoveNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle* nh);
