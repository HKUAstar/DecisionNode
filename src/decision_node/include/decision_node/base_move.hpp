#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

void RegisterBaseMoveNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle* nh, ros::Publisher* target_yaw_pub);
