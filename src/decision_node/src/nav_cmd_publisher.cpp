#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_cmd_publisher");
    ros::NodeHandle nh("~");
    
    // 发布者
    ros::Publisher nav_pub = nh.advertise<geometry_msgs::Twist>("/cmd/nav", 1);
    
    // 从launch文件读取参数
    float x_velocity = 0.0f;
    float y_velocity = 0.0f;
    float omega = 0.0f;
    uint8_t received = 0;
    uint8_t arrived = 0;
    
    nh.param("x_velocity", x_velocity, 0.0f);
    nh.param("y_velocity", y_velocity, 0.0f);
    nh.param("omega", omega, 0.0f);
    nh.param("received", received, (uint8_t)0);
    nh.param("arrived", arrived, (uint8_t)0);
    
    ROS_INFO("Nav Command Publisher initialized:");
    ROS_INFO("  x_velocity: %.2f", x_velocity);
    ROS_INFO("  y_velocity: %.2f", y_velocity);
    ROS_INFO("  omega: %.2f", omega);
    ROS_INFO("  received: %u", received);
    ROS_INFO("  arrived: %u", arrived);
    
    // 创建并发送命令
    geometry_msgs::Twist cmd;
    cmd.linear.x = x_velocity;
    cmd.linear.y = y_velocity;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = omega;
    
    ros::Rate loop_rate(10);  // 10Hz发布
    
    while (ros::ok())
    {
        nav_pub.publish(cmd);
        ROS_DEBUG("Published nav command: x=%.2f, y=%.2f, omega=%.2f", 
                  x_velocity, y_velocity, omega);
        loop_rate.sleep();
    }
    
    return 0;
}
