#include <ros/ros.h>
#include <serial/serial.h>
#include <decision_node/mcu_comm.hpp>
#include <cstring>
#include <cmath>

/**
 * MCU Simulator - 用于测试MCU通讯模块
 * 通过虚拟串口对(socat)模拟下位机发送数据
 * 
 * 使用方法:
 * 1. 创建虚拟串口对:
 *    socat -d -d pty,raw,echo=0 pty,raw,echo=0
 *    获得输出如: /dev/pts/11 <--> /dev/pts/12
 * 
 * 2. 启动通讯模块(监听其中一个口):
 *    rosrun decision_node mcu_communicator _serial_port:=/dev/pts/11
 * 
 * 3. 启动模拟器(写到另一个口):
 *    rosrun decision_node mcu_simulator _serial_port:=/dev/pts/12
 */
class MCUSimulator
{
public:
    MCUSimulator() : nh_("~"), serial_port_(""), serial_baudrate_(115200), 
                    counter_(0), frame_count_(0)
    {
        nh_.param("serial_port", serial_port_, std::string("/dev/pts/12"));
        nh_.param("baudrate", serial_baudrate_, 115200);
        
        try
        {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(serial_baudrate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
            
            if (serial_.isOpen())
            {
                ROS_INFO("MCU Simulator opened port: %s @ %d baud", 
                         serial_port_.c_str(), serial_baudrate_);
            }
            else
            {
                ROS_ERROR("Failed to open serial port: %s", serial_port_.c_str());
            }
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Serial exception: %s", e.what());
        }
    }
    
    ~MCUSimulator()
    {
        if (serial_.isOpen())
        {
            serial_.close();
        }
    }
    
    void run()
    {
        ros::Rate loop_rate(10);  // 10Hz
        
        while (ros::ok())
        {
            if (!serial_.isOpen())
            {
                try
                {
                    serial_.open();
                    ROS_INFO("Reconnected to serial port");
                }
                catch (const serial::SerialException& e)
                {
                    ROS_WARN("Failed to reconnect: %s", e.what());
                    loop_rate.sleep();
                    continue;
                }
            }
            
            sendFrame();
            loop_rate.sleep();
        }
    }
    
private:
    ros::NodeHandle nh_;
    serial::Serial serial_;
    std::string serial_port_;
    int serial_baudrate_;
    uint32_t counter_;
    uint32_t frame_count_;
    
    void sendFrame()
    {
        MCUDataFrame frame;
        frame.sof = MCU_FRAME_SOF;
        
        // 模拟数据（这些字段在reserved_1_17中，不能直接访问）
        // float yaw_angle = std::sin(counter_ * 0.01f) * 3.14159f;  // 云台摇摆
        // float chassis_imu = std::cos(counter_ * 0.01f) * 3.14159f; // 底盘旋转
        // int motion_mode = (counter_ / 50) % 4;  // 0-3循环
        // float operator_x = 100.0f + 50.0f * std::sin(counter_ * 0.01f);
        // float operator_y = 200.0f + 50.0f * std::cos(counter_ * 0.01f);
        
        frame.robot_id = 7;          // 哨兵ID
        frame.robot_color = 0;       // 红方
        frame.game_progress = 1;     // 比赛进行中
        
        // 模拟血量变化
        frame.red_1_hp = 300 + (counter_ % 50) * 2;
        frame.red_3_hp = 250 + (counter_ % 40) * 2;
        frame.red_7_hp = 200 + (counter_ % 30) * 2;
        
        frame.blue_1_hp = 320 + (counter_ % 50) * 2;
        frame.blue_3_hp = 280 + (counter_ % 40) * 2;
        frame.blue_7_hp = 220 + (counter_ % 30) * 2;
        
        // 死亡状态(模拟)
        frame.red_dead = (counter_ % 200 < 50) ? 0x11 : 0x00;  // 英雄和3号交替死亡
        frame.blue_dead = 0x00;
        
        // 自身状态
        frame.self_hp = 400 - (counter_ % 100);
        frame.self_max_hp = 400;
        frame.bullet_remain = 1000 - (counter_ % 500);
        frame.occupy_status = (counter_ / 100) % 3;  // 0: 未占领, 1: 友方, 2: 敌方
        
        frame.eof = MCU_FRAME_EOF;
        frame.crc8 = 0x00;  // 简化处理，实际应计算CRC8
        
        // 发送帧
        try
        {
            std::vector<uint8_t> buffer((uint8_t*)&frame, (uint8_t*)&frame + sizeof(MCUDataFrame));
            serial_.write(buffer);
            
            frame_count_++;
            if (frame_count_ % 100 == 0)
            {
                ROS_INFO("MCU Simulator: sent %u frames, "
                        "self_hp=%u, bullet=%u, occupy=%u",
                        frame_count_,
                        frame.self_hp, frame.bullet_remain, frame.occupy_status);
            }
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Serial write exception: %s", e.what());
            if (serial_.isOpen())
            {
                serial_.close();
            }
        }
        
        counter_++;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mcu_simulator");
    
    try
    {
        MCUSimulator simulator;
        ROS_INFO("MCU Simulator started");
        simulator.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Fatal error: %s", e.what());
        return 1;
    }
    
    return 0;
}
