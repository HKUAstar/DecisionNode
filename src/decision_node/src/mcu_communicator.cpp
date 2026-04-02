#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <decision_node/mcu_comm.hpp>
#include <thread>
#include <cstdio>
#include <algorithm>

class MCUCommunicator
{
public:
    MCUCommunicator() : nh_("~"), serial_port_(""), serial_baudrate_(921600), 
                       frame_buffer_index_(0)
    {
        // 读取参数
        nh_.param("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
        nh_.param("baudrate", serial_baudrate_, 921600);
        
        // 读取导航发布频率 (默认50Hz)
        double nav_frequency = 50.0;
        nh_.param("nav_frequency", nav_frequency, 50.0);
        double nav_period = 1.0 / nav_frequency;  // 转换为周期(秒)
        
        pub_game_progress_ = nh_.advertise<std_msgs::UInt8>("/referee/game_progress", 1);
        pub_remain_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/remain_hp", 1);
        pub_bullet_remain_ = nh_.advertise<std_msgs::UInt16>("/referee/bullet_remain", 1);
        pub_occupy_status_ = nh_.advertise<std_msgs::UInt8>("/referee/occupy_status", 1);
        
        pub_robot_id_ = nh_.advertise<std_msgs::UInt8>("/robot/robot_id", 1);
        pub_robot_color_ = nh_.advertise<std_msgs::UInt8>("/robot/robot_color", 1);
        pub_self_hp_ = nh_.advertise<std_msgs::UInt16>("/robot/self_hp", 1);
        pub_self_max_hp_ = nh_.advertise<std_msgs::UInt16>("/robot/self_max_hp", 1);
        
        pub_red_1_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/red_1_hp", 1);
        pub_red_3_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/red_3_hp", 1);
        pub_red_7_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/red_7_hp", 1);
        pub_blue_1_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_1_hp", 1);
        pub_blue_3_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_3_hp", 1);
        pub_blue_7_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_7_hp", 1);
        
        pub_red_dead_ = nh_.advertise<std_msgs::UInt16>("/referee/red_dead", 1);
        pub_blue_dead_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_dead", 1);
        
        pub_friendly_score_ = nh_.advertise<std_msgs::Int32>("/referee/friendly_score", 1);
        pub_enemy_score_ = nh_.advertise<std_msgs::Int32>("/referee/enemy_score", 1);
        
        pub_enemy_hero_ = nh_.advertise<geometry_msgs::Point>("/enemy/hero_position", 1);
        pub_enemy_engineer_ = nh_.advertise<geometry_msgs::Point>("/enemy/engineer_position", 1);
        pub_enemy_standard_3_ = nh_.advertise<geometry_msgs::Point>("/enemy/standard_3_position", 1);
        pub_enemy_standard_4_ = nh_.advertise<geometry_msgs::Point>("/enemy/standard_4_position", 1);
        pub_enemy_sentry_ = nh_.advertise<geometry_msgs::Point>("/enemy/sentry_position", 1);
        pub_suggested_target_ = nh_.advertise<std_msgs::UInt8>("/radar/suggested_target", 1);
        pub_radar_flags_ = nh_.advertise<std_msgs::UInt16>("/radar/radar_flags", 1);
        
        // ===== 新增：发布分离的TF数据 =====
        pub_yaw_angle_ = nh_.advertise<std_msgs::Float32>("/mcu/yaw_angle", 1);
        pub_chassis_imu_ = nh_.advertise<std_msgs::Float32>("/mcu/chassis_imu", 1);
        // ===== 新增结束 =====
        sub_navigation_ = nh_.subscribe<geometry_msgs::Vector3>("/navigation", 1,
                                                               &MCUCommunicator::navigationCallback, this);
        sub_nav_received_ = nh_.subscribe<std_msgs::UInt8>("/nav_received", 1,
                                                          &MCUCommunicator::navReceivedCallback, this);
        sub_dstar_status_ = nh_.subscribe<std_msgs::Bool>("/dstar_status", 1,
                                                         &MCUCommunicator::dstarStatusCallback, this);
        sub_cmd_vel_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,
                                                          &MCUCommunicator::cmdVelCallback, this);
        
        // 创建导航命令定时器
        navigation_timer_ = nh_.createTimer(ros::Duration(nav_period),
                                            &MCUCommunicator::navigationTimerCallback, this);
        
        try
        {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(serial_baudrate_);
            serial_.setBytesize(serial::eightbits);      // 数据位：8
            serial_.setParity(serial::parity_none);      // 校验位：None
            serial_.setStopbits(serial::stopbits_one);   // 停止位：1
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
            
            if (serial_.isOpen())
            {
                ROS_INFO("MCU Serial port opened successfully: %s @ %d baud", 
                         serial_port_.c_str(), serial_baudrate_);
            }
            else
            {
                ROS_ERROR("Failed to open serial port: %s", serial_port_.c_str());
                ROS_ERROR("Debugging steps:");
                ROS_ERROR("   1. Check device exists: ls -la %s", serial_port_.c_str());
                ROS_ERROR("   2. Check permissions: stat %s", serial_port_.c_str());
            }
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Serial exception during port setup: %s", e.what());
        }
        
        recv_thread_ = std::thread(&MCUCommunicator::receiveThread, this);
    }
    
    ~MCUCommunicator()
    {
        if (recv_thread_.joinable())
        {
            recv_thread_.join();
        }
        if (serial_.isOpen())
        {
            serial_.close();
        }
    }
    
private:
    ros::NodeHandle nh_;
    serial::Serial serial_;
    std::string serial_port_;
    int serial_baudrate_;
    
    // ROS 发布者
    ros::Publisher pub_game_progress_;
    ros::Publisher pub_remain_hp_;
    ros::Publisher pub_bullet_remain_;
    ros::Publisher pub_occupy_status_;
    
    ros::Publisher pub_robot_id_;
    ros::Publisher pub_robot_color_;
    ros::Publisher pub_self_hp_;
    ros::Publisher pub_self_max_hp_;
    
    ros::Publisher pub_red_1_hp_;
    ros::Publisher pub_red_3_hp_;
    ros::Publisher pub_red_7_hp_;
    ros::Publisher pub_blue_1_hp_;
    ros::Publisher pub_blue_3_hp_;
    ros::Publisher pub_blue_7_hp_;
    
    ros::Publisher pub_red_dead_;
    ros::Publisher pub_blue_dead_;
    
    ros::Publisher pub_friendly_score_;
    ros::Publisher pub_enemy_score_;
    
    ros::Publisher pub_enemy_hero_;
    ros::Publisher pub_enemy_engineer_;
    ros::Publisher pub_enemy_standard_3_;
    ros::Publisher pub_enemy_standard_4_;
    ros::Publisher pub_enemy_sentry_;
    ros::Publisher pub_suggested_target_;
    ros::Publisher pub_radar_flags_;
    
    // ===== 新增：分离的TF数据topic =====
    ros::Publisher pub_yaw_angle_;      // 云台yaw弧度
    ros::Publisher pub_chassis_imu_;    // 底盘IMU弧度
    // ===== 新增结束 =====
    
    // ROS 订阅者

    ros::Subscriber sub_navigation_;
    ros::Subscriber sub_nav_received_;
    ros::Subscriber sub_dstar_status_;
    ros::Subscriber sub_cmd_vel_;
    ros::Timer navigation_timer_;
    
    // 发送缓冲
    uint8_t tx_buffer_[256];
    size_t tx_buffer_index_;
    
    // 接收缓冲
    uint8_t frame_buffer_[MCU_FRAME_SIZE];
    size_t frame_buffer_index_;
    std::thread recv_thread_;
    

    
    // 导航数据变量
    float current_nav_vx_ = 0.0f;
    float current_nav_vy_ = 0.0f;
    float current_nav_z_angle_ = 0.0f;
    uint8_t current_nav_received_ = 0;
    uint8_t current_nav_arrived_ = 0;
    
    // 敌方位置缓存 - 用于处理-8888无效值
    float cached_enemy_hero_x_ = 0.0f;
    float cached_enemy_hero_y_ = 0.0f;
    float cached_enemy_engineer_x_ = 0.0f;
    float cached_enemy_engineer_y_ = 0.0f;
    float cached_enemy_standard_3_x_ = 0.0f;
    float cached_enemy_standard_3_y_ = 0.0f;
    float cached_enemy_standard_4_x_ = 0.0f;
    float cached_enemy_standard_4_y_ = 0.0f;
    float cached_enemy_sentry_x_ = 0.0f;
    float cached_enemy_sentry_y_ = 0.0f;
    
    // 分数追踪变量
    int32_t friendly_score_ = 200;  // 己方初始分数
    int32_t enemy_score_ = 200;     // 敌方初始分数
    ros::Time last_score_update_time_;  // 上次更新分数的时间
    int last_occupy_status_ = 0;    // 上一帧的占领状态
    uint16_t last_red_dead_ = 0;    // 上一帧红方死亡状态
    uint16_t last_blue_dead_ = 0;   // 上一帧蓝方死亡状态
    uint8_t robot_color_ = 0;       // 0=red, 1=blue
    

    // Navigation
    void navigationCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        sendNavigationCommand(msg->x, msg->y, msg->z);
    }
    
    // Nav Received
    void navReceivedCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_nav_received_ = msg->data;
        // ROS_DEBUG("Nav received updated: received=%u", current_nav_received_);
    }
    
    // D* Status
    void dstarStatusCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        current_nav_arrived_ = msg->data ? 1 : 0;
        ROS_INFO("D* status updated: raw=%s, arrived=%u",
                 msg->data ? "true" : "false",
                 static_cast<unsigned int>(current_nav_arrived_));
    }
    
    // Cmd Vel: 订阅速度命令，更新导航数据变量
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        current_nav_vx_ = msg->linear.x;
        current_nav_vy_ = msg->linear.y;
        current_nav_z_angle_ = msg->angular.z;
        
        ROS_DEBUG("CmdVel received: vx=%.4f, vy=%.4f, z_angle=%.4f", 
                  current_nav_vx_, current_nav_vy_, current_nav_z_angle_);
    }
    
    // 导航命令定时器回调 - 固定频率发送NavigationFrame到下位机
    void navigationTimerCallback(const ros::TimerEvent& event)
    {
        sendNavigationCommand(current_nav_vx_, current_nav_vy_, current_nav_z_angle_);
    }
    
    // 验证敌方坐标有效性（-8888为无效值）
    float validateEnemyCoordinate(float new_value, float cached_value)
    {
        // 如果新值为-8888（无效值），返回缓存的旧值
        if (new_value == -8888.0f)
        {
            return cached_value;
        }
        // 否则返回新值并更新缓存
        return new_value;
    }
    

    // 发送导航命令到下位机
    void sendNavigationCommand(float vx, float vy, float z_angle)
    {
        current_nav_vx_ = vx;
        current_nav_vy_ = vy;
        current_nav_z_angle_ = z_angle;
        
        NavigationCommandFrame frame;
        
        // 初始化帧头
        frame.header.sof[0] = HK_FRAME_SOF_H;      // 'H' (0x48)
        frame.header.sof[1] = HK_FRAME_SOF_K;      // 'K' (0x4B)
        frame.header.length = sizeof(NavigationCommandFrame);  // 整包长度 (小端序)
        frame.header.packet_type = 0x02;           // 运动指令帧 (Packet Type = 0x02)
        frame.header.reserved = 0;
        static uint8_t packet_seq = 0;
        frame.header.packet_seq = packet_seq++;    // 包序号递增
        frame.header.reserved2 = 0;
        
        // 计算Header CRC8 (对字节0-7)
        frame.header.header_crc8 = calculateCRC8((uint8_t*)&frame.header, 8, 0xFF);
        
        // 初始化数据段
        frame.data.reserved0 = 0;                  // 空变量
        frame.data.at_place = current_nav_arrived_;                 // 保留
        // 单位转换：m/s → mm/s, rad/s → 0.01 rad/s, clamp to int16 range
        int32_t vx_mm = (int32_t)(vx * 1000.0f);
        int32_t vy_mm = (int32_t)(vy * 1000.0f);
        int32_t wz_centi = (int32_t)(z_angle * 100.0f);
        vx_mm = std::max((int32_t)-32768, std::min((int32_t)32767, vx_mm));
        vy_mm = std::max((int32_t)-32768, std::min((int32_t)32767, vy_mm));
        wz_centi = std::max((int32_t)-32768, std::min((int32_t)32767, wz_centi));
        frame.data.vx = (int16_t)vx_mm;            // mm/s
        frame.data.vy = (int16_t)vy_mm;            // mm/s
        frame.data.wz = (int16_t)wz_centi;         // 0.01 rad/s
//test
        ROS_INFO_THROTTLE(0.01,
                         "Send nav frame: at_place=%u, vx=%.4f m/s(%d mm/s), vy=%.4f m/s(%d mm/s), wz=%.4f rad/s(%d)",
                         static_cast<unsigned int>(frame.data.at_place),
                         vx, frame.data.vx,
                         vy, frame.data.vy,
                         z_angle, frame.data.wz);
        
        // 计算Packet CRC16 (对整个帧从字节0到数据段结尾, 21-4=17字节)
        frame.packet_crc16 = calculateCRC16((uint8_t*)&frame, 
                                           sizeof(NavigationCommandFrame) - 4, 0xFFFF);
        
        // 初始化帧尾
        frame.trailer[0] = HK_FRAME_TRAILER_K;    // 'K' (0x4B)
        frame.trailer[1] = HK_FRAME_TRAILER_H;    // 'H' (0x48)
        
        try
        {
            if (!serial_.isOpen())
            {
                ROS_ERROR("Serial port is CLOSED! Cannot send navigation command.");
                return;
            }
            
            int written = serial_.write((uint8_t*)&frame, sizeof(frame));
            
            if (written == (int)sizeof(frame))
            {
                ROS_DEBUG("Navigation command sent: vx=%.4f m/s, vy=%.4f m/s, wz=%.4f rad/s", 
                         vx, vy, z_angle);
            }
            else if (written > 0)
            {
                ROS_WARN("Partial write: expected %zu bytes, but only wrote %d bytes", 
                        sizeof(frame), written);
            }
            else
            {
                ROS_ERROR("Write failed: write returned %d", written);
            }
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Serial exception during navigation write: %s", e.what());
        }
    }
    
    // CRC16 查表实现 - DJI标准
    // 计算从start_pos开始，长度为len的字节的CRC16
    uint16_t calculateCRC16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF)
    {
        for (size_t i = 0; i < len; i++)
        {
            uint8_t index = (crc ^ data[i]) & 0xFF;
            crc = (crc >> 8) ^ CRC16_TABLE[index];
        }
        return crc;
    }
    
    // CRC16验证函数 - 验证HK协议数据帧
    bool verifyCRC16(MCUDataFrame* frame)
    {
        uint16_t received_crc = frame->packet_crc16;
        
        // Packet CRC16: 对字节0到Data结束计算（0-77共78字节）
        // 即从sof[0]开始到data末尾的所有数据，初始值0xFFFF
        uint16_t calculated_crc = calculateCRC16((uint8_t*)&frame->header, HK_FRAME_HEADER_SIZE + HK_FRAME_DATA_SIZE, 0xFFFF);
        
        if (received_crc != calculated_crc)
        {
            ROS_WARN("CRC16 mismatch: received=0x%04X, calculated=0x%04X", received_crc, calculated_crc);
            return false;
        }
        return true;
    }
    
    // Header CRC8 验证函数 - 验证HK协议帧头
    bool verifyHeaderCRC8(MCUDataFrame* frame)
    {
        // 这里可以添加Header CRC8验证，如果需要的话
        // 但根据协议，主要依赖Packet CRC16
        // 暂时保留此函数以便后续扩展
        return true;
    }
    
    void receiveThread()
    {
        ros::Rate loop_rate(100);  // 100Hz **接受频率
        
        while (ros::ok())
        {
            if (!serial_.isOpen())
            {
                // 尝试重新连接
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
            
            try
            {
                size_t available = serial_.available();
                if (available > 0)
                {
                    std::string data = serial_.read(available);
                    std::vector<uint8_t> buffer(data.begin(), data.end());
                    processReceivedData(buffer);
                }
            }
            catch (const serial::SerialException& e)
            {
                ROS_ERROR("Serial read exception: %s", e.what());
                if (serial_.isOpen())
                {
                    serial_.close();
                }
            }
            
            loop_rate.sleep();
        }
    }
    
    void processReceivedData(const std::vector<uint8_t>& data)
    {
        for (uint8_t byte : data)
        {
            // 寻找HK协议帧头 ('H' = 0x48)
            if (frame_buffer_index_ == 0)
            {
                if (byte == HK_FRAME_SOF_H)
                {
                    frame_buffer_[frame_buffer_index_++] = byte;
                }
                continue;
            }
            
            // 检查第二个字节是否为'K' (0x4B)
            if (frame_buffer_index_ == 1)
            {
                if (byte == HK_FRAME_SOF_K)
                {
                    frame_buffer_[frame_buffer_index_++] = byte;
                }
                else
                {
                    // 不是有效的帧头，重置
                    frame_buffer_index_ = 0;
                    // 检查当前字节是否为'H'（准备下一个帧）
                    if (byte == HK_FRAME_SOF_H)
                    {
                        frame_buffer_[frame_buffer_index_++] = byte;
                    }
                }
                continue;
            }
            
            // 接收数据
            frame_buffer_[frame_buffer_index_++] = byte;
            
            // 检查是否接收完整帧 (82字节)
            if (frame_buffer_index_ == HK_FRAME_SIZE)
            {
                // 验证帧尾 ('K', 'H')
                if (frame_buffer_[HK_FRAME_SIZE - 2] == HK_FRAME_TRAILER_K &&
                    frame_buffer_[HK_FRAME_SIZE - 1] == HK_FRAME_TRAILER_H)
                {
                    // 解析并发布数据
                    parseAndPublish();
                }
                else
                {
                    ROS_DEBUG("Invalid frame trailer: received 0x%02X 0x%02X (expected 0x%02X 0x%02X)",
                             frame_buffer_[HK_FRAME_SIZE - 2], frame_buffer_[HK_FRAME_SIZE - 1],
                             HK_FRAME_TRAILER_K, HK_FRAME_TRAILER_H);
                    
                    // 尝试重新同步：寻找缓冲区中的下一个帧头 ('H')
                    bool found_resync = false;
                    for (size_t i = 1; i < HK_FRAME_SIZE; i++)
                    {
                        if (frame_buffer_[i] == HK_FRAME_SOF_H && i + 1 < HK_FRAME_SIZE && 
                            frame_buffer_[i + 1] == HK_FRAME_SOF_K)
                        {
                            memmove(frame_buffer_, frame_buffer_ + i, HK_FRAME_SIZE - i);
                            frame_buffer_index_ = HK_FRAME_SIZE - i;
                            found_resync = true;
                            break;
                        }
                    }
                    
                    if (!found_resync)
                    {
                        frame_buffer_index_ = 0;  // 无法重新同步，重置缓冲区
                    }
                }
                
                // 如果成功解析，重置缓冲区
                if (frame_buffer_[HK_FRAME_SIZE - 2] == HK_FRAME_TRAILER_K &&
                    frame_buffer_[HK_FRAME_SIZE - 1] == HK_FRAME_TRAILER_H)
                {
                    frame_buffer_index_ = 0;
                }
            }
        }
    }
    
    void parseAndPublish()
    {
        // 将原始字节数据复制到结构体
        MCUDataFrame frame;
        memcpy(&frame, frame_buffer_, HK_FRAME_SIZE);
        
        // 验证帧头
        if (frame.header.sof[0] != HK_FRAME_SOF_H || frame.header.sof[1] != HK_FRAME_SOF_K)
        {
            ROS_WARN("Invalid frame header: SOF=0x%02X 0x%02X (expected 0x%02X 0x%02X)",
                    frame.header.sof[0], frame.header.sof[1], HK_FRAME_SOF_H, HK_FRAME_SOF_K);
            return;
        }
        
        // 验证帧尾
        if (frame.trailer[0] != HK_FRAME_TRAILER_K || frame.trailer[1] != HK_FRAME_TRAILER_H)
        {
            ROS_WARN("Invalid frame trailer: 0x%02X 0x%02X (expected 0x%02X 0x%02X)",
                    frame.trailer[0], frame.trailer[1], HK_FRAME_TRAILER_K, HK_FRAME_TRAILER_H);
            return;
        }
        
        // 验证Packet Type
        if (frame.header.packet_type != HK_PACKET_TYPE_GAME)
        {
            ROS_WARN("Invalid packet type: 0x%02X (expected 0x%02X)", 
                    frame.header.packet_type, HK_PACKET_TYPE_GAME);
            return;
        }
        
        // 验证CRC16校验
        if (!verifyCRC16(&frame))
        {
            ROS_WARN("CRC16 verification failed");
            return;
        }
        
        ROS_DEBUG("Valid HK frame received: game_progress=%u, self_hp=%u, crc16=0x%04X",
                 frame.data.game_progress, frame.data.self_hp, frame.packet_crc16);
        
        // 更新敌方位置数据（处理坐标有效性，单位转换cm -> m）
        cached_enemy_hero_x_ = frame.data.enemy_hero_x / 100.0f;
        cached_enemy_hero_y_ = frame.data.enemy_hero_y / 100.0f;
        cached_enemy_engineer_x_ = frame.data.enemy_engineer_x / 100.0f;
        cached_enemy_engineer_y_ = frame.data.enemy_engineer_y / 100.0f;
        cached_enemy_standard_3_x_ = frame.data.enemy_std3_x / 100.0f;
        cached_enemy_standard_3_y_ = frame.data.enemy_std3_y / 100.0f;
        cached_enemy_standard_4_x_ = frame.data.enemy_std4_x / 100.0f;
        cached_enemy_standard_4_y_ = frame.data.enemy_std4_y / 100.0f;
        cached_enemy_sentry_x_ = frame.data.enemy_sentry_x / 100.0f;
        cached_enemy_sentry_y_ = frame.data.enemy_sentry_y / 100.0f;
        
        // 更新机器人颜色（0=red, 1=blue）
        robot_color_ = frame.data.robot_color;
        
        // 发布数据到各个Topic
        std_msgs::UInt8 msg_uint8;
        std_msgs::UInt16 msg_uint16;
        std_msgs::Float32 msg_float;
        std_msgs::Int32 msg_int32;
        geometry_msgs::Point enemy_pos;

        // 云台yaw角固定为0
        // msg_float.data = frame.data.yaw_angle;
        msg_float.data = 0.0f;
        pub_yaw_angle_.publish(msg_float);
        
        // 比赛状态
        msg_uint8.data = frame.data.game_progress;
        pub_game_progress_.publish(msg_uint8);
        
        // 自身血量（从robot_state中获取）
        msg_uint16.data = frame.data.self_hp;
        pub_remain_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.bullet_remain;
        pub_bullet_remain_.publish(msg_uint16);
        
        // 占领状态
        msg_uint8.data = frame.data.occupy_status;
        pub_occupy_status_.publish(msg_uint8);
        
        // 机器人ID和颜色
        msg_uint8.data = frame.data.robot_id;
        pub_robot_id_.publish(msg_uint8);
        
        msg_uint8.data = frame.data.robot_color;
        pub_robot_color_.publish(msg_uint8);
        
        // 自身血量信息
        msg_uint16.data = frame.data.self_hp;
        pub_self_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.self_max_hp;
        pub_self_max_hp_.publish(msg_uint16);
        
        // 红方血量
        msg_uint16.data = frame.data.red_1_hp;
        pub_red_1_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.red_3_hp;
        pub_red_3_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.red_7_hp;
        pub_red_7_hp_.publish(msg_uint16);
        
        // 蓝方血量
        msg_uint16.data = frame.data.blue_1_hp;
        pub_blue_1_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.blue_3_hp;
        pub_blue_3_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.blue_7_hp;
        pub_blue_7_hp_.publish(msg_uint16);
        
        // 敌方位置数据
        enemy_pos.x = cached_enemy_hero_x_;
        enemy_pos.y = cached_enemy_hero_y_;
        enemy_pos.z = 0.0f;
        pub_enemy_hero_.publish(enemy_pos);
        
        enemy_pos.x = cached_enemy_engineer_x_;
        enemy_pos.y = cached_enemy_engineer_y_;
        enemy_pos.z = 0.0f;
        pub_enemy_engineer_.publish(enemy_pos);
        
        enemy_pos.x = cached_enemy_standard_3_x_;
        enemy_pos.y = cached_enemy_standard_3_y_;
        enemy_pos.z = 0.0f;
        pub_enemy_standard_3_.publish(enemy_pos);
        
        enemy_pos.x = cached_enemy_standard_4_x_;
        enemy_pos.y = cached_enemy_standard_4_y_;
        enemy_pos.z = 0.0f;
        pub_enemy_standard_4_.publish(enemy_pos);
        
        enemy_pos.x = cached_enemy_sentry_x_;
        enemy_pos.y = cached_enemy_sentry_y_;
        enemy_pos.z = 0.0f;
        pub_enemy_sentry_.publish(enemy_pos);
        
        // 雷达相关
        msg_uint8.data = frame.data.suggested_target;
        pub_suggested_target_.publish(msg_uint8);
        
        msg_uint16.data = frame.data.radar_flags;
        pub_radar_flags_.publish(msg_uint16);
        
        // 死亡位标记
        msg_uint16.data = frame.data.red_dead_bits;
        pub_red_dead_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.blue_dead_bits;
        pub_blue_dead_.publish(msg_uint16);
        
        // 分数更新
        updateScore(frame.data);
        
        msg_int32.data = friendly_score_;
        pub_friendly_score_.publish(msg_int32);
        msg_int32.data = enemy_score_;
        pub_enemy_score_.publish(msg_int32);
        
        ROS_DEBUG("MCU frame parsed: game_progress=%u, self_hp=%u, bullet=%u, friendly_score=%d, enemy_score=%d",
                 frame.data.game_progress, frame.data.self_hp, frame.data.bullet_remain, 
                 friendly_score_, enemy_score_);
    }
    //分数计算部分
    void updateScore(const HKGameData& data)
    {
        // 只有在游戏开始(game_progress == 4)时才计算分数
        // 否则分数保持在200不变
        if (data.game_progress != 4)
        {
            friendly_score_ = 200;
            enemy_score_ = 200;
            // 重置时间戳，为下一次游戏开始做准备
            last_score_update_time_ = ros::Time(0);
            last_occupy_status_ = 0;
            last_red_dead_ = 0;
            last_blue_dead_ = 0;
            return;
        }
        
        ros::Time current_time = ros::Time::now();
        
        // 初始化时间戳
        if (last_score_update_time_.isZero())
        {
            last_score_update_time_ = current_time;
        }
        
        // 检测占领状态变化 - 每秒扣1分
        if (data.occupy_status != last_occupy_status_)
        {
        
            last_occupy_status_ = data.occupy_status;
            last_score_update_time_ = current_time;
        }
        else if ((current_time - last_score_update_time_).toSec() >= 1.0)
        {
            // occupy_status == 2: 对方占领 → 己方扣1分
            if (data.occupy_status == 2)
            {
                friendly_score_ = std::max(0, friendly_score_ - 1);
            }
            // occupy_status == 1: 己方占领 → 对方扣1分
            else if (data.occupy_status == 1)
            {
                enemy_score_ = std::max(0, enemy_score_ - 1);
            }
            last_score_update_time_ = current_time;
        }
        
        // 检测红方死亡状态变化 (使用red_dead_bits而不是red_dead)
        if (data.red_dead_bits != last_red_dead_)
        {
            // 暂时禁用：死亡事件导致分数变化
            // uint16_t new_deaths = data.red_dead_bits - last_red_dead_;
            // for (uint16_t i = 0; i < new_deaths; i++)
            // {
            //     if (robot_color_ == 0)  // 己方是红色
            //     {
            //         // 己方被击杀，自己人扣20分
            //         friendly_score_ = std::max(0, friendly_score_ - 20);
            //         ROS_INFO("Red robot killed (friendly): friendly_score now %d", friendly_score_);
            //     }
            //     else  // 己方是蓝色
            //     {
            //         // 击杀对方，敌方扣20分
            //         enemy_score_ = std::max(0, enemy_score_ - 20);
            //         ROS_INFO("Red robot killed (enemy): enemy_score now %d", enemy_score_);
            //     }
            // }
            last_red_dead_ = data.red_dead_bits;
        }
        
        // 检测蓝方死亡状态变化 (使用blue_dead_bits而不是blue_dead)
        if (data.blue_dead_bits != last_blue_dead_)
        {
            // 暂时禁用：死亡事件导致分数变化
            // uint16_t new_deaths = data.blue_dead_bits - last_blue_dead_;
            // for (uint16_t i = 0; i < new_deaths; i++)
            // {
            //     if (robot_color_ == 1)  // 己方是蓝色
            //     {
            //         // 己方被击杀，自己人扣20分
            //         friendly_score_ = std::max(0, friendly_score_ - 20);
            //         ROS_INFO("Blue robot killed (friendly): friendly_score now %d", friendly_score_);
            //     }
            //     else  // 己方是红色
            //     {
            //         // 击杀对方，敌方扣20分
            //         enemy_score_ = std::max(0, enemy_score_ - 20);
            //         ROS_INFO("Blue robot killed (enemy): enemy_score now %d", enemy_score_);
            //     }
            // }
            last_blue_dead_ = data.blue_dead_bits;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mcu_communicator");
    
    try
    {
        MCUCommunicator comm;
        ROS_INFO("MCU Communicator node started");
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Fatal error: %s", e.what());
        return 1;
    }
    
    return 0;
}
