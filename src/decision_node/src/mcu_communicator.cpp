#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
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
                       tx_buffer_index_(0), frame_buffer_index_(0)
    {
        // 读取参数
        nh_.param("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
        nh_.param("baudrate", serial_baudrate_, 921600);
        
        // 读取导航发布频率 (默认50Hz)
        double nav_frequency = 50.0;
        nh_.param("nav_frequency", nav_frequency, 50.0);
        double nav_period = 1.0 / nav_frequency;  // 转换为周期(秒)
        
        pub_yaw_angle_ = nh_.advertise<std_msgs::Float32>("/mcu/yaw_angle", 1);
        pub_chassis_imu_ = nh_.advertise<std_msgs::Float32>("/mcu/chassis_imu", 1);

        pub_game_progress_ = nh_.advertise<std_msgs::UInt8>("/referee/game_progress", 1);
        pub_stage_remain_time_ = nh_.advertise<std_msgs::UInt16>("/referee/stage_remain_time", 1);
        
        pub_ally_base_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/ally_base_hp", 1);
        
        pub_central_ground_status_ = nh_.advertise<std_msgs::UInt8>("/referee/central_ground_status", 1);
        pub_trap_ground_status_ = nh_.advertise<std_msgs::UInt8>("/referee/trap_ground_status", 1);
        pub_fortress_status_ = nh_.advertise<std_msgs::UInt8>("/referee/fortress_status", 1);
        pub_outpost_status_ = nh_.advertise<std_msgs::UInt8>("/referee/outpost_status", 1);

        pub_robot_id_ = nh_.advertise<std_msgs::UInt8>("/robot/robot_id", 1);
        pub_self_hp_ = nh_.advertise<std_msgs::UInt16>("/robot/self_hp", 1);
        
        pub_projectile_17mm_ = nh_.advertise<std_msgs::UInt16>("/referee/projectile_17mm", 1);
        pub_projectile_fortress_ = nh_.advertise<std_msgs::UInt16>("/referee/projectile_fortress", 1);
        pub_remaining_gold_ = nh_.advertise<std_msgs::UInt16>("/referee/remaining_gold", 1);

        pub_accumulated_bullet_ = nh_.advertise<std_msgs::UInt16>("/referee/accumulated_bullet", 1);
        pub_can_exchange_respawn_ = nh_.advertise<std_msgs::Bool>("/referee/can_exchange_respawn", 1);
        pub_respawn_money_ = nh_.advertise<std_msgs::UInt16>("/referee/respawn_money", 1);
        
        pub_out_of_combat_ = nh_.advertise<std_msgs::Bool>("/referee/out_of_combat", 1);
        pub_projectile_allowance_ = nh_.advertise<std_msgs::UInt16>("/referee/projectile_allowance", 1);
        pub_power_rune_available_ = nh_.advertise<std_msgs::Bool>("/referee/power_rune_available", 1);

        pub_enemy_hero_ = nh_.advertise<geometry_msgs::Point>("/enemy/hero_position", 1);
        pub_enemy_engineer_ = nh_.advertise<geometry_msgs::Point>("/enemy/engineer_position", 1);
        pub_enemy_standard_3_ = nh_.advertise<geometry_msgs::Point>("/enemy/standard_3_position", 1);
        pub_enemy_standard_4_ = nh_.advertise<geometry_msgs::Point>("/enemy/standard_4_position", 1);
        pub_enemy_sentry_ = nh_.advertise<geometry_msgs::Point>("/enemy/sentry_position", 1);
        pub_suggested_target_ = nh_.advertise<std_msgs::UInt8>("/radar/suggested_target", 1);
        pub_radar_flags_ = nh_.advertise<std_msgs::UInt16>("/radar/radar_flags", 1);
        
        pub_enemy_base_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/enemy_base_hp", 1);
        
    
        sub_dstar_status_ = nh_.subscribe<std_msgs::Bool>("/dstar_status", 1,
                                                         &MCUCommunicator::dstarStatusCallback, this);
        sub_cmd_vel_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,
                                                          &MCUCommunicator::cmdVelCallback, this);
        sub_motion_ = nh_.subscribe<std_msgs::UInt8>("/motion", 1,
                                                    &MCUCommunicator::motionCallback, this);
        sub_spin_ = nh_.subscribe<std_msgs::UInt8>("/spin", 1,
                                                  &MCUCommunicator::spinCallback, this);
        sub_target_yaw_ = nh_.subscribe<std_msgs::Float32>("/target_yaw", 1,
                                                          &MCUCommunicator::targetYawCallback, this);
        sub_activate_power_rune_ = nh_.subscribe<std_msgs::UInt8>("/activate_power_rune", 1,
                                                                  &MCUCommunicator::activatePowerRuneCallback, this);
        sub_exchange_respwan_ = nh_.subscribe<std_msgs::UInt8>("/exchange_respwan", 1,
                                                              &MCUCommunicator::exchangeRespwanCallback, this);
        
        // 创建导航命令定时器
        navigation_timer_ = nh_.createTimer(ros::Duration(nav_period),
                                            &MCUCommunicator::navigationTimerCallback, this);

        // 启动时先尝试打开一次串口；如果设备暂时不存在，不让节点退出
        configureSerial();
        if (!tryOpenSerial())
        {
            ROS_WARN("Serial port %s is unavailable at startup. Node will keep retrying in background.",
                     serial_port_.c_str());
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
    
    ros::Publisher pub_yaw_angle_;      
    ros::Publisher pub_chassis_imu_; 

    ros::Publisher pub_game_progress_;
    ros::Publisher pub_stage_remain_time_;
    
    ros::Publisher pub_ally_base_hp_;

    ros::Publisher pub_central_ground_status_;
    ros::Publisher pub_trap_ground_status_;
    ros::Publisher pub_fortress_status_;
    ros::Publisher pub_outpost_status_;

    ros::Publisher pub_robot_id_;
    ros::Publisher pub_self_hp_;

    ros::Publisher pub_projectile_17mm_;
    ros::Publisher pub_projectile_fortress_;
    ros::Publisher pub_remaining_gold_;

    ros::Publisher pub_accumulated_bullet_;
    ros::Publisher pub_can_exchange_respawn_;
    ros::Publisher pub_respawn_money_;

    ros::Publisher pub_out_of_combat_;
    ros::Publisher pub_projectile_allowance_;
    ros::Publisher pub_power_rune_available_;

    ros::Publisher pub_enemy_hero_;
    ros::Publisher pub_enemy_engineer_;
    ros::Publisher pub_enemy_standard_3_;
    ros::Publisher pub_enemy_standard_4_;
    ros::Publisher pub_enemy_sentry_;
    ros::Publisher pub_suggested_target_;
    ros::Publisher pub_radar_flags_;
    
    ros::Publisher pub_enemy_base_hp_;
    
    
    ros::Subscriber sub_dstar_status_;
    ros::Subscriber sub_cmd_vel_;
    
    ros::Subscriber sub_motion_;
    ros::Subscriber sub_spin_;
    ros::Subscriber sub_target_yaw_;
    ros::Subscriber sub_activate_power_rune_;
    ros::Subscriber sub_exchange_respwan_;
    
    ros::Timer navigation_timer_;
    
    // 发送缓冲
    uint8_t tx_buffer_[256];
    size_t tx_buffer_index_;
    
    // 接收缓冲
    uint8_t frame_buffer_[MCU_FRAME_SIZE];
    size_t frame_buffer_index_;
    std::thread recv_thread_;
    
    // 导航数据变量 - 原有
    float current_nav_vx_ = 0.0f;
    float current_nav_vy_ = 0.0f;
    uint8_t current_nav_arrived_ = 0;
    
    // ===== 导航数据变量 - 新增 =====
    uint8_t current_motion_ = 0;           // 0=比赛未开始; 1=进攻; 2=防御; 3=移动
    uint8_t current_spin_ = 0;             // 0=正常; 1=对齐角度; 2=上坡; 3=下坡
    float current_target_yaw_ = 0.0f;      // 目标角度
    uint8_t current_activate_power_rune_ = 0;  // 激活能量机关
    uint8_t current_exchange_respwan_ = 0;     // 兑换复活
    
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

    void configureSerial()
    {
        serial_.setPort(serial_port_);
        serial_.setBaudrate(serial_baudrate_);
        serial_.setBytesize(serial::eightbits);      // 数据位：8
        serial_.setParity(serial::parity_none);      // 校验位：None
        serial_.setStopbits(serial::stopbits_one);   // 停止位：1
        auto timeout = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(timeout);
    }

    bool tryOpenSerial()
    {
        if (serial_.isOpen())
        {
            return true;
        }

        try
        {
            configureSerial();
            serial_.open();

            if (serial_.isOpen())
            {
                ROS_INFO("MCU Serial port opened successfully: %s @ %d baud",
                         serial_port_.c_str(), serial_baudrate_);
                return true;
            }
        }
        catch (const serial::IOException& e)
        {
            ROS_WARN_THROTTLE(2.0, "Serial IO exception while opening %s: %s",
                              serial_port_.c_str(), e.what());
        }
        catch (const serial::SerialException& e)
        {
            ROS_WARN_THROTTLE(2.0, "Serial exception while opening %s: %s",
                              serial_port_.c_str(), e.what());
        }
        catch (const std::exception& e)
        {
            ROS_WARN_THROTTLE(2.0, "Unexpected exception while opening %s: %s",
                              serial_port_.c_str(), e.what());
        }

        return false;
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
        
        ROS_DEBUG("CmdVel received: vx=%.4f, vy=%.4f", 
                  current_nav_vx_, current_nav_vy_);
    }
    
    // ===== 新增 Callback 函数 =====
    // Motion 状态
    void motionCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_motion_ = msg->data;
        ROS_DEBUG("Motion received: %u", current_motion_);
    }
    
    // Spin 状态
    void spinCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_spin_ = msg->data;
        ROS_DEBUG("Spin received: %u", current_spin_);
    }
    
    // Target Yaw 角度
    void targetYawCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        current_target_yaw_ = msg->data;
        ROS_DEBUG("Target yaw received: %.4f rad", current_target_yaw_);
    }
    
    // 激活能量机关
    void activatePowerRuneCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_activate_power_rune_ = msg->data;
        ROS_DEBUG("Activate power rune received: %u", current_activate_power_rune_);
    }
    
    // 兑换复活
    void exchangeRespwanCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_exchange_respwan_ = msg->data;
        ROS_DEBUG("Exchange respwan received: %u", current_exchange_respwan_);
    }
    // ===== 新增 Callback 函数结束 =====
    
    // 导航命令定时器回调 - 固定频率发送NavigationFrame到下位机
    void navigationTimerCallback(const ros::TimerEvent& event)
    {
        sendNavigationCommand(current_nav_vx_, current_nav_vy_);
    }
    

    // 发送导航命令到下位机
    void sendNavigationCommand(float vx, float vy)
    {
        // 强制 vx/vy 置零
        if (current_spin_ == 1)
        {
            vx = 0.0f;
            vy = 0.0f;
        }

        current_nav_vx_ = vx;
        current_nav_vy_ = vy;
        
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
        // 单位转换：m/s → mm/s, clamp to int16 range
        int32_t vx_mm = (int32_t)(vx * 1000.0f);
        int32_t vy_mm = (int32_t)(vy * 1000.0f);
        vx_mm = std::max((int32_t)-32768, std::min((int32_t)32767, vx_mm));
        vy_mm = std::max((int32_t)-32768, std::min((int32_t)32767, vy_mm));
        frame.data.vx = (int16_t)vx_mm;            // mm/s
        frame.data.vy = (int16_t)vy_mm;            // mm/s
        frame.data.target_yaw = (int16_t)((current_target_yaw_ * 100.0f));  // 转换为 0.01 rad/s
        frame.data.motion = current_motion_;        // 0=比赛未开始；1=进攻；2=防御；3=移动
        frame.data.spin = current_spin_;            // 0=正常；1=对齐角度；2=上坡；3=下坡
        frame.data.activate_power_rune = current_activate_power_rune_;  // 激活能量机关
        frame.data.exchange_respwan = current_exchange_respwan_;        // 兑换复活
//test
        // ROS_INFO_THROTTLE(0.01,
        //                  "Send nav frame: at_place=%u, vx=%.4f m/s(%d mm/s), vy=%.4f m/s(%d mm/s), target_yaw=%.4f, motion=%u, spin=%u",
        //                  static_cast<unsigned int>(frame.data.at_place),
        //                  vx, frame.data.vx,
        //                  vy, frame.data.vy,
        //                  current_target_yaw_, current_motion_, current_spin_);
        
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
                ROS_WARN_THROTTLE(2.0, "Serial port %s is not open yet, skip sending navigation command.",
                                  serial_port_.c_str());
                return;
            }
            
            int written = serial_.write((uint8_t*)&frame, sizeof(frame));
            
            if (written == (int)sizeof(frame))
            {
                ROS_DEBUG("Navigation command sent: vx=%.4f m/s, vy=%.4f m/s", 
                         vx, vy);
            }
            else if (written > 0)
            {
                ROS_WARN("Partial write: expected %zu bytes, but only wrote %d bytes", 
                        sizeof(frame), written);
            }
            else
            {
                ROS_WARN("Write failed: write returned %d", written);
            }
        }
        catch (const serial::IOException& e)
        {
            ROS_WARN("Serial IO exception during navigation write: %s", e.what());
            if (serial_.isOpen())
            {
                serial_.close();
            }
        }
        catch (const serial::SerialException& e)
        {
            ROS_WARN("Serial exception during navigation write: %s", e.what());
            if (serial_.isOpen())
            {
                serial_.close();
            }
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
            
            // 添加详细调试信息以诊断结构体是否一致
            ROS_WARN("=== CRC16 Debug Info ===");
            ROS_WARN("Frame size: %zu bytes (HKFrameHeader=%zu, HKGameData=%zu, MCUDataFrame=%zu)",
                     sizeof(MCUDataFrame), sizeof(HKFrameHeader), sizeof(HKGameData), sizeof(MCUDataFrame));
            ROS_WARN("CRC计算范围: 头部(%d) + 数据(%d) = %d 字节",
                     HK_FRAME_HEADER_SIZE, HK_FRAME_DATA_SIZE, HK_FRAME_HEADER_SIZE + HK_FRAME_DATA_SIZE);
            ROS_WARN("帧头信息: SOF=[0x%02X,0x%02X], type=0x%02X, len=%u, seq=%u",
                     frame->header.sof[0], frame->header.sof[1], frame->header.packet_type, 
                     frame->header.length, frame->header.packet_seq);
            ROS_WARN("数据片段 (前16字节): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                     ((uint8_t*)frame)[0], ((uint8_t*)frame)[1], ((uint8_t*)frame)[2], ((uint8_t*)frame)[3],
                     ((uint8_t*)frame)[4], ((uint8_t*)frame)[5], ((uint8_t*)frame)[6], ((uint8_t*)frame)[7],
                     ((uint8_t*)frame)[8], ((uint8_t*)frame)[9], ((uint8_t*)frame)[10], ((uint8_t*)frame)[11],
                     ((uint8_t*)frame)[12], ((uint8_t*)frame)[13], ((uint8_t*)frame)[14], ((uint8_t*)frame)[15]);
            
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
        ros::Rate loop_rate(20);  // 后台持续重连/接收
        
        while (ros::ok())
        {
            if (!serial_.isOpen())
            {
                if (!tryOpenSerial())
                {
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
            catch (const serial::IOException& e)
            {
                ROS_WARN_THROTTLE(2.0, "Serial IO read exception: %s", e.what());
                if (serial_.isOpen())
                {
                    serial_.close();
                }
            }
            catch (const serial::SerialException& e)
            {
                ROS_WARN_THROTTLE(2.0, "Serial read exception: %s", e.what());
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
        
        // 添加调试信息：打印接收到的完整帧数据
        ROS_INFO("Received frame hex dump (first 40 bytes):");
        for (size_t i = 0; i < 40 && i < HK_FRAME_SIZE; i += 16)
        {
            char hex_str[100];
            int len = 0;
            for (size_t j = 0; j < 16 && i + j < 40; j++)
            {
                len += sprintf(hex_str + len, "%02X ", frame_buffer_[i + j]);
            }
            ROS_INFO("  [%02d-%02d]: %s", (int)i, (int)i + 15, hex_str);
        }
        
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
        ROS_INFO("CRC16 verification: received=0x%04X, will verify with header(%d) + data(%d) bytes",
                 frame.packet_crc16, HK_FRAME_HEADER_SIZE, HK_FRAME_DATA_SIZE);
        ROS_INFO("Frame tail bytes (last 6 bytes): %02X %02X %02X %02X %02X %02X",
                 frame_buffer_[HK_FRAME_SIZE-6], frame_buffer_[HK_FRAME_SIZE-5], frame_buffer_[HK_FRAME_SIZE-4],
                 frame_buffer_[HK_FRAME_SIZE-3], frame_buffer_[HK_FRAME_SIZE-2], frame_buffer_[HK_FRAME_SIZE-1]);
        
        if (!verifyCRC16(&frame))
        {
            ROS_WARN("CRC16 verification failed");
            return;
        }
        
        ROS_INFO("Valid HK frame received: game_progress=%u, self_hp=%u, crc16=0x%04X",
                 frame.data.game_progress, frame.data.current_HP, frame.packet_crc16);
        
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
        
        // 发布数据到各个Topic
        std_msgs::UInt8 msg_uint8;
        std_msgs::UInt16 msg_uint16;
        std_msgs::Bool msg_bool;
        std_msgs::Float32 msg_float;
        std_msgs::Int32 msg_int32;
        geometry_msgs::Point enemy_pos;

        
        // 云台yaw角
        msg_float.data = frame.data.yaw_angle;
        pub_yaw_angle_.publish(msg_float);
        
        // 底盘IMU角
        msg_float.data = frame.data.chassis_imu;
        pub_chassis_imu_.publish(msg_float);
        
        // 比赛状态
        msg_uint8.data = frame.data.game_progress;
        pub_game_progress_.publish(msg_uint8);
        
        // 机器人ID
        msg_uint8.data = frame.data.robot_id;
        pub_robot_id_.publish(msg_uint8);
        
        // 自身血量信息
        msg_uint16.data = frame.data.current_HP;
        pub_self_hp_.publish(msg_uint16);
        
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
        
        // ===== 新增Topic发布 - HKGameData新字段 =====
        msg_uint16.data = frame.data.stage_remain_time;
        pub_stage_remain_time_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.ally_base_HP;
        pub_ally_base_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.enemy_base_HP;
        pub_enemy_base_hp_.publish(msg_uint16);
        
        msg_uint8.data = frame.data.central_elevated_ground_status;
        pub_central_ground_status_.publish(msg_uint8);
        
        msg_uint8.data = frame.data.trapezoidal_elevated_ground_status;
        pub_trap_ground_status_.publish(msg_uint8);
        
        msg_uint8.data = frame.data.fortress_status;
        pub_fortress_status_.publish(msg_uint8);
        
        msg_uint8.data = frame.data.outpost_status;
        pub_outpost_status_.publish(msg_uint8);
        
        msg_uint16.data = frame.data.projectile_allowance_17mm;
        pub_projectile_17mm_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.projectile_allowance_fortress;
        pub_projectile_fortress_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.remaining_gold_coin;
        pub_remaining_gold_.publish(msg_uint16);
        
        msg_uint16.data = frame.data.accumulated_bullet_conversion;
        pub_accumulated_bullet_.publish(msg_uint16);
        
        msg_bool.data = frame.data.can_exchange_respawn;
        pub_can_exchange_respawn_.publish(msg_bool);
        
        msg_uint16.data = frame.data.respawn_money;
        pub_respawn_money_.publish(msg_uint16);
        
        msg_bool.data = frame.data.out_of_combat;
        pub_out_of_combat_.publish(msg_bool);
        
        msg_uint16.data = frame.data.projectile_allowance;
        pub_projectile_allowance_.publish(msg_uint16);
        
        msg_bool.data = frame.data.power_rune_available;
        pub_power_rune_available_.publish(msg_bool);
        
        ROS_DEBUG("MCU frame parsed: game_progress=%u, current_HP=%u",
                 frame.data.game_progress, frame.data.current_HP);
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
