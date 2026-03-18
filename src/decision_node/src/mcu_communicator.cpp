#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <decision_node/mcu_communicator.hpp>
#include <thread>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstring>

// ===== 调试开关 =====
// #define DEBUG_CAN_TX
// #define DEBUG_CAN_RX

class MCUCANCommunicator
{
public:
    MCUCANCommunicator() : nh_("~"), can_socket_(-1)
    {
        // 读取参数
        nh_.param("can_interface", can_interface_, std::string("can0"));
        
        // 读取导航发布频率 (默认50Hz)
        double nav_frequency = 50.0;
        nh_.param("nav_frequency", nav_frequency, 50.0);
        double nav_period = 1.0 / nav_frequency;
        
        // ===== 发布C板信息 =====
        pub_yaw_angle_ = nh_.advertise<std_msgs::Float32>("/mcu/yaw_angle", 1);
        pub_chassis_imu_ = nh_.advertise<std_msgs::Float32>("/mcu/chassis_imu", 1);
        
        // ===== 订阅输入 =====
        sub_cmd_vel_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,
                                                          &MCUCANCommunicator::cmdVelCallback, this);
        // 姿态指令
        // sub_motion_ = nh_.subscribe<std_msgs::UInt8>("/motion", 1, 
        //                                                      &MCUCANCommunicator::motionCallback, this);
        // sub_recover_ = nh_.subscribe<std_msgs::UInt8>("/recover", 1,
        //                                               &MCUCANCommunicator::recoverCallback, this);
        // sub_bullet_up_ = nh_.subscribe<std_msgs::UInt8>("/bullet_up", 1,
        //                                                 &MCUCANCommunicator::bulletUpCallback, this);
        // sub_bullet_num_ = nh_.subscribe<std_msgs::UInt8>("/bullet_num", 1,
        //                                                  &MCUCANCommunicator::bulletNumCallback, this);
        sub_dstar_status_ = nh_.subscribe<std_msgs::Bool>("/dstar_status", 1,
                                                         &MCUCANCommunicator::dstarStatusCallback, this);
        
        // 固定频率发送导航数据
        navigation_timer_ = nh_.createTimer(ros::Duration(nav_period),
                                            &MCUCANCommunicator::navigationTimerCallback, this);
        
        // 初始化CAN通讯
        if (!initCAN())
        {
            ROS_ERROR("Failed to initialize CAN interface: %s", can_interface_.c_str());
            return;
        }
        
        // 启动接收线程
        recv_thread_ = std::thread(&MCUCANCommunicator::receiveThread, this);
    }
    
    ~MCUCANCommunicator()
    {
        if (recv_thread_.joinable())
        {
            recv_thread_.join();
        }
        if (can_socket_ >= 0)
        {
            close(can_socket_);
        }
    }
    
private:
    ros::NodeHandle nh_;
    int can_socket_;
    std::string can_interface_;
    
    // ROS发布
    ros::Publisher pub_yaw_angle_;
    ros::Publisher pub_chassis_imu_;
    
    // ROS订
    ros::Subscriber sub_cmd_vel_;
    // 姿态指令订阅
    // ros::Subscriber sub_motion_;
    // ros::Subscriber sub_recover_;
    // ros::Subscriber sub_bullet_up_;
    // ros::Subscriber sub_bullet_num_;
    ros::Subscriber sub_dstar_status_;  
    ros::Timer navigation_timer_;
    
    // 接收线程
    std::thread recv_thread_;
    
    // ===== 发送命令时使用的状态变量 =====
    // 姿态命令状态
    // uint8_t current_motion_mode_ = 0;
    // uint8_t current_hp_up_ = 0;
    // uint8_t current_bullet_up_ = 0;
    // uint8_t current_bullet_num_ = 0;
    
    // 导航命令状态
    float current_nav_vx_ = 0.0f;
    float current_nav_vy_ = 0.0f;
    float current_nav_z_angle_ = 0.0f;
    uint8_t current_nav_arrived_ = 0;
    
    // ===== CAN 初始化 =====
    bool initCAN()
    {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            ROS_ERROR("Failed to create CAN socket");
            return false;
        }
        
        struct ifreq ifr;
        struct sockaddr_can addr;
        
        strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            ROS_ERROR("Failed to get CAN interface index for: %s", can_interface_.c_str());
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }
        
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            ROS_ERROR("Failed to bind CAN socket");
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }
        
        // 设置CAN套接字为非阻塞模式
        int flags = fcntl(can_socket_, F_GETFL);
        fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
        
        ROS_INFO("CAN interface initialized: %s", can_interface_.c_str());
        return true;
    }
    
    // ===== CRC8 计算 =====
    uint8_t calculateCRC8(const uint8_t* data, size_t length, uint8_t init = 0xFF)
    {
        uint8_t crc = init;
        for (size_t i = 0; i < length; i++)
        {
            crc = CRC8_TABLE[crc ^ data[i]];
        }
        return crc;
    }
    
    // ===== 回调函数 =====
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // 保存导航数据 
        current_nav_vx_ = msg->linear.x;      // X方向速度 [m/s]
        current_nav_vy_ = msg->linear.y;      // Y方向速度 [m/s]
        current_nav_z_angle_ = msg->angular.z; // 偏航角速度 [rad/s]
    }
    
    // 姿态回调函数
    // void motionCallback(const std_msgs::UInt8::ConstPtr& msg)
    // {
    //     current_motion_mode_ = msg->data;
    //     sendMotionCommand(msg->data);
    // }
    // 
    // void recoverCallback(const std_msgs::UInt8::ConstPtr& msg)
    // {
    //     current_hp_up_ = (msg->data != 0) ? 1 : 0;
    //     sendMotionCommand(current_motion_mode_);
    // }
    // 
    // void bulletUpCallback(const std_msgs::UInt8::ConstPtr& msg)
    // {
    //     current_bullet_up_ = (msg->data != 0) ? 1 : 0;
    //     sendMotionCommand(current_motion_mode_);
    // }
    // 
    // void bulletNumCallback(const std_msgs::UInt8::ConstPtr& msg)
    // {
    //     current_bullet_num_ = msg->data;
    //     sendMotionCommand(current_motion_mode_);
    // }
    
    // ===== 导航状态回调  =====
    void dstarStatusCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        current_nav_arrived_ = msg->data ? 1 : 0;
    }
    
    void navigationTimerCallback(const ros::TimerEvent& event)
    {
        sendNavigationCommand(current_nav_vx_, current_nav_vy_, current_nav_z_angle_);
    }
    
    // ===== 发送导航数据 (CAN ID: 0x113, 8字节) =====
    // NUC → C板：导航数据
    // 
    // 输入数据单位: m/s, rad/s
    // 输出数据单位: mm/s, mrad (用于 CAN 传输)
    // 
    // 数据格式：vx(2) + vy(2) + z_angle(2) + flags(1) + reserved(1) = 8字节
    // 编码规则：
    //   vx (m/s) × 1000 → int16 (mm/s 原始值)
    //   vy (m/s) × 1000 → int16 (mm/s 原始值) 
    //   z_angle (rad/s) × 1000 → int16 (mrad 原始值)
    //   flags: bit0 = arrived (是否到达目标点)
    void sendNavigationCommand(float vx, float vy, float z_angle)
    {
        if (can_socket_ < 0)
        {
            return;
        }
        
        // 构造CAN帧
        struct can_frame frame;
        frame.can_id = CAN_ID_NAVIGATION;
        frame.can_dlc = 8;
        //单位换算 m → mm
        CANNavigationFrame nav_data;
        nav_data.vx = (int16_t)(vx * 1000.0f);           // 线速度: m/s → mm/s
        nav_data.vy = (int16_t)(vy * 1000.0f);           // 线速度: m/s → mm/s
        nav_data.z_angle = (int16_t)(z_angle * 1000.0f); // 角速度: rad → mrad
        nav_data.flags = current_nav_arrived_;           // bit0: arrived flag
        nav_data.reserved = 0;
        
        // 复制数据到CAN帧
        memcpy(frame.data, &nav_data, 8);
        
        // 调试输出
        #ifdef DEBUG_CAN_TX
        ROS_INFO("[CAN TX 0x113] vx=%.3f m/s→%d mm/s, vy=%.3f m/s→%d mm/s, z_angle=%.3f rad→%d mrad, arrived=%u | "
                 "Raw: [%02X %02X %02X %02X %02X %02X %02X %02X]",
                 vx, nav_data.vx, vy, nav_data.vy, z_angle, nav_data.z_angle, current_nav_arrived_,
                 frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                 frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
        #endif
        
        if (write(can_socket_, &frame, sizeof(frame)) < 0)
        {
            ROS_ERROR("Failed to send CAN navigation frame (0x113)");
        }
    }
    
    // [CAN 0x114 - 注释] 发送姿态命令函数
    // // ===== 发送姿态命令 (CAN ID: 0x114, 5字节) =====
    // // NUC → C板：姿态指令 (stance, 买弹等)
    // // 数据格式：motion_mode(1) + hp_up(1) + bullet_up(1) + bullet_num(1) + reserved(1) = 5字节
    // void sendMotionCommand(uint8_t motion_mode)
    // {
    //     if (can_socket_ < 0)
    //     {
    //         return;
    //     }
    //     
    //     // 构造CAN帧
    //     struct can_frame frame;
    //     frame.can_id = CAN_ID_MOTION;
    //     frame.can_dlc = 5;
    //     
    //     // 构造姿态命令数据结构
    //     // 严格按照协议编码：
    //     // 字节0: motion_mode (运动模式)
    //     // 字节1: hp_up (回血标志)
    //     // 字节2: bullet_up (买弹标志)
    //     // 字节3: bullet_num (买弹数量)
    //     // 字节4: reserved = 0
    //     CANMotionCommandFrame motion_data;
    //     motion_data.motion_mode = motion_mode;
    //     motion_data.hp_up = current_hp_up_;
    //     motion_data.bullet_up = current_bullet_up_;
    //     motion_data.bullet_num = current_bullet_num_;
    //     motion_data.reserved = 0;
    //     
    //     // 复制数据到CAN帧
    //     memcpy(frame.data, &motion_data, 5);
    //     
    //     // 调试输出
    //     #ifdef DEBUG_CAN_TX
    //     ROS_INFO("[CAN TX 0x114] motion=%u, hp_up=%u, bullet_up=%u, bullet_num=%u | "
    //              "Raw: [%02X %02X %02X %02X %02X]",
    //              motion_mode, current_hp_up_, current_bullet_up_, current_bullet_num_,
    //              frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4]);
    //     #endif
    //     
    //     if (write(can_socket_, &frame, sizeof(frame)) < 0)
    //     {
    //         ROS_ERROR("Failed to send CAN motion command frame (0x114)");
    //     }
    // }
    
    // ===== 接收线程 =====
    void receiveThread()
    {
        ros::Rate loop_rate(100);  // 100Hz
        
        while (ros::ok())
        {
            if (can_socket_ < 0)
            {
                loop_rate.sleep();
                continue;
            }
            
            struct can_frame frame;
            int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
            
            if (nbytes > 0)
            {
                // 只处理C板数据 (CAN ID: 0x411)
                if (frame.can_id == CAN_ID_MCU_DATA)
                {
                    processCANDataFrame(&frame);
                }
            }
            
            loop_rate.sleep();
        }
    }
    
    // ===== 处理来自C板的CAN数据 (CAN ID: 0x411, 12字节) =====
    // 数据格式: SOF(1) + yaw_angle(4) + chassis_imu(4) + CRC8(1) + padding(2)
    void processCANDataFrame(struct can_frame* frame)
    {
        // 将原始CAN帧数据解析为数据结构
        CANDataFrameFromMCU mcu_data;
        memcpy(&mcu_data, frame->data, sizeof(CANDataFrameFromMCU));
        
        // ===== 数据验证：检查帧头 =====
        if (mcu_data.sof != 0x91)
        {
            ROS_WARN("[CAN RX 0x411] Invalid SOF: 0x%02X (expected 0x91)", mcu_data.sof);
            return;
        }
        
        // ===== 数据验证：检查CRC8 =====
        // CRC8计算范围：字节0-8（从sof开始，到chassis_imu结束，共9字节）
        uint8_t calculated_crc = calculateCRC8((uint8_t*)&mcu_data.sof, 9, 0xFF);
        if (calculated_crc != mcu_data.crc8)
        {
            ROS_WARN("[CAN RX 0x411] CRC8 mismatch: received=0x%02X, calculated=0x%02X",
                    mcu_data.crc8, calculated_crc);
            return;
        }
        
        // ===== 调试输出 =====
        #ifdef DEBUG_CAN_RX
        ROS_INFO("[CAN RX 0x411] yaw_angle=%.6f rad, chassis_imu=%.6f rad, crc8=0x%02X [OK] | "
                 "Raw: [%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]",
                 mcu_data.yaw_angle, mcu_data.chassis_imu, mcu_data.crc8,
                 frame->data[0], frame->data[1], frame->data[2], frame->data[3],
                 frame->data[4], frame->data[5], frame->data[6], frame->data[7],
                 frame->data[8], frame->data[9], frame->data[10], frame->data[11]);
        #endif
        
        // ===== 发布接收到的数据 =====
        std_msgs::Float32 msg_float;
        
        // 发布云台yaw角度
        msg_float.data = mcu_data.yaw_angle;
        pub_yaw_angle_.publish(msg_float);
        
        // 发布底盘IMU角度
        msg_float.data = mcu_data.chassis_imu;
        pub_chassis_imu_.publish(msg_float);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mcu_communicator");
    
    try
    {
        MCUCANCommunicator comm;
        ROS_INFO("MCU CAN Communicator node started successfully");
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Fatal error: %s", e.what());
        return 1;
    }
    
    return 0;
}
