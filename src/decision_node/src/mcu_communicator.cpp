#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <decision_node/mcu_comm.hpp>
#include <thread>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

class MCUCommunicator
{
public:
    MCUCommunicator() : nh_("~"), can_socket_(-1), can_interface_("can0"),
                       frame_buffer_index_(0)
    {
        // 读取参数
        nh_.param("can_interface", can_interface_, std::string("can0"));
        
        // 初始化CAN socket
        try
        {
            initCANSocket();
            ROS_INFO("CAN interface initialized successfully: %s (Socket FD: %d)", 
                     can_interface_.c_str(), can_socket_);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Failed to initialize CAN socket: %s", e.what());
        }
        
        // 创建发布者 - 只发布yaw_angle和chassis_imu
        pub_yaw_angle_ = nh_.advertise<std_msgs::Float32>("/mcu/yaw_angle", 1);
        pub_chassis_imu_ = nh_.advertise<std_msgs::Float32>("/mcu/chassis_imu", 1);
        
        // 创建订阅者 - 订阅速度命令用于发送到C板
        sub_cmd_vel_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,
                                                          &MCUCommunicator::cmdVelCallback, this);
        
        // 启动接收线程
        recv_thread_ = std::thread(&MCUCommunicator::receiveThread, this);
    }
    
    ~MCUCommunicator()
    {
        if (recv_thread_.joinable())
        {
            recv_thread_.join();
        }
        closeCANSocket();
    }
    
private:
    ros::NodeHandle nh_;
    int can_socket_;
    std::string can_interface_;
    
    // ROS 发布者 - 只发布yaw_angle和chassis_imu
    ros::Publisher pub_yaw_angle_;     // 云台yaw弧度
    ros::Publisher pub_chassis_imu_;   // 底盘IMU弧度
    
    // ROS 订阅者
    ros::Subscriber sub_cmd_vel_;
    
    // 接收缓冲
    uint8_t frame_buffer_[CAN_DATA_FRAME_SIZE];
    size_t frame_buffer_index_;
    std::thread recv_thread_;
    
    // CRC8 查表 - 与 MCU 端完全相同（初始值 0xFF）
    static constexpr uint8_t CRC8_TABLE[256] = {
        0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
        0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
        0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
        0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
        0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
        0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
        0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
        0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
        0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
        0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
        0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
        0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
        0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
        0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
        0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
        0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
    };
    
    // CAN Socket初始化函数
    void initCANSocket()
    {
        // 创建CAN socket
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            throw std::runtime_error("Failed to create CAN socket");
        }
        
        // 获取网络接口索引
        struct ifreq ifr;
        strcpy(ifr.ifr_name, can_interface_.c_str());
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            close(can_socket_);
            can_socket_ = -1;
            throw std::runtime_error("Failed to get CAN interface index");
        }
        
        // 绑定socket到CAN接口
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            close(can_socket_);
            can_socket_ = -1;
            throw std::runtime_error("Failed to bind CAN socket");
        }
        
        // 设置socket为非阻塞模式（可选）
        // fcntl(can_socket_, F_SETFL, O_NONBLOCK);
        
        ROS_INFO("CAN socket initialized on %s", can_interface_.c_str());
    }
    
    void closeCANSocket()
    {
        if (can_socket_ >= 0)
        {
            close(can_socket_);
            can_socket_ = -1;
        }
    }
    
    // Cmd Vel: 订阅速度命令，发送到C板
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // 计算线速度和角速度，发送给C板
        float v = sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y);
        float theta_angle = atan2(msg->linear.y, msg->linear.x);
        float w = theta_angle;  // 或根据需要乘以系数K
        
        sendCANCommand(v, w);
        
        ROS_DEBUG("CmdVel received: linear.x=%.4f, linear.y=%.4f, angular.z=%.4f, v=%.4f, w=%.4f",
                  msg->linear.x, msg->linear.y, msg->angular.z, v, w);
    }
    
    // 发送CAN命令到C板 (NUC→C板, CAN ID: 0x113)
    void sendCANCommand(float v, float w)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        
        frame.can_id = CAN_ID_TX;  // 0x113
        frame.can_dlc = CAN_COMMAND_FRAME_SIZE;
        
        CANCommandFrame cmd_frame;
        memset(&cmd_frame, 0, sizeof(cmd_frame));  
        cmd_frame.sof = CAN_COMMAND_FRAME_SOF;  // 0x92
        cmd_frame.v = v;
        cmd_frame.w = w;
        // CRC8: 计算前9字节（sof到w的结尾）
        cmd_frame.crc8 = calculateCRC8((uint8_t*)&cmd_frame, 9, 0xFF);
        
        memcpy(frame.data, (uint8_t*)&cmd_frame, CAN_COMMAND_FRAME_SIZE);
        
        if (write(can_socket_, &frame, sizeof(frame)) < 0)
        {
            ROS_ERROR("Failed to send CAN frame: %s", strerror(errno));
            return;
        }
        
        ROS_DEBUG("CAN command sent (ID:0x%03X): v=%.4f, w=%.4f", frame.can_id, v, w);
    }
    
    // CRC8 查表实现
    uint8_t calculateCRC8(const uint8_t* pch_message, size_t dw_length, uint8_t ucCRC8 = 0xFF)
    {
        unsigned char uc_index;
        while (dw_length--)
        {
            uc_index = ucCRC8 ^ (*pch_message++);
            ucCRC8 = CRC8_TABLE[uc_index];
        }
        return ucCRC8;
    }
    
    // CRC8验证函数 - 验证CAN数据帧
    bool verifyCANCRC8(CANDataFrame* frame)
    {
        uint8_t received_crc = frame->crc8;
        // 计算CRC8：对 data[0-8]（即sof到chassis_imu结尾）
        uint8_t calculated_crc = calculateCRC8((uint8_t*)&frame->sof, 9, 0xFF);
        
        if (received_crc != calculated_crc)
        {
            ROS_WARN("CAN CRC8 mismatch: received=0x%02X, calculated=0x%02X", received_crc, calculated_crc);
            return false;
        }
        return true;
    }
    
    void receiveThread()
    {
        ros::Rate loop_rate(100);  // 100Hz
        struct can_frame frame;
        
        while (ros::ok())
        {
            if (can_socket_ < 0)
            {
                // 尝试重新连接
                try
                {
                    initCANSocket();
                    ROS_INFO("Reconnected to CAN interface");
                }
                catch (const std::exception& e)
                {
                    ROS_WARN("Failed to reconnect CAN: %s", e.what());
                    loop_rate.sleep();
                    continue;
                }
            }
            
            try
            {
                // 接收CAN消息
                ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
                
                if (nbytes < 0)
                {
                    if (errno != EAGAIN && errno != EWOULDBLOCK)
                    {
                        ROS_ERROR("CAN read error: %s", strerror(errno));
                        closeCANSocket();
                    }
                }
                else if (nbytes > 0)
                {
                    // 检查是否是我们期望的数据帧ID (0x411)
                    if (frame.can_id == CAN_ID_RX)
                    {
                        processCANData((uint8_t*)frame.data, frame.can_dlc);
                    }
                }
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("CAN receive exception: %s", e.what());
                closeCANSocket();
            }
            
            loop_rate.sleep();
        }
    }
    
    void processCANData(const uint8_t* data, size_t len)
    {
        if (len < CAN_DATA_FRAME_SIZE)
        {
            ROS_WARN("CAN frame size mismatch: expected %zu, got %zu", CAN_DATA_FRAME_SIZE, len);
            return;
        }
        
        // 将原始字节数据复制到结构体
        CANDataFrame frame;
        memcpy(&frame, data, CAN_DATA_FRAME_SIZE);
        
        // 验证帧头
        if (frame.sof != CAN_DATA_FRAME_SOF)
        {
            ROS_WARN("Invalid CAN frame SOF: 0x%02X (expected 0x%02X)", frame.sof, CAN_DATA_FRAME_SOF);
            return;
        }
        
        // 验证CRC8校验
        if (!verifyCANCRC8(&frame))
        {
            ROS_WARN("CAN CRC8 verification failed");
            return;
        }
        
        ROS_DEBUG("Valid CAN frame received: yaw_angle=%.4f rad, chassis_imu=%.4f rad",
                 frame.yaw_angle, frame.chassis_imu);
        
        // 发布数据
        std_msgs::Float32 msg_float;
        
        msg_float.data = frame.yaw_angle;
        pub_yaw_angle_.publish(msg_float);
        
        msg_float.data = frame.chassis_imu;
        pub_chassis_imu_.publish(msg_float);
    }
};

constexpr uint8_t MCUCommunicator::CRC8_TABLE[256];

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
