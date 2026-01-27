#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3.h>
#include <serial/serial.h>
#include <decision_node/mcu_comm.hpp>
#include <thread>

class MCUCommunicator
{
public:
    MCUCommunicator() : nh_("~"), serial_port_(""), serial_baudrate_(115200), 
                       frame_buffer_index_(0)
    {
        // 读取参数
        nh_.param("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
        nh_.param("baudrate", serial_baudrate_, 115200);
        
        // 初始化发布者 - 对应已有的topic
        pub_game_progress_ = nh_.advertise<std_msgs::Int32>("/referee/game_progress", 1);
        pub_remain_hp_ = nh_.advertise<std_msgs::Int32>("/referee/remain_hp", 1);
        pub_bullet_remain_ = nh_.advertise<std_msgs::Int32>("/referee/bullet_remain", 1);
        pub_occupy_status_ = nh_.advertise<std_msgs::Int32>("/referee/occupy_status", 1);
        
        // 新增发布者 - 导航和云台相关数据
        pub_yaw_angle_ = nh_.advertise<std_msgs::Float32>("/nav/yaw_angle", 1);
        pub_chassis_imu_ = nh_.advertise<std_msgs::Float32>("/nav/chassis_imu", 1);
        pub_operator_pos_ = nh_.advertise<geometry_msgs::Vector3>("/nav/operator_position", 1);
        pub_motion_mode_ = nh_.advertise<std_msgs::UInt8>("/nav/motion_mode", 1);
        
        // 新增发布者 - 机器人状态
        pub_robot_id_ = nh_.advertise<std_msgs::UInt8>("/robot/robot_id", 1);
        pub_robot_color_ = nh_.advertise<std_msgs::UInt8>("/robot/robot_color", 1);
        pub_self_hp_ = nh_.advertise<std_msgs::UInt16>("/robot/self_hp", 1);
        pub_self_max_hp_ = nh_.advertise<std_msgs::UInt16>("/robot/self_max_hp", 1);
        
        // 新增发布者 - 其他机器人血量信息
        pub_red_1_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/red_1_hp", 1);
        pub_red_3_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/red_3_hp", 1);
        pub_red_7_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/red_7_hp", 1);
        pub_blue_1_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_1_hp", 1);
        pub_blue_3_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_3_hp", 1);
        pub_blue_7_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_7_hp", 1);
        
        // 新增发布者 - 死亡状态
        pub_red_dead_ = nh_.advertise<std_msgs::UInt16>("/referee/red_dead", 1);
        pub_blue_dead_ = nh_.advertise<std_msgs::UInt16>("/referee/blue_dead", 1);
        
        // 初始化串口
        try
        {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(serial_baudrate_);
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
            }
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Serial exception: %s", e.what());
        }
        
        // 启动接收线程
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
    
    ros::Publisher pub_yaw_angle_;
    ros::Publisher pub_chassis_imu_;
    ros::Publisher pub_operator_pos_;
    ros::Publisher pub_motion_mode_;
    
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
    
    // 接收缓冲
    uint8_t frame_buffer_[MCU_FRAME_SIZE];
    size_t frame_buffer_index_;
    std::thread recv_thread_;
    
    void receiveThread()
    {
        ros::Rate loop_rate(100);  // 100Hz
        
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
            // 寻找帧头
            if (frame_buffer_index_ == 0)
            {
                if (byte == MCU_FRAME_SOF)
                {
                    frame_buffer_[frame_buffer_index_++] = byte;
                }
                continue;
            }
            
            // 接收数据
            frame_buffer_[frame_buffer_index_++] = byte;
            
            // 检查是否接收完整帧
            if (frame_buffer_index_ == MCU_FRAME_SIZE)
            {
                // 验证帧尾
                if (frame_buffer_[MCU_FRAME_SIZE - 1] == MCU_FRAME_EOF)
                {
                    // 解析并发布数据
                    parseAndPublish();
                }
                else
                {
                    ROS_WARN("Invalid frame end marker: 0x%02X (expected 0xFE)", 
                            frame_buffer_[MCU_FRAME_SIZE - 1]);
                }
                
                frame_buffer_index_ = 0;
            }
        }
    }
    
    void parseAndPublish()
    {
        // 将原始字节数据复制到结构体
        MCUDataFrame frame;
        memcpy(&frame, frame_buffer_, MCU_FRAME_SIZE);
        
        // 验证帧头和帧尾
        if (frame.sof != MCU_FRAME_SOF || frame.eof != MCU_FRAME_EOF)
        {
            ROS_WARN("Invalid frame markers: SOF=0x%02X, EOF=0x%02X", frame.sof, frame.eof);
            return;
        }
        
        // 发布已有topic的数据
        std_msgs::Int32 msg_int;
        std_msgs::UInt16 msg_uint16;
        std_msgs::UInt8 msg_uint8;
        std_msgs::Float32 msg_float;
        
        // 比赛进度
        msg_int.data = frame.game_progress;
        pub_game_progress_.publish(msg_int);
        
        // 自身血量
        msg_int.data = frame.self_hp;
        pub_remain_hp_.publish(msg_int);
        
        // 剩余弹量
        msg_int.data = frame.bullet_remain;
        pub_bullet_remain_.publish(msg_int);
        
        // 占领状态
        msg_int.data = frame.occupy_status;
        pub_occupy_status_.publish(msg_int);
        
        // 发布导航数据
        msg_float.data = frame.yaw_angle;
        pub_yaw_angle_.publish(msg_float);
        
        msg_float.data = frame.chassis_imu;
        pub_chassis_imu_.publish(msg_float);
        
        geometry_msgs::Vector3 msg_pos;
        msg_pos.x = frame.operator_x;
        msg_pos.y = frame.operator_y;
        msg_pos.z = 0.0;
        pub_operator_pos_.publish(msg_pos);
        
        msg_uint8.data = frame.motion_mode;
        pub_motion_mode_.publish(msg_uint8);
        
        // 发布机器人信息
        msg_uint8.data = frame.robot_id;
        pub_robot_id_.publish(msg_uint8);
        
        msg_uint8.data = frame.robot_color;
        pub_robot_color_.publish(msg_uint8);
        
        msg_uint16.data = frame.self_hp;
        pub_self_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.self_max_hp;
        pub_self_max_hp_.publish(msg_uint16);
        
        // 发布其他机器人血量
        msg_uint16.data = frame.red_1_hp;
        pub_red_1_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.red_3_hp;
        pub_red_3_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.red_7_hp;
        pub_red_7_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.blue_1_hp;
        pub_blue_1_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.blue_3_hp;
        pub_blue_3_hp_.publish(msg_uint16);
        
        msg_uint16.data = frame.blue_7_hp;
        pub_blue_7_hp_.publish(msg_uint16);
        
        // 发布死亡状态
        msg_uint16.data = frame.red_dead;
        pub_red_dead_.publish(msg_uint16);
        
        msg_uint16.data = frame.blue_dead;
        pub_blue_dead_.publish(msg_uint16);
        
        ROS_DEBUG("MCU frame parsed: game_progress=%u, self_hp=%u, yaw=%.2f, chassis_imu=%.2f",
                 frame.game_progress, frame.self_hp, frame.yaw_angle, frame.chassis_imu);
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
