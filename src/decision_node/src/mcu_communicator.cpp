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
        pub_game_progress_ = nh_.advertise<std_msgs::UInt8>("/referee/game_progress", 1);
        pub_remain_hp_ = nh_.advertise<std_msgs::UInt8>("/referee/remain_hp", 1);
        pub_bullet_remain_ = nh_.advertise<std_msgs::UInt8>("/referee/bullet_remain", 1);
        pub_occupy_status_ = nh_.advertise<std_msgs::UInt8>("/referee/occupy_status", 1);
        
        // 新增发布者 - 导航和云台相关数据
        pub_yaw_angle_ = nh_.advertise<std_msgs::Float32>("/nav/yaw_angle", 1);
        pub_chassis_imu_ = nh_.advertise<std_msgs::Float32>("/nav/chassis_imu", 1);
        pub_operator_pos_ = nh_.advertise<geometry_msgs::Vector3>("/nav/operator_position", 1);
        pub_motion_mode_ = nh_.advertise<std_msgs::UInt8>("/nav/motion_mode", 1);
        

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
        
        // 上位机命令
        sub_motion_ = nh_.subscribe<std_msgs::UInt8>("motion", 1, 
                                                     &MCUCommunicator::motionCallback, this);
        sub_recover_ = nh_.subscribe<std_msgs::UInt8>("recover", 1,
                                                      &MCUCommunicator::recoverCallback, this);
        sub_bullet_up_ = nh_.subscribe<std_msgs::UInt8>("bullet_up", 1,
                                                        &MCUCommunicator::bulletUpCallback, this);
        
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
    
    ros::Publisher pub_friendly_score_;
    ros::Publisher pub_enemy_score_;
    
    // ROS 订阅者
    ros::Subscriber sub_motion_;
    ros::Subscriber sub_recover_;
    ros::Subscriber sub_bullet_up_;
    
    // 发送缓冲
    uint8_t tx_buffer_[256];
    size_t tx_buffer_index_;
    
    // 接收缓冲
    uint8_t frame_buffer_[MCU_FRAME_SIZE];
    size_t frame_buffer_index_;
    std::thread recv_thread_;
    
    // Motion回调函数
    // void motionCallback(const std_msgs::UInt8::ConstPtr& msg)
    // {
    //     sendMotionCommand(msg->data);
    // }
    
    // 存储当前的 hp_up 和 bullet_up 状态
    uint8_t current_hp_up_ = 0;
    uint8_t current_bullet_up_ = 0;
    uint8_t current_motion_mode_ = 0;
    
    // 分数追踪变量
    int32_t friendly_score_ = 200;  // 己方初始分数
    int32_t enemy_score_ = 200;     // 敌方初始分数
    ros::Time last_score_update_time_;  // 上次更新分数的时间
    int last_occupy_status_ = 0;    // 上一帧的占领状态
    uint16_t last_red_dead_ = 0;    // 上一帧红方死亡状态
    uint16_t last_blue_dead_ = 0;   // 上一帧蓝方死亡状态
    uint8_t robot_color_ = 0;       // 0=red, 1=blue（稍后从MCU数据更新）
    
    // Motion回调函数 - 根据 motion 值和当前状态发送命令帧
    void motionCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        sendMotionCommand(msg->data);
    }
    
    // Recover（回血）回调函数
    void recoverCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_hp_up_ = (msg->data != 0) ? 1 : 0;
        sendMotionCommand(current_motion_mode_);
    }
    
    // Bullet（买弹）回调函数
    void bulletUpCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        current_bullet_up_ = (msg->data != 0) ? 1 : 0;
        sendMotionCommand(current_motion_mode_);
    }
    
    // 发送Motion命令到下位机
    void sendMotionCommand(uint8_t motion_mode)
    {
        current_motion_mode_ = motion_mode;
        
        // 构建Motion命令帧
        MotionCommandFrame frame;
        frame.sof = 0x92;              // 0x92
        frame.motion_mode_up = motion_mode;
        frame.hp_up = current_hp_up_;
        frame.bullet_up = current_bullet_up_;
        frame.eof = 0xFE;             // 0xFE
        
        // CRC8校验 (包含 sof, motion_mode_up, hp_up, bullet_up)
        uint8_t crc_data[4] = {frame.sof, frame.motion_mode_up, frame.hp_up, frame.bullet_up};
        frame.crc8 = calculateCRC8(crc_data, 4);
        
        try
        {
            if (serial_.isOpen())
            {
                serial_.write((uint8_t*)&frame, sizeof(frame));
                ROS_DEBUG("Motion command sent: motion_mode=%u, hp_up=%u, bullet_up=%u (frame size=%zu)", 
                         motion_mode, current_hp_up_, current_bullet_up_, sizeof(frame));
            }
            else
            {
                ROS_WARN("Serial port is not open, cannot send motion command");
            }
        }
        catch (const serial::SerialException& e)
        {
            ROS_ERROR("Failed to send motion command: %s", e.what());
        }
    }
    
    // CRC8校验函数
    uint8_t calculateCRC8(const uint8_t* data, size_t length)
    {
        uint8_t crc = 0xFF;
        for (size_t i = 0; i < length; i++)
        {
            crc ^= data[i];
            for (int j = 0; j < 8; j++)
            {
                if (crc & 0x80)
                {
                    crc = (crc << 1) ^ 0x07;
                }
                else
                {
                    crc = (crc << 1);
                }
            }
        }
        return crc;
    }
    
    // CRC8验证函数 - 用于接收数据
    bool verifyCRC8(MCUDataFrame* frame)
    {
        // 计算除了CRC和EOF之外的所有数据的CRC值
        // frame结构体中CRC8的位置需要知道
        // 假设MCUDataFrame结构体中CRC8在最后一个字段之前
        
        // 获取frame中存储的CRC值（假设CRC8字段在EOF之前）
        uint8_t received_crc = frame->crc8;
        
        // 重新计算CRC（计算SOF之后到CRC之前的所有字段）
        // 这里需要知道MCUDataFrame的具体结构
        // 通常是从SOF开始到EOF之前的所有数据
        uint8_t calculated_crc = calculateCRC8((uint8_t*)&frame->sof, MCU_FRAME_SIZE - 2);  // 减去CRC8和EOF
        
        if (received_crc != calculated_crc)
        {
            ROS_DEBUG("CRC mismatch: received=0x%02X, calculated=0x%02X", received_crc, calculated_crc);
            return false;
        }
        return true;
    }
    
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
        
        // 验证CRC8校验
        if (!verifyCRC8(&frame))
        {
            ROS_WARN("CRC8 verification failed for received frame");
            return;
        }
        
        // 更新机器人颜色（0=red, 1=blue）
        robot_color_ = frame.robot_color;
        
        // 发布已有topic的数据
        std_msgs::UInt8 msg_uint8;
        std_msgs::UInt16 msg_uint16;
        std_msgs::UInt8 msg_uint8;
        std_msgs::Float32 msg_float;
        
        // 比赛进度
        msg_uint8.data = frame.game_progress;
        pub_game_progress_.publish(msg_uint8);
        // 自身血量
        msg_uint8.data = frame.self_hp;
        pub_remain_hp_.publish(msg_uint8);
        // 剩余弹量
        msg_uint8.data = frame.bullet_remain;
        pub_bullet_remain_.publish(msg_uint8);
        // 占领状态
        msg_uint8.data = frame.occupy_status;
        pub_occupy_status_.publish(msg_uint8);
        
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
        
        // 更新分数
        updateScore(frame);
        
        // 发布分数
        std_msgs::Int32 msg_score;
        msg_score.data = friendly_score_;
        pub_friendly_score_.publish(msg_score);
        msg_score.data = enemy_score_;
        pub_enemy_score_.publish(msg_score);
        
        ROS_DEBUG("MCU frame parsed: game_progress=%u, self_hp=%u, yaw=%.2f, chassis_imu=%.2f, friendly_score=%d, enemy_score=%d",
                 frame.game_progress, frame.self_hp, frame.yaw_angle, frame.chassis_imu, friendly_score_, enemy_score_);
    }
    
    void updateScore(const MCUDataFrame& frame)
    {
        ros::Time current_time = ros::Time::now();
        
        // 初始化时间戳
        if (last_score_update_time_.isZero())
        {
            last_score_update_time_ = current_time;
        }
        
        // 检测占领状态变化 - 每秒扣1分
        if (frame.occupy_status != last_occupy_status_)
        {
            last_occupy_status_ = frame.occupy_status;
            last_score_update_time_ = current_time;
        }
        else if ((current_time - last_score_update_time_).toSec() >= 1.0)
        {
            // occupy_status == 2: 对方占领 → 己方扣1分
            if (frame.occupy_status == 2)
            {
                friendly_score_ = std::max(0, friendly_score_ - 1);
            }
            // occupy_status == 1: 己方占领 → 对方扣1分
            else if (frame.occupy_status == 1)
            {
                enemy_score_ = std::max(0, enemy_score_ - 1);
            }
            last_score_update_time_ = current_time;
        }
        
        // 检测红方死亡状态变化
        if (frame.red_dead != last_red_dead_)
        {
            uint16_t new_deaths = frame.red_dead - last_red_dead_;
            for (uint16_t i = 0; i < new_deaths; i++)
            {
                if (robot_color_ == 0)  // 己方是红色
                {
                    // 己方被击杀，自己人扣20分
                    friendly_score_ = std::max(0, friendly_score_ - 20);
                    ROS_INFO("Red robot killed (friendly): friendly_score now %d", friendly_score_);
                }
                else  // 己方是蓝色
                {
                    // 击杀对方，敌方扣20分
                    enemy_score_ = std::max(0, enemy_score_ - 20);
                    ROS_INFO("Red robot killed (enemy): enemy_score now %d", enemy_score_);
                }
            }
            last_red_dead_ = frame.red_dead;
        }
        
        // 检测蓝方死亡状态变化
        if (frame.blue_dead != last_blue_dead_)
        {
            uint16_t new_deaths = frame.blue_dead - last_blue_dead_;
            for (uint16_t i = 0; i < new_deaths; i++)
            {
                if (robot_color_ == 1)  // 己方是蓝色
                {
                    // 己方被击杀，自己人扣20分
                    friendly_score_ = std::max(0, friendly_score_ - 20);
                    ROS_INFO("Blue robot killed (friendly): friendly_score now %d", friendly_score_);
                }
                else  // 己方是红色
                {
                    // 击杀对方，敌方扣20分
                    enemy_score_ = std::max(0, enemy_score_ - 20);
                    ROS_INFO("Blue robot killed (enemy): enemy_score now %d", enemy_score_);
                }
            }
            last_blue_dead_ = frame.blue_dead;
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
