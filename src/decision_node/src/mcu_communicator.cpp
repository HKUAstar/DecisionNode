#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <decision_node/mcu_comm.hpp>
#include <thread>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class MCUCommunicator
{
public:
    MCUCommunicator() : nh_("~"), serial_fd_(-1), serial_port_("/dev/ttyACM0"),
                        serial_baudrate_(115200)
    {
        // 读取参数
        nh_.param("serial_port", serial_port_, std::string("/dev/ttyACM0"));
        nh_.param("serial_baudrate", serial_baudrate_, 115200);
        
        // 初始化串口
        try
        {
            initSerial();
            ROS_INFO("Serial port initialized successfully: %s @ %d baud (FD: %d)", 
                     serial_port_.c_str(), serial_baudrate_, serial_fd_);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Failed to initialize serial port: %s", e.what());
        }
        
        // 创建发布者 - 发布chassis_imu和yaw_angle
        pub_chassis_imu_ = nh_.advertise<std_msgs::Float32>("/mcu/chassis_imu", 1);
        pub_yaw_angle_ = nh_.advertise<std_msgs::Float32>("/mcu/yaw_angle", 1);
        
        // 创建订阅者 - 订阅速度命令
        sub_cmd_vel_ = nh_.subscribe<geometry_msgs::Twist>("/new_cmd_vel", 1,
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
        closeSerial();
    }
    
private:
    ros::NodeHandle nh_;
    int serial_fd_;
    std::string serial_port_;
    int serial_baudrate_;
    
    // ROS 发布者
    ros::Publisher pub_chassis_imu_;
    ros::Publisher pub_yaw_angle_;
    
    // ROS 订阅者
    ros::Subscriber sub_cmd_vel_;
    
    std::thread recv_thread_;
    
    // 初始化串口
    void initSerial()
    {
        // 打开串口设备
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0)
        {
            throw std::runtime_error("Failed to open serial port: " + serial_port_);
        }
        
        // 配置串口参数
        struct termios options;
        tcgetattr(serial_fd_, &options);
        
        // 设置波特率
        speed_t baud;
        switch (serial_baudrate_)
        {
            case 9600:   baud = B9600;   break;
            case 19200:  baud = B19200;  break;
            case 38400:  baud = B38400;  break;
            case 57600:  baud = B57600;  break;
            case 115200: baud = B115200; break;
            default:     baud = B115200; break;
        }
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        
        // 8N1: 8数据位, 无奇偶校验, 1停止位
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        
        // 禁用流控
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= CREAD | CLOCAL;
        
        // 原始模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        
        // 设置读取超时
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;
        
        tcsetattr(serial_fd_, TCSANOW, &options);
        tcflush(serial_fd_, TCIOFLUSH);
        
        ROS_INFO("Serial port configured: %d baud, 8N1", serial_baudrate_);
    }
    
    void closeSerial()
    {
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }
    
    // 速度命令回调 - 发送v和w到C板
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        float v = msg->linear.x ;
        float w = msg->angular.z;
        
        sendCommand(v, w);
        
        ROS_DEBUG("CmdVel received: linear.x=%.4f, linear.y=%.4f, angular.z=%.4f, v=%.4f, w=%.4f",
                  msg->linear.x, msg->linear.y, msg->angular.z, v, w);
    }
    
    // 发送命令到C板 (NUC→C板)
    void sendCommand(float v, float w)
    {
        CANCommandFrame cmd_frame;
        cmd_frame.v = v;
        cmd_frame.w = w;
        
        if (write(serial_fd_, (uint8_t*)&cmd_frame, MCU_COMMAND_FRAME_SIZE) < 0)
        {
            ROS_ERROR("Failed to send serial data: %s", strerror(errno));
            return;
        }
        
        ROS_DEBUG("Command sent: v=%.4f, w=%.4f", v, w);
    }
    
    // 接收线程
    void receiveThread()
    {
        ros::Rate loop_rate(100);  // 100Hz
        uint8_t buffer[256];
        
        while (ros::ok())
        {
            if (serial_fd_ < 0)
            {
                // 尝试重新连接
                try
                {
                    initSerial();
                    ROS_INFO("Reconnected to serial port");
                }
                catch (const std::exception& e)
                {
                    ROS_WARN("Failed to reconnect serial port: %s", e.what());
                    loop_rate.sleep();
                    continue;
                }
            }
            
            try
            {
                // 接收数据
                ssize_t nbytes = read(serial_fd_, buffer, sizeof(buffer));
                
                if (nbytes < 0)
                {
                    if (errno != EAGAIN && errno != EWOULDBLOCK)
                    {
                        ROS_ERROR("Serial read error: %s", strerror(errno));
                        closeSerial();
                    }
                }
                else if (nbytes > 0 && nbytes >= (ssize_t)MCU_DATA_FRAME_SIZE)
                {
                    // 处理接收到的数据
                    processData(buffer, nbytes);
                }
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("Serial receive exception: %s", e.what());
                closeSerial();
            }
            
            loop_rate.sleep();
        }
    }
    
    // 处理接收到的数据
    void processData(const uint8_t* data, size_t len)
    {
        if (len < MCU_DATA_FRAME_SIZE)
        {
            ROS_WARN("Data frame size mismatch: expected %zu, got %zu", MCU_DATA_FRAME_SIZE, len);
            return;
        }
        
        // 复制数据到结构体
        CANDataFrame frame;
        memcpy(&frame, data, MCU_DATA_FRAME_SIZE);
        
        ROS_DEBUG("Data received: chassis_imu=%.4f rad, yaw_angle=%.4f rad",
                 frame.chassis_imu, frame.yaw_angle);
        
        // 发布数据
        std_msgs::Float32 msg_float;
        
        msg_float.data = frame.chassis_imu;
        pub_chassis_imu_.publish(msg_float);
        
        msg_float.data = frame.yaw_angle;
        pub_yaw_angle_.publish(msg_float);
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
