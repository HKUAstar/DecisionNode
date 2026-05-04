#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>

class CommandTester
{
public:
    CommandTester() : nh_(""), nav_frequency_(50), nav_enabled_(false), 
                      nav_vx_(0.0f), nav_vy_(0.0f)
    {
        // MCU 直出数据
        pub_yaw_angle_ = nh_.advertise<std_msgs::Float32>("/mcu/yaw_angle", 1);
        pub_chassis_imu_ = nh_.advertise<std_msgs::Float32>("/mcu/chassis_imu", 1);

        // 裁判系统数据
        pub_game_progress_ = nh_.advertise<std_msgs::UInt8>("/referee/game_progress", 1);
        pub_stage_remain_time_ = nh_.advertise<std_msgs::UInt16>("/referee/stage_remain_time", 1);
        pub_ally_base_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/ally_base_hp", 1);
        pub_enemy_base_hp_ = nh_.advertise<std_msgs::UInt16>("/referee/enemy_base_hp", 1);

        // 场地状态
        pub_central_ground_status_ = nh_.advertise<std_msgs::UInt8>("/referee/central_ground_status", 1);
        pub_trap_ground_status_ = nh_.advertise<std_msgs::UInt8>("/referee/trap_ground_status", 1);
        pub_fortress_status_ = nh_.advertise<std_msgs::UInt8>("/referee/fortress_status", 1);
        pub_outpost_status_ = nh_.advertise<std_msgs::UInt8>("/referee/outpost_status", 1);

        // 自身状态
        pub_robot_id_ = nh_.advertise<std_msgs::UInt8>("/robot/robot_id", 1);
        pub_self_hp_ = nh_.advertise<std_msgs::UInt16>("/robot/self_hp", 1);

        // 弹药物资
        pub_projectile_17mm_ = nh_.advertise<std_msgs::UInt16>("/referee/projectile_17mm", 1);
        pub_projectile_fortress_ = nh_.advertise<std_msgs::UInt16>("/referee/projectile_fortress", 1);
        pub_remaining_gold_ = nh_.advertise<std_msgs::UInt16>("/referee/remaining_gold", 1);

        // 哨兵特殊
        pub_accumulated_bullet_ = nh_.advertise<std_msgs::UInt16>("/referee/accumulated_bullet", 1);
        pub_can_exchange_respawn_ = nh_.advertise<std_msgs::Bool>("/referee/can_exchange_respawn", 1);
        pub_respawn_money_ = nh_.advertise<std_msgs::UInt16>("/referee/respawn_money", 1);

        pub_out_of_combat_ = nh_.advertise<std_msgs::Bool>("/referee/out_of_combat", 1);
        pub_projectile_allowance_ = nh_.advertise<std_msgs::UInt16>("/referee/projectile_allowance", 1);
        pub_power_rune_available_ = nh_.advertise<std_msgs::Bool>("/referee/power_rune_available", 1);

        // 敌方位置
        pub_enemy_hero_ = nh_.advertise<geometry_msgs::Point>("/enemy/hero_position", 1);
        pub_enemy_engineer_ = nh_.advertise<geometry_msgs::Point>("/enemy/engineer_position", 1);
        pub_enemy_standard_3_ = nh_.advertise<geometry_msgs::Point>("/enemy/standard_3_position", 1);
        pub_enemy_standard_4_ = nh_.advertise<geometry_msgs::Point>("/enemy/standard_4_position", 1);
        pub_enemy_sentry_ = nh_.advertise<geometry_msgs::Point>("/enemy/sentry_position", 1);

        // 雷达
        pub_suggested_target_ = nh_.advertise<std_msgs::UInt8>("/radar/suggested_target", 1);
        pub_radar_flags_ = nh_.advertise<std_msgs::UInt16>("/radar/radar_flags", 1);

        // 导航相关（供 mcu_communicator 订阅）
        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        pub_dstar_status_ = nh_.advertise<std_msgs::Bool>("/dstar_status", 1);

        // 导航定时发送线程
        nav_publish_thread_ = std::thread(&CommandTester::navigationPublishLoop, this);

        ROS_INFO("Command Tester initialized with all MCU topics");
    }

    ~CommandTester()
    {
        if (nav_publish_thread_.joinable())
            nav_publish_thread_.join();
    }

    void run()
    {
        std::string command;

        while (ros::ok())
        {
            printMenu();
            std::cout << "\nEnter command: ";
            std::getline(std::cin, command);

            if (command == "quit" || command == "q")
            {
                ROS_INFO("Exiting...");
                break;
            }

            if (command == "yaw_angle")          { handleFloat("/mcu/yaw_angle", pub_yaw_angle_); }
            else if (command == "chassis_imu")   { handleFloat("/mcu/chassis_imu", pub_chassis_imu_); }
            else if (command == "game_progress") { handleUInt8("/referee/game_progress", pub_game_progress_); }
            else if (command == "stage_time")    { handleUInt16("/referee/stage_remain_time", pub_stage_remain_time_); }
            else if (command == "ally_base")     { handleUInt16("/referee/ally_base_hp", pub_ally_base_hp_); }
            else if (command == "enemy_base")    { handleUInt16("/referee/enemy_base_hp", pub_enemy_base_hp_); }
            else if (command == "central")       { handleUInt8("/referee/central_ground_status", pub_central_ground_status_); }
            else if (command == "trap")          { handleUInt8("/referee/trap_ground_status", pub_trap_ground_status_); }
            else if (command == "fortress")      { handleUInt8("/referee/fortress_status", pub_fortress_status_); }
            else if (command == "outpost")       { handleUInt8("/referee/outpost_status", pub_outpost_status_); }
            else if (command == "robot_id")      { handleUInt8("/robot/robot_id", pub_robot_id_); }
            else if (command == "self_hp")       { handleUInt16("/robot/self_hp", pub_self_hp_); }
            else if (command == "proj_17mm")     { handleUInt16("/referee/projectile_17mm", pub_projectile_17mm_); }
            else if (command == "proj_fort")     { handleUInt16("/referee/projectile_fortress", pub_projectile_fortress_); }
            else if (command == "gold")          { handleUInt16("/referee/remaining_gold", pub_remaining_gold_); }
            else if (command == "acc_bullet")    { handleUInt16("/referee/accumulated_bullet", pub_accumulated_bullet_); }
            else if (command == "can_respawn")   { handleBool("/referee/can_exchange_respawn", pub_can_exchange_respawn_); }
            else if (command == "respawn_money") { handleUInt16("/referee/respawn_money", pub_respawn_money_); }
            else if (command == "out_combat")    { handleBool("/referee/out_of_combat", pub_out_of_combat_); }
            else if (command == "proj_allow")    { handleUInt16("/referee/projectile_allowance", pub_projectile_allowance_); }
            else if (command == "power_rune")    { handleBool("/referee/power_rune_available", pub_power_rune_available_); }
            else if (command == "enemy_hero")    { handleEnemyPos("/enemy/hero_position", pub_enemy_hero_); }
            else if (command == "enemy_eng")     { handleEnemyPos("/enemy/engineer_position", pub_enemy_engineer_); }
            else if (command == "enemy_std3")    { handleEnemyPos("/enemy/standard_3_position", pub_enemy_standard_3_); }
            else if (command == "enemy_std4")    { handleEnemyPos("/enemy/standard_4_position", pub_enemy_standard_4_); }
            else if (command == "enemy_sentry")  { handleEnemyPos("/enemy/sentry_position", pub_enemy_sentry_); }
            else if (command == "suggested")     { handleUInt8("/radar/suggested_target", pub_suggested_target_); }
            else if (command == "radar_flags")   { handleUInt16("/radar/radar_flags", pub_radar_flags_); }
            else if (command == "navigation")    { handleNavigation(); }
            else if (command == "dstar")         { handleDstar(); }
            else if (!command.empty())
            {
                std::cout << "Unknown command: " << command << std::endl;
            }

            ros::spinOnce();
            ros::Duration(0.05).sleep();
        }
    }

private:
    ros::NodeHandle nh_;

    // === 发布者 ===
    ros::Publisher pub_yaw_angle_;
    ros::Publisher pub_chassis_imu_;
    ros::Publisher pub_game_progress_;
    ros::Publisher pub_stage_remain_time_;
    ros::Publisher pub_ally_base_hp_;
    ros::Publisher pub_enemy_base_hp_;
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
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_dstar_status_;

    // === 导航定时发送 ===
    int nav_frequency_;
    bool nav_enabled_;
    float nav_vx_, nav_vy_;
    std::mutex nav_mutex_;
    std::thread nav_publish_thread_;

    // ============ 处理函数 ============

    void handleUInt8(const std::string& topic, ros::Publisher& pub)
    {
        int val;
        std::cout << "Enter value (0-255): ";
        if (std::cin >> val)
        {
            std::cin.ignore();
            if (val >= 0 && val <= 255)
            {
                std_msgs::UInt8 msg;
                msg.data = static_cast<uint8_t>(val);
                pub.publish(msg);
                ROS_INFO("Published %s = %d", topic.c_str(), val);
            }
            else { std::cout << "Out of range (0-255)" << std::endl; }
        }
        else { std::cin.clear(); std::cin.ignore(10000, '\n'); }
    }

    void handleUInt16(const std::string& topic, ros::Publisher& pub)
    {
        int val;
        std::cout << "Enter value (0-65535): ";
        if (std::cin >> val)
        {
            std::cin.ignore();
            if (val >= 0 && val <= 65535)
            {
                std_msgs::UInt16 msg;
                msg.data = static_cast<uint16_t>(val);
                pub.publish(msg);
                ROS_INFO("Published %s = %d", topic.c_str(), val);
            }
            else { std::cout << "Out of range (0-65535)" << std::endl; }
        }
        else { std::cin.clear(); std::cin.ignore(10000, '\n'); }
    }

    void handleFloat(const std::string& topic, ros::Publisher& pub)
    {
        float val;
        std::cout << "Enter float value: ";
        if (std::cin >> val)
        {
            std::cin.ignore();
            std_msgs::Float32 msg;
            msg.data = val;
            pub.publish(msg);
            ROS_INFO("Published %s = %.4f", topic.c_str(), val);
        }
        else { std::cin.clear(); std::cin.ignore(10000, '\n'); }
    }

    void handleBool(const std::string& topic, ros::Publisher& pub)
    {
        int val;
        std::cout << "Enter 0 or 1: ";
        if (std::cin >> val)
        {
            std::cin.ignore();
            std_msgs::Bool msg;
            msg.data = (val != 0);
            pub.publish(msg);
            ROS_INFO("Published %s = %s", topic.c_str(), msg.data ? "true" : "false");
        }
        else { std::cin.clear(); std::cin.ignore(10000, '\n'); }
    }

    void handleEnemyPos(const std::string& topic, ros::Publisher& pub)
    {
        float x, y;
        std::cout << "Enter x (m): ";
        if (!(std::cin >> x)) { std::cin.clear(); std::cin.ignore(10000, '\n'); return; }
        std::cout << "Enter y (m): ";
        if (!(std::cin >> y)) { std::cin.clear(); std::cin.ignore(10000, '\n'); return; }
        std::cin.ignore();

        geometry_msgs::Point msg;
        msg.x = x;
        msg.y = y;
        msg.z = 0.0f;
        pub.publish(msg);
        ROS_INFO("Published %s = (%.2f, %.2f)", topic.c_str(), x, y);
    }

    void handleNavigation()
    {
        float vx, vy;
        std::cout << "Enter vx (m/s): ";
        if (!(std::cin >> vx)) { std::cin.clear(); std::cin.ignore(10000, '\n'); return; }
        std::cout << "Enter vy (m/s): ";
        if (!(std::cin >> vy)) { std::cin.clear(); std::cin.ignore(10000, '\n'); return; }
        std::cin.ignore();

        {
            std::lock_guard<std::mutex> lock(nav_mutex_);
            nav_vx_ = vx;
            nav_vy_ = vy;
            nav_enabled_ = true;
        }
        ROS_INFO("Navigation set: vx=%.4f, vy=%.4f, publishing to /cmd_vel", vx, vy);
    }

    void handleDstar()
    {
        int val;
        std::cout << "Enter dstar_status (0=not arrived, 1=arrived): ";
        if (std::cin >> val)
        {
            std::cin.ignore();
            std_msgs::Bool msg;
            msg.data = (val != 0);
            pub_dstar_status_.publish(msg);
            ROS_INFO("Published /dstar_status = %s", msg.data ? "true (arrived)" : "false (not arrived)");
        }
        else { std::cin.clear(); std::cin.ignore(10000, '\n'); }
    }

    // ============ 导航定时线程 ============

    void navigationPublishLoop()
    {
        ros::Rate rate(nav_frequency_);
        ROS_INFO("Navigation publish thread running at %d Hz", nav_frequency_);

        while (ros::ok())
        {
            if (nav_enabled_)
            {
                float vx, vy;
                {
                    std::lock_guard<std::mutex> lock(nav_mutex_);
                    vx = nav_vx_;
                    vy = nav_vy_;
                }

                geometry_msgs::Twist msg;
                msg.linear.x = vx;
                msg.linear.y = vy;
                msg.angular.z = 0.0f;
                pub_cmd_vel_.publish(msg);

                static int cnt = 0;
                if (++cnt % 50 == 0)
                    ROS_DEBUG("cmd_vel: vx=%.4f, vy=%.4f", vx, vy);
            }
            rate.sleep();
        }
    }

    // ============ 菜单 ============

    void printMenu()
    {
        std::cout << "\n==================== Command Menu ====================" << std::endl;
        std::cout << " MCU Raw:" << std::endl;
        std::cout << "   yaw_angle / chassis_imu" << std::endl;
        std::cout << " Referee:" << std::endl;
        std::cout << "   game_progress / stage_time / ally_base / enemy_base" << std::endl;
        std::cout << "   proj_17mm / proj_fort / gold / acc_bullet" << std::endl;
        std::cout << "   can_respawn / respawn_money / out_combat / proj_allow / power_rune" << std::endl;
        std::cout << " Field:" << std::endl;
        std::cout << "   central / trap / fortress / outpost" << std::endl;
        std::cout << " Robot:" << std::endl;
        std::cout << "   robot_id / self_hp" << std::endl;
        std::cout << " Enemy:" << std::endl;
        std::cout << "   enemy_hero / enemy_eng / enemy_std3 / enemy_std4 / enemy_sentry" << std::endl;
        std::cout << " Radar:" << std::endl;
        std::cout << "   suggested / radar_flags" << std::endl;
        std::cout << " Nav:" << std::endl;
        std::cout << "   navigation (vx,vy -> /cmd_vel) / dstar" << std::endl;
        std::cout << "   quit" << std::endl;
        std::cout << "=======================================================" << std::endl;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "command_tester");

    try
    {
        CommandTester tester;
        tester.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Fatal error: %s", e.what());
        return 1;
    }

    return 0;
}
