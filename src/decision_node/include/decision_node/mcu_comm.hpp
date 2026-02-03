#ifndef MCU_COMM_HPP
#define MCU_COMM_HPP

#include <cstdint>
#include <cstring>

// 下位机数据帧结构 (共49字节)
struct MCUDataFrame
{
    uint8_t  sof;            // 0x91 (字节0)
    uint8_t  reserved_1_17[17];  // 预留 (字节1-17，包含 yaw_angle, chassis_imu, motion_mode, operator_x, operator_y)
    uint8_t  robot_id;       // 机器人ID (字节18)
    uint8_t  robot_color;    // 颜色 (字节19)
    uint8_t  game_progress;  // 比赛阶段 (0，1，2，3，5未开始，4比赛中)
    uint16_t red_1_hp;       // 红英雄 
    uint16_t red_3_hp;       // 红步兵3 
    uint16_t red_7_hp;       // 红哨兵 
    uint16_t blue_1_hp;      // 蓝英雄 
    uint16_t blue_3_hp;      // 蓝步兵3 
    uint16_t blue_7_hp;      // 蓝哨兵 
    uint16_t red_dead;       // 红方死亡位 (bit 0英雄(1号)死亡；bit 2步兵3号死亡；bit 4哨兵(7号)死亡)
    uint16_t blue_dead;      // 蓝方死亡位 
    uint16_t remain_hp;      // 自身血量 
    uint16_t max_hp;         // 最大血量 
    uint16_t bullet_remain;  // 剩余弹量 
    uint8_t  occupy_status;  // 占领状态 (0未占领； 1己方占领； 2对方占领； 3双方占领)
    uint8_t  crc8;           // CRC8 
    uint8_t  eof;            // 0xFE 
} __attribute__((packed));

static_assert(sizeof(MCUDataFrame) == 46, "MCUDataFrame must be exactly 46 bytes");

// 上位机发送的Motion命令帧结构 (共7字节)
struct MotionCommandFrame
{
    uint8_t  sof;              // 0x92 (字节0)
    uint8_t  motion_mode_up;   // 运动模式 (！！这里和实际要和裁判系统通讯的略有不同 0进攻； 1防御； 2移动； 3制动)
    uint8_t  hp_up;            // 0不回血 1回血 
    uint8_t  bullet_up;        // 0不买弹 1买弹 
    uint8_t  bullet_num;       // 买多少 
    uint8_t  crc8;             // CRC8 
    uint8_t  eof;              // 0xFE 
} __attribute__((packed));

static_assert(sizeof(MotionCommandFrame) == 7, "MotionCommandFrame must be exactly 7 bytes");

// 导航数据帧结构 (共17字节)
struct NavigationFrame
{
    uint8_t  sof;            // 0x93 (字节0)
    float    vx;             // (4字节)
    float    vy;             
    float    z_angle;        
    uint8_t  received;      
    uint8_t  arrived;       
    uint8_t  crc8;           // CRC8 
    uint8_t  eof;            // 0xFE
} __attribute__((packed));

static_assert(sizeof(NavigationFrame) == 17, "NavigationFrame must be exactly 17 bytes");

#define MCU_FRAME_SOF 0x91          // 下位机数据帧头
#define MOTION_FRAME_SOF 0x92       // 上位机motion命令帧头
#define NAVIGATION_FRAME_SOF 0x93   // 导航数据帧头
#define MCU_FRAME_EOF 0xFE
#define MCU_FRAME_SIZE 46
#define MOTION_FRAME_SIZE 7
#define NAVIGATION_FRAME_SIZE 17

#endif // MCU_COMM_HPP
