#ifndef MCU_COMM_HPP
#define MCU_COMM_HPP

#include <cstdint>
#include <cstring>

// 下位机数据帧结构 (共46字节)
struct MCUDataFrame
{
    uint8_t  sof;            // 0x91 (字节0)
    float    yaw_angle;      // 云台yaw弧度 (字节1-4)
    float    chassis_imu;    // 底盘IMU弧度 (字节5-8)
    uint8_t  motion_mode;    // 运动模式 (字节9)
    float    operator_x;     // 操作手x (字节10-13)
    float    operator_y;     // 操作手y (字节14-17)
    uint8_t  robot_id;       // 机器人ID (字节18)
    uint8_t  robot_color;    // 颜色 (字节19)
    uint8_t  game_progress;  // 比赛阶段 (字节20)
    uint16_t red_1_hp;       // 红英雄 (字节21-22)
    uint16_t red_3_hp;       // 红步兵3 (字节23-24)
    uint16_t red_7_hp;       // 红哨兵 (字节25-26)
    uint16_t blue_1_hp;      // 蓝英雄 (字节27-28)
    uint16_t blue_3_hp;      // 蓝步兵3 (字节29-30)
    uint16_t blue_7_hp;      // 蓝哨兵 (字节31-32)
    uint16_t red_dead;       // 红方死亡位 (字节33-34)
    uint16_t blue_dead;      // 蓝方死亡位 (字节35-36)
    uint16_t self_hp;        // 自身血量 (字节37-38)
    uint16_t self_max_hp;    // 最大血量 (字节39-40)
    uint16_t bullet_remain;  // 剩余弹量 (字节41-42)
    uint8_t  occupy_status;  // 占领状态 (字节43)
    uint8_t  crc8;           // CRC8 (字节44)
    uint8_t  eof;            // 0xFE (字节45)
} __attribute__((packed));

static_assert(sizeof(MCUDataFrame) == 46, "MCUDataFrame must be exactly 46 bytes");

// 上位机发送的Motion命令帧结构 (共6字节)
struct MotionCommandFrame
{
    uint8_t  sof;              // 0x92 (字节0)
    uint8_t  motion_mode_up;   // 运动模式 (字节1)
    uint8_t  hp_up;            // 0不回血 1回血 (字节2)
    uint8_t  bullet_up;        // 0不买弹 1买弹 (字节3)
    uint8_t  crc8;             // CRC8 (字节4)
    uint8_t  eof;              // 0xFE (字节5)
} __attribute__((packed));

static_assert(sizeof(MotionCommandFrame) == 6, "MotionCommandFrame must be exactly 6 bytes");

// 上位机发送的导航命令帧结构 (共17字节)
struct NavCommandFrame
{
    uint8_t  sof;           // 0x4A (字节0)
    float    x_velocity;    // x轴速度 (字节1-4)
    float    y_velocity;    // y轴速度 (字节5-8)
    float    omega;         // 导航希望地盘相对于上电位置的夹角 (字节9-12)
    uint8_t  received;      // 表示是否成功收到了板信息 (字节13)
    uint8_t  arrived;       // 表示是否到达目的地，0:在路上，1:到了 (字节14)
    uint8_t  crc8;          // CRC8 (字节15)
    uint8_t  eof;           // 0xFE (字节16)
} __attribute__((packed));

static_assert(sizeof(NavCommandFrame) == 17, "NavCommandFrame must be exactly 17 bytes");
#define MCU_FRAME_SOF 0x91          // 下位机数据帧头
#define MOTION_FRAME_SOF 0x92       // 上位机motion命令帧头
#define NAV_FRAME_SOF 0x4A          // 上位机导航命令帧头
#define MCU_FRAME_EOF 0xFE
#define MCU_FRAME_SIZE 46
#define MOTION_FRAME_SIZE 4
#define NAV_FRAME_SIZE 17

#endif // MCU_COMM_HPP
