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

// 常量定义
#define MCU_FRAME_SOF 0x91
#define MCU_FRAME_EOF 0xFE
#define MCU_FRAME_SIZE 46

#endif // MCU_COMM_HPP
