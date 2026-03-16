#ifndef MCU_COMM_HPP
#define MCU_COMM_HPP

#include <cstdint>
#include <cstring>

// 下板→上板 (C板→NUC)
struct CANDataFrame
{
    float    yaw_angle;       // 云台yaw角度(弧度)
    float    chassis_imu;     // 底盘IMU角度(弧度)
} __attribute__((packed));

static_assert(sizeof(CANDataFrame) == 8, "CANDataFrame must be exactly 8 bytes");

// 上板→下板 (NUC→C板)
struct CANCommandFrame
{
    float    v;               // 线速度
    float    w;               // 角速度
} __attribute__((packed));

static_assert(sizeof(CANCommandFrame) == 8, "CANCommandFrame must be exactly 8 bytes");

// 相关常量
#define MCU_DATA_FRAME_SIZE sizeof(CANDataFrame)
#define MCU_COMMAND_FRAME_SIZE sizeof(CANCommandFrame)

#endif // MCU_COMM_HPP
