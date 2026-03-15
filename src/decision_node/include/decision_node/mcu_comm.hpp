#ifndef MCU_COMM_HPP
#define MCU_COMM_HPP

#include <cstdint>
#include <cstring>

// CAN数据帧结构 - C板→NUC (CAN ID: 0x411)
// 帧长度：12字节（实际有效10字节）
struct CANDataFrame
{
    uint8_t  sof;             // data[0]: 0x91
    float    yaw_angle;       // data[1-4]: 云台yaw角度(弧度)
    float    chassis_imu;     // data[5-8]: 底盘IMU角度(弧度)
    uint8_t  crc8;            // data[9]: CRC8校验 (对 data[0-8] 的CRC8)
    uint8_t  padding[2];      // data[10-11]: padding
} __attribute__((packed));

static_assert(sizeof(CANDataFrame) == 12, "CANDataFrame must be exactly 12 bytes");


// NUC→C板的CAN数据结构 (CAN ID: 0x113) - 可选，用于发送控制命令
struct CANCommandFrame
{
    uint8_t  sof;             // data[0]: 0x92
    float    v;               // data[1-4]: 线速度
    float    w;               // data[5-8]: 角速度
    uint8_t  crc8;            // data[9]: CRC8校验
    uint8_t  padding[2];      // data[10-11]: padding
} __attribute__((packed));

static_assert(sizeof(CANCommandFrame) == 12, "CANCommandFrame must be exactly 12 bytes");

// CAN相关常量
#define CAN_DATA_FRAME_SOF 0x91        // CAN数据帧头
#define CAN_COMMAND_FRAME_SOF 0x92    // CAN命令帧头
#define CAN_DATA_FRAME_SIZE sizeof(CANDataFrame)
#define CAN_COMMAND_FRAME_SIZE sizeof(CANCommandFrame)
#define CAN_ID_RX 0x411               // 接收数据帧 ID (C板→NUC)
#define CAN_ID_TX 0x113               // 发送命令帧 ID (NUC→C板)

#endif // MCU_COMM_HPP
