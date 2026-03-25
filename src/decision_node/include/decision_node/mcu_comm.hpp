#ifndef MCU_COMM_HPP
#define MCU_COMM_HPP

#include <cstdint>
#include <cstring>

// ===== CRC16 查表 - DJI标准 =====
static constexpr uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
};

// HK协议帧头 - 接收自上板(NUC)的数据帧格式 (总长度78字节)
struct HKFrameHeader
{
    uint8_t sof[2];           // 0-1: 'H', 'K' (0x48, 0x4B)
    uint16_t length;          // 2-3: 整包长度 (Little-Endian)
    uint8_t packet_type;      // 4: 包类型 (0x01 = 比赛数据)
    uint8_t reserved;         // 5: 保留字段
    uint8_t packet_seq;       // 6: 包序号 (0-255循环)
    uint8_t reserved2;        // 7: 保留字段2
    uint8_t header_crc8;      // 8: 帧头CRC8校验
} __attribute__((packed));

// HK协议数据负载 - 比赛数据帧 (65字节)
struct HKGameData
{
    // game_state (4B)
    uint8_t game_progress;    // 比赛阶段
    uint8_t occupy_status;    // 占领状态
    uint8_t robot_id;         // 机器人ID
    uint8_t robot_color;      // 0=红方, 1=蓝方
    
    // hp_red (8B)
    uint16_t red_1_hp;        // 红方英雄血量
    uint16_t red_3_hp;        // 红方步兵3血量
    uint16_t red_7_hp;        // 红方哨兵血量
    uint16_t red_dead_bits;   // 红方死亡位标记
    
    // hp_blue (8B)
    uint16_t blue_1_hp;       // 蓝方英雄血量
    uint16_t blue_3_hp;       // 蓝方步兵3血量
    uint16_t blue_7_hp;       // 蓝方哨兵血量
    uint16_t blue_dead_bits;  // 蓝方死亡位标记
    
    // enemy_pos_1 (8B)
    int16_t enemy_hero_x;     // 敌方英雄X (cm)
    int16_t enemy_hero_y;     // 敌方英雄Y (cm)
    int16_t enemy_engineer_x; // 敌方工程X (cm)
    int16_t enemy_engineer_y; // 敌方工程Y (cm)
    
    // enemy_pos_2 (8B)
    int16_t enemy_std3_x;     // 敌方步兵3 X (cm)
    int16_t enemy_std3_y;     // 敌方步兵3 Y (cm)
    int16_t enemy_std4_x;     // 敌方步兵4 X (cm)
    int16_t enemy_std4_y;     // 敌方步兵4 Y (cm)
    
    // enemy_pos_3 (8B)
    int16_t enemy_sentry_x;   // 敌方哨兵X (cm)
    int16_t enemy_sentry_y;   // 敌方哨兵Y (cm)
    uint8_t suggested_target; // 雷达建议目标
    uint16_t radar_flags;     // 雷达标记信息
    uint8_t reserved_ep3;     // 保留字段
    
    // sentry_info (2B)
    uint8_t can_free_revive;  // 可免费复活
    uint8_t can_instant_revive; // 可立即复活
    
    // robot_state (8B)
    uint16_t self_hp;         // 本机血量
    uint16_t self_max_hp;     // 本机最大血量
    uint16_t bullet_remain;   // 剩余弹量
    uint16_t reserved_rs;     // 保留字段
    
    // operator_input (8B)
    float operator_x;         // 操作手X输入 (m/s)
    float operator_y;         // 操作手Y输入 (m/s)
    
    // game_result (1B)
    uint8_t winner;           // 比赛结果
    
    // hurt (2B)
    uint8_t hurt_info;        // 低4位=armor_id, 高4位=hurt_reason
    uint8_t reserved_hurt;    // 保留字段
} __attribute__((packed));

// HK协议完整数据帧 (78字节)
struct MCUDataFrame
{
    HKFrameHeader header;     // 0-8: 帧头 (9字节)
    HKGameData data;          // 9-73: 数据 (65字节)
    uint16_t packet_crc16;    // 74-75: 数据CRC16
    uint8_t trailer[2];       // 76-77: 帧尾 'K', 'H'
} __attribute__((packed));

static_assert(sizeof(HKFrameHeader) == 9, "HKFrameHeader must be exactly 9 bytes");
static_assert(sizeof(HKGameData) == 65, "HKGameData must be exactly 65 bytes");
static_assert(sizeof(MCUDataFrame) == 78, "MCUDataFrame must be exactly 78 bytes");


// HK协议相关常量
#define HK_FRAME_SOF_H 0x48         // 'H'
#define HK_FRAME_SOF_K 0x4B         // 'K'
#define HK_FRAME_TRAILER_K 0x4B     // 'K'
#define HK_FRAME_TRAILER_H 0x48     // 'H'
#define HK_PACKET_TYPE_GAME 0x01    // 比赛数据帧
#define HK_FRAME_SIZE sizeof(MCUDataFrame)  // 78字节
#define HK_FRAME_HEADER_SIZE 9      // 帧头大小
#define HK_FRAME_DATA_SIZE 65       // 数据大小

#endif // MCU_COMM_HPP
