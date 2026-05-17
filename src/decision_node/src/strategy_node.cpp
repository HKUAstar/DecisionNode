/*
决策模块
通信：
  订阅裁判系统相关话题（占位，后续可替换为 Referee_Task 的真实桥接）
  订阅导航到达状态话题（复用现有语义）
  发布目标点话题（clicked_point）
  发布烧饼状态话题（motion）
[TODO] 
- 后续可接真实视觉数据
- 修改上下位机通讯逻辑，只负责运动控制的收发
- 在决策判断是否到达感觉很别扭，是否放在上下位机通讯模块更合适？
*/
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/decorators/inverter_node.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

#include "decision_node/battle_field_status.hpp"
#include "decision_node/motion_change.hpp"
#include "decision_node/recover_change.hpp"
#include "decision_node/chase.hpp"
#include "decision_node/base_move.hpp"
namespace
{
constexpr int kDefaultTickHz = 20;

std::string toUpper(std::string s)
{
  for (auto& c : s)
  {
    c = static_cast<char>(::toupper(c));
  }
  return s;
}
// 2D 刚体变换（旋转+平移）的最小二乘求解
// 从 N 组对应点对 (src[i] → tgt[i]) 计算最优旋转角 theta [rad]
// 和平移 (tx, ty)，使得 sum ||R*src_i + t - tgt_i||^2 最小
struct Rigid2D
{
  double theta = 0.0;
  double tx    = 0.0;
  double ty    = 0.0;
};

Rigid2D computeRigid2D(const std::vector<std::pair<double, double>>& src_pts,
                       const std::vector<std::pair<double, double>>& tgt_pts)
{
  const size_t N = src_pts.size();
  if (N == 0) return Rigid2D{};

  // 1. 计算质心
  double src_cx = 0, src_cy = 0, tgt_cx = 0, tgt_cy = 0;
  for (size_t i = 0; i < N; ++i)
  {
    src_cx += src_pts[i].first;   src_cy += src_pts[i].second;
    tgt_cx += tgt_pts[i].first;   tgt_cy += tgt_pts[i].second;
  }
  src_cx /= static_cast<double>(N); src_cy /= static_cast<double>(N);
  tgt_cx /= static_cast<double>(N); tgt_cy /= static_cast<double>(N);

  // 2. 去质心后求最优旋转角（最小二乘）
  double num = 0, den = 0;
  for (size_t i = 0; i < N; ++i)
  {
    double dx1 = src_pts[i].first  - src_cx;
    double dy1 = src_pts[i].second - src_cy;
    double dx2 = tgt_pts[i].first  - tgt_cx;
    double dy2 = tgt_pts[i].second - tgt_cy;
    num += dx1 * dy2 - dy1 * dx2;   // cross
    den += dx1 * dx2 + dy1 * dy2;   // dot
  }
  double theta = std::atan2(num, den);

  // 3. 回代求平移
  double cos_t = std::cos(theta), sin_t = std::sin(theta);
  double tx = tgt_cx - (cos_t * src_cx - sin_t * src_cy);
  double ty = tgt_cy - (sin_t * src_cx + cos_t * src_cy);

  return Rigid2D{theta, tx, ty};
}

}  // namespace

// ---------------------------
// Shared state (ROS callbacks)
// ---------------------------
struct RefereeState
{
  float yaw_angle = 0.0f;          // 云台yaw角 (rad)
  
  uint8_t game_progress = 0;    // 比赛阶段
  uint16_t stage_remain_time = 420;

  
  uint16_t ally_base_HP = 5000;    //基地血量

  uint16_t ally_1_robot_HP = 200; // 1号机器人血量
  uint16_t ally_2_robot_HP = 250; // 2号机器人血量
  uint16_t ally_3_robot_HP = 150; // 3号机器人血量
  uint16_t ally_4_robot_HP = 150; // 4号机器人血量

  uint8_t central_elevated_ground_status = 0; // 中央高地状态（bit 7-8）
  uint8_t trapezoidal_elevated_ground_status = 0; // 梯形高地状态（bit 9-10）
  uint8_t fortress_status = 0; // 堡垒状态（bit 25-26）
  uint8_t outpost_status = 0; // 前哨战状态（bit 27-28）
  uint16_t ally_outpost_HP = 1500; // 我方前哨战血量

  uint8_t robot_id = 7;         // 机器人ID
  uint16_t current_HP = 400;

  uint16_t projectile_allowance_17mm = 300;
  uint16_t projectile_allowance_fortress = 100;
  uint16_t remaining_gold_coin = 400;

  uint16_t accumulated_bullet_conversion = 0; // 累计哨兵远程兑换弹量（bit 0-10）
  bool can_exchange_respawn = false;     // 哨兵是否可兑换复活（bit 20）
  uint16_t respawn_money = 760; // 哨兵复活所需金币（bit 21-31）

  bool out_of_combat = true;       // 脱战状态（bit 0）
  uint16_t projectile_allowance = 400; //全队可兑换17mm弹量（bit 1-11）
  bool power_rune_available = false; // 是否有可用的能量符（bit 14)

  float enemy_hero_x = -88.88f;     // 敌方英雄X (m)
  float enemy_hero_y = -88.88f;     // 敌方英雄Y (m)
  float enemy_engineer_x = -88.88f; // 敌方工程X (m)
  float enemy_engineer_y = -88.88f; // 敌方工程Y (m)

  float enemy_std3_x = -88.88f;     // 敌方步兵3 X (m)
  float enemy_std3_y = -88.88f;     // 敌方步兵3 Y (m)
  float enemy_std4_x = -88.88f;     // 敌方步兵4 X (m)
  float enemy_std4_y = -88.88f;     // 敌方步兵4 Y (m)

  float enemy_sentry_x = -88.88f;   // 敌方哨兵X (m)
  float enemy_sentry_y = -88.88f;   // 敌方哨兵Y (m) 
  uint8_t suggested_target = 0; // 雷达建议目标
  uint16_t radar_flags = 0;     // 雷达标记信息

  uint16_t enemy_base_HP = 5000;     //敌方基地血量
  
  float operator_x = -88.88f;       // 操作手坐标X (m，来自 /referee/operator Point)
  float operator_y = -88.88f;       // 操作手坐标Y (m，来自 /referee/operator Point)

  bool supplement_resource = false;  // /referee/supplement_resource
  bool supplement_nonresource = false; // /referee/supplement_nonresource
  bool ally_fortress_rfid = false;   // /ally_fortress_rfid

};

struct NavigationState
{
  bool arrived = false;
};

struct OdomState
{
  double raw_x = 0.0, raw_y = 0.0;   // 原始坐标系下的位置（来自 /odom）[m]
  double gimbal_angle = 0.0;  // 世界系中的yaw角（弧度）
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;  // 四元数
  double yaw_angle = 0.0;  // MCU上报的云台yaw角（弧度）
};

struct VisionState
{
  float target_distance = 0.0f;  // /vision/target_distance
  bool detected = false;         // /vision/detected
};

// ---------------------------
// BT Nodes: Update blackboard
// ---------------------------
class UpdateRefereeBB : public BT::SyncActionNode
{
public:
  UpdateRefereeBB(const std::string& name, const BT::NodeConfiguration& config, const RefereeState* state)
    : BT::SyncActionNode(name, config), state_(state)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    bb->set("ref.game_progress", state_->game_progress);
    bb->set("ref.stage_remain_time", state_->stage_remain_time);
    bb->set("ref.ally_base_HP", state_->ally_base_HP);
    bb->set("ref.ally_1_robot_HP", state_->ally_1_robot_HP);
    bb->set("ref.ally_2_robot_HP", state_->ally_2_robot_HP);
    bb->set("ref.ally_3_robot_HP", state_->ally_3_robot_HP);
    bb->set("ref.ally_4_robot_HP", state_->ally_4_robot_HP);
    bb->set("ref.central_elevated_ground_status", state_->central_elevated_ground_status);
    bb->set("ref.trapezoidal_elevated_ground_status", state_->trapezoidal_elevated_ground_status);
    bb->set("ref.fortress_status", state_->fortress_status);
    bb->set("ref.outpost_status", state_->outpost_status);
    bb->set("ref.ally_outpost_HP", state_->ally_outpost_HP);
    bb->set("ref.robot_id", state_->robot_id);
    bb->set("ref.current_HP", state_->current_HP);
    bb->set("ref.projectile_allowance_17mm", state_->projectile_allowance_17mm);
    bb->set("ref.projectile_allowance_fortress", state_->projectile_allowance_fortress);
    bb->set("ref.remaining_gold_coin", state_->remaining_gold_coin);
    bb->set("ref.accumulated_bullet_conversion", state_->accumulated_bullet_conversion);
    bb->set("ref.can_exchange_respawn", state_->can_exchange_respawn);
    bb->set("ref.respawn_money", state_->respawn_money);
    bb->set("ref.out_of_combat", state_->out_of_combat);
    bb->set("ref.projectile_allowance", state_->projectile_allowance);
    bb->set("ref.power_rune_available", state_->power_rune_available);
    bb->set("ref.enemy_hero_x", state_->enemy_hero_x);
    bb->set("ref.enemy_hero_y", state_->enemy_hero_y);
    bb->set("ref.enemy_engineer_x", state_->enemy_engineer_x);
    bb->set("ref.enemy_engineer_y", state_->enemy_engineer_y);
    bb->set("ref.enemy_std3_x", state_->enemy_std3_x);
    bb->set("ref.enemy_std3_y", state_->enemy_std3_y);
    bb->set("ref.enemy_std4_x", state_->enemy_std4_x);
    bb->set("ref.enemy_std4_y", state_->enemy_std4_y);
    bb->set("ref.enemy_sentry_x", state_->enemy_sentry_x);
    bb->set("ref.enemy_sentry_y", state_->enemy_sentry_y);
    bb->set("ref.suggested_target", state_->suggested_target);
    bb->set("ref.radar_flags", state_->radar_flags);
    bb->set("ref.enemy_base_HP", state_->enemy_base_HP);
    bb->set("ref.operator_x", state_->operator_x);
    bb->set("ref.operator_y", state_->operator_y);
    bb->set("ref.supplement", state_->supplement_resource || state_->supplement_nonresource);
    bb->set("ref.ally_fortress_rfid", state_->ally_fortress_rfid);
    
    return BT::NodeStatus::SUCCESS;
  }

private:
  const RefereeState* state_;
};

class UpdateNavigationBB : public BT::SyncActionNode
{
public:
  UpdateNavigationBB(const std::string& name, const BT::NodeConfiguration& config, const NavigationState* state)
    : BT::SyncActionNode(name, config), state_(state)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    bool arrived = state_->arrived;
    config().blackboard->set("nav.arrived", arrived);
    // 保持 nav.arrived 的值不变，直到下一次收到新的 /dstar_status 消息
    // 这样才能保证 CheckArrived 能正确读取到 arrived=true
    return BT::NodeStatus::SUCCESS;
  }

private:
  const NavigationState* state_;
};

class UpdateOdomBB : public BT::SyncActionNode
{
public:
  UpdateOdomBB(const std::string& name, const BT::NodeConfiguration& config, const OdomState* state)
    : BT::SyncActionNode(name, config), state_(state)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    
    // ---- 读取 TF 变换参数，将原始坐标系下的位置转为目标坐标系 ----
    double raw_x = state_->raw_x;
    double raw_y = state_->raw_y;

    // 始终写入原始坐标
    bb->set("odom.raw_x", raw_x);
    bb->set("odom.raw_y", raw_y);

    try {
      double theta = bb->get<double>("tf.src_to_tgt.theta");
      double tx    = bb->get<double>("tf.src_to_tgt.tx");
      double ty    = bb->get<double>("tf.src_to_tgt.ty");
      double cos_t = std::cos(theta), sin_t = std::sin(theta);
      double x_tgt = cos_t * raw_x - sin_t * raw_y + tx;
      double y_tgt = sin_t * raw_x + cos_t * raw_y + ty;
      bb->set("odom.x", x_tgt);
      bb->set("odom.y", y_tgt);
    } catch (...) {
      // TF 参数尚未加载，直接使用原始坐标
      bb->set("odom.x", raw_x);
      bb->set("odom.y", raw_y);
    }
    
    // 计算yaw角（关于世界系的夹角）
    // 从四元数 (qx, qy, qz, qw) 计算yaw角
    // 返回范围: [-π, π] 弧度制
    // yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2))
    double yaw = std::atan2(
      2.0 * (state_->qw * state_->qz + state_->qx * state_->qy),
      1.0 - 2.0 * (state_->qy * state_->qy + state_->qz * state_->qz)
    );
    
    bb->set("odom.gimbal_angle", yaw);
    bb->set("odom.yaw_angle", state_->yaw_angle);
    
    return BT::NodeStatus::SUCCESS;
  }

private:
  const OdomState* state_;
};

class CalculateVisionYaw : public BT::SyncActionNode
{
public:
  CalculateVisionYaw(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    // 读取当前 gimbal_angle（由 UpdateOdomBB 写入）
    double yaw = 0.0;
    try {
      yaw = bb->get<double>("odom.gimbal_angle");
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }

    // 死区阈值(度) 
    constexpr double kDeadbandDeg = 2.5;
    constexpr double kEmaAlpha    = 0.10;   // EMA 平滑系数
    constexpr double kDeadbandRad = kDeadbandDeg * M_PI / 180.0;

    // 1) EMA 低通滤波（角度感知，处理 ±π 环绕）
    if (ema_initialized_)
    {
      double diff = yaw - yaw_ema_;
      while (diff >  M_PI) diff -= 2.0 * M_PI;
      while (diff < -M_PI) diff += 2.0 * M_PI;
      yaw_ema_ += kEmaAlpha * diff;
      while (yaw_ema_ >  M_PI) yaw_ema_ -= 2.0 * M_PI;
      while (yaw_ema_ < -M_PI) yaw_ema_ += 2.0 * M_PI;
    }
    else
    {
      yaw_ema_ = yaw;
      ema_initialized_ = true;
    }

    // 2) 死区：只有 EMA 偏离 stable 超过阈值才更新
    if (stable_initialized_)
    {
      double diff = yaw_ema_ - stable_yaw_;
      while (diff >  M_PI) diff -= 2.0 * M_PI;
      while (diff < -M_PI) diff += 2.0 * M_PI;
      if (std::abs(diff) > kDeadbandRad)
      {
        stable_yaw_ = yaw_ema_;
      }
    }
    else
    {
      stable_yaw_ = yaw_ema_;
      stable_initialized_ = true;
    }

    bb->set("odom.yaw_ema",    yaw_ema_);
    bb->set("odom.stable_yaw", stable_yaw_);

    return BT::NodeStatus::SUCCESS;
  }

private:
  double yaw_ema_ = 0.0;
  bool   ema_initialized_ = false;
  double stable_yaw_ = 0.0;
  bool   stable_initialized_ = false;
};

class UpdateVisionBB : public BT::SyncActionNode
{
public:
  UpdateVisionBB(const std::string& name, const BT::NodeConfiguration& config, const VisionState* state)
    : BT::SyncActionNode(name, config), state_(state)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    config().blackboard->set("vision.target_distance", state_->target_distance);
    config().blackboard->set("vision.detected", state_->detected);
    return BT::NodeStatus::SUCCESS;
  }

private:
  const VisionState* state_;
};

class UpdateTimersBB : public BT::SyncActionNode
{
public:
  UpdateTimersBB(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    // 初始化 enemy_outpost 标志位（默认 true，表示有敌方前哨站）
    try
    {
      (void)bb->get<bool>("count.enemy_outpost");
    }
    catch (...)
    {
      bb->set("count.enemy_outpost", true);
    }

    return BT::NodeStatus::SUCCESS;
  }
};

class UpdateBulletBB : public BT::SyncActionNode
{
public:
  UpdateBulletBB(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), last_minute_added_(0)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    int game_progress = 0;
    int remain_time = 420;
    try {
      game_progress = bb->get<int>("ref.game_progress");
      remain_time = bb->get<int>("ref.stage_remain_time");
    } catch (...) {}

    if (game_progress != 4)
    {
      // 比赛未开始或已结束 → 清零
      bb->set("count.free_bullet", 0);
      last_minute_added_ = 0;
      return BT::NodeStatus::SUCCESS;
    }

    // 比赛进行中：计算当前分钟数（420 为第 0 分钟，不增加）
    int elapsed = 420 - remain_time;
    if (elapsed < 0) elapsed = 0;
    int current_minute = elapsed / 60;  // 0, 1, 2, ...

    // 只在首次跨过新的分钟边界时增量 +100
    if (current_minute > last_minute_added_)
    {
      int free_bullet = 0;
      try { free_bullet = bb->get<int>("count.free_bullet"); } catch (...) {}

      int increments = current_minute - last_minute_added_;
      if (last_minute_added_ == 0 && current_minute >= 1)
      {
        // 从第 0 分钟开始，跳过 420s 那次（420s 不增加）
        // 但如果 last_minute_added_=0 且 current_minute>=1，这是首次跨过 420→360 边界
        free_bullet += increments * 100;
      }
      else if (last_minute_added_ >= 1)
      {
        free_bullet += increments * 100;
      }

      bb->set("count.free_bullet", free_bullet);
      last_minute_added_ = current_minute;

      ROS_INFO("UpdateBulletBB: +%d (elapsed=%ds, minute %d→%d), total=%d",
               increments * 100, elapsed, current_minute - increments, current_minute, free_bullet);
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  int last_minute_added_;  // 上次 +100 时对应的分钟数
};

class UpdateDerivedFlags : public BT::SyncActionNode
{
public:
  UpdateDerivedFlags(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("danger_hp", 100, "HP threshold: below which is considered 'in danger'"),
      BT::InputPort<int>("sufficient_bullet", 10, "Bullet threshold: below which is considered insufficient"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    const int remain_hp = bb->get<int>("ref.current_HP");
    const int bullet_remain = bb->get<int>("ref.projectile_allowance_17mm");
    
    // --- Damage Tracking Logic ---
    ros::Time now = ros::Time::now();
    
    // Initialize last_hp_ on first run or if it was reset
    if (last_hp_ == -1) {
        last_hp_ = remain_hp;
    }

    // Detect damage (hp drop)
    // Only accumulate damage if we are consistently tracking. 
    // If massive jump up (respawn?), reset? For now just track drops.
    if (remain_hp < last_hp_) {
        int damage = last_hp_ - remain_hp;
        damage_history_.push_back({now, damage});
        // ROS_DEBUG("UpdateDerivedFlags: Detected damage %d. History size: %lu", damage, damage_history_.size());
    } else if (remain_hp > last_hp_) {
        // Healed or respawned
        // Do we reset history on respawn? Maybe not necessary for small heals.
        // If respawn (hp jump to max), maybe clear history? 
        // Assuming standard healing, we just update last_hp_.
    }
    last_hp_ = remain_hp;

    // Prune history older than 2 seconds
    while (!damage_history_.empty()) {
        double time_diff = (now - damage_history_.front().first).toSec();
        if (time_diff > 2.0) {
            damage_history_.pop_front();
        } else {
            break; 
        }
    }

    // Sum damage in window
    int total_damage_2s = 0;
    for (const auto& entry : damage_history_) {
        total_damage_2s += entry.second;
    }
    
    bb->set("derived.damage_2s", total_damage_2s);
    // ----------------------------

    // ROS_INFO("UpdateDerivedFlags: remain_hp=%d, bullet_remain=%d", remain_hp, bullet_remain);

    int danger_hp = 100;
    (void)getInput("danger_hp", danger_hp);

    int sufficient_bullet = 10;
    (void)getInput("sufficient_bullet", sufficient_bullet);

    const bool is_dead = (remain_hp <= 0);
    const bool is_in_danger = (remain_hp > 0 && remain_hp < danger_hp);
    const bool bullet_sufficient = (bullet_remain >= sufficient_bullet);

    bb->set("is_dead", is_dead);
    bb->set("is_in_danger", is_in_danger);
    bb->set("bullet_sufficient", bullet_sufficient);

    // V1: central_occupiable 先作为占位，后续接裁判/受击统计
    try
    {
      (void)bb->get<bool>("central_occupiable");
    }
    catch (...)
    {
      bb->set("central_occupiable", false);
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  int last_hp_         = -1;
  ros::Time last_tick_ = ros::Time(0);
  std::deque<std::pair<ros::Time, int>> damage_history_;
};

class IntenseHarm : public BT::ConditionNode
{
public:
  IntenseHarm(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), is_active_(false), last_hp_(-1)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("threshold_activate", 100, "Damage threshold to activate"),
      BT::InputPort<int>("threshold_deactivate", 50, "Damage threshold to deactivate"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int remain_hp = 0;
    try {
      remain_hp = bb->get<int>("ref.current_HP");
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }

    ros::Time now = ros::Time::now();
    if (last_hp_ == -1) last_hp_ = remain_hp;

    if (remain_hp < last_hp_) {
      damage_history_.push_back({now, last_hp_ - remain_hp});
    }
    last_hp_ = remain_hp;

    // 清理 2 秒以外的旧记录
    while (!damage_history_.empty()) {
      if ((now - damage_history_.front().first).toSec() > 2.0)
        damage_history_.pop_front();
      else
        break;
    }

    int damage_2s = 0;
    for (const auto& e : damage_history_) damage_2s += e.second;
    // ----------------------------------------------------

    int t_on = 100, t_off = 50;
    getInput("threshold_activate", t_on);
    getInput("threshold_deactivate", t_off);

    if (!is_active_) {
      if (damage_2s > t_on) {
        is_active_ = true;
        ROS_INFO("IntenseHarm: Activated! Damage(2s)=%d >= %d", damage_2s, t_on);
      }
    } else {
      if (damage_2s < t_off) {
        is_active_ = false;
        ROS_INFO("IntenseHarm: Deactivated! Damage(2s)=%d < %d", damage_2s, t_off);
      }
    }

    return is_active_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  bool is_active_;
  int  last_hp_;
  std::deque<std::pair<ros::Time, int>> damage_history_;
};

// ---------------------------
// BT Nodes: Conditions
// ---------------------------
class IsGameStarted : public BT::ConditionNode
{
public:
  IsGameStarted(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() 
  { 
    return {BT::InputPort<bool>("expect_started", true, "Expected game state: true=started, false=not started")};
  }

  BT::NodeStatus tick() override
  {
    const int gp = config().blackboard->get<int>("ref.game_progress");
    const bool is_started = (gp == 4);  // 4 means game in progress
    
    // Get the expect_started parameter, default to true if not provided
    const bool expect_started = getInput<bool>("expect_started").value_or(true);
    
    // Return SUCCESS if actual state matches expected state
    const bool condition_met = (is_started == expect_started);
    return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsSentryDead : public BT::ConditionNode
{
public:
  IsSentryDead(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    int hp = 0;
    try {
      hp = config().blackboard->get<int>("ref.current_HP");
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }
    return (hp <= 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsSentryAlive : public BT::ConditionNode
{
public:
  IsSentryAlive(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    int hp = 0;
    try {
      hp = config().blackboard->get<int>("ref.current_HP");
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }
    return (hp > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsSentryInDanger : public BT::ConditionNode
{
public:
  IsSentryInDanger(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("danger_hp", 100, "HP threshold: below which is considered in danger"),
      BT::InputPort<std::string>("hp_key", "ref.current_HP", "Blackboard key for current HP"),
    };
  }

  BT::NodeStatus tick() override
  {
    int danger_hp = 100;
    (void)getInput("danger_hp", danger_hp);

    std::string hp_key = "ref.current_HP";
    (void)getInput("hp_key", hp_key);

    int remain_hp = 0;
    try
    {
      remain_hp = config().blackboard->get<int>(hp_key);
    }
    catch (...)
    {
      ROS_WARN_THROTTLE(1.0, "IsSentryInDanger: failed to read hp from key '%s'", hp_key.c_str());
      return BT::NodeStatus::FAILURE;
    }

    const bool in_danger = (remain_hp > 0 && remain_hp < danger_hp);
    return in_danger ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class NotBulletSufficient : public BT::ConditionNode
{
public:
  NotBulletSufficient(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("sufficient_bullet", 10, "Bullet threshold: below this is insufficient"),
    };
  }

  BT::NodeStatus tick() override
  {
    int bullet = 0;
    try {
      bullet = config().blackboard->get<int>("ref.projectile_allowance_17mm");
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }

    int threshold = 10;
    (void)getInput("sufficient_bullet", threshold);

    return (bullet < threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsAction : public BT::ConditionNode
{
public:
  IsAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("value", "INIT", "Action name")};
  }

  BT::NodeStatus tick() override
  {
    std::string value;
    if (!getInput("value", value))
    {
      return BT::NodeStatus::FAILURE;
    }

    std::string action;
    try
    {
      action = config().blackboard->get<std::string>("action");
    }
    catch (...)
    {
      return BT::NodeStatus::FAILURE;
    }

    bool result = (toUpper(action) == toUpper(value));
    // ROS_INFO("IsAction: checking action=%s, expect=%s, result=%d", 
    //          toUpper(action).c_str(), toUpper(value).c_str(), result);
    return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// ---------------------------
// BT Nodes: Actions
// ---------------------------
class SetAction : public BT::SyncActionNode
{
public:
  SetAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("action")}; }

  BT::NodeStatus tick() override
  {
    std::string action;
    if (!getInput("action", action))
    {
      return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set("action", toUpper(action));
   
     ROS_INFO_THROTTLE(2.0, "[SetAction] action set to %s", toUpper(action).c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

class SetBBValue : public BT::SyncActionNode
{
public:
  SetBBValue(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("key", "", "黑板键名"),
      BT::InputPort<std::string>("type", "int", "值类型: int / double / bool / string"),
      BT::InputPort<std::string>("value", "0", "目标值的字符串表示"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string key, type, val_str;
    if (!getInput("key", key) || key.empty())
    {
      ROS_WARN("SetBBValue: 'key' is required");
      return BT::NodeStatus::FAILURE;
    }
    (void)getInput("type", type);
    (void)getInput("value", val_str);

    auto bb = config().blackboard;

    if (type == "int")
    {
      bb->set(key, std::stoi(val_str));
      // ROS_INFO("SetBBValue: %s = %d (int)", key.c_str(), std::stoi(val_str));
    }
    else if (type == "double")
    {
      bb->set(key, std::stod(val_str));
      // ROS_INFO("SetBBValue: %s = %.3f (double)", key.c_str(), std::stod(val_str));
    }
    else if (type == "bool")
    {
      bool v = (val_str == "true" || val_str == "1");
      bb->set(key, v);
      // ROS_INFO("SetBBValue: %s = %s (bool)", key.c_str(), v ? "true" : "false");
    }
    else if (type == "string")
    {
      bb->set(key, val_str);
      // ROS_INFO("SetBBValue: %s = \"%s\" (string)", key.c_str(), val_str.c_str());
    }
    else
    {
      ROS_WARN("SetBBValue: unknown type '%s'", type.c_str());
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }
};

class ClearGoal : public BT::SyncActionNode
{
public:
  ClearGoal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    // ROS_INFO("ClearGoal: setting goal.valid = false");
    bb->set("goal.valid", false);
    return BT::NodeStatus::SUCCESS;
  }
};

// Wait: 等待指定的秒数
class Wait : public BT::StatefulActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", 1.0, "等待的秒数"),
    };
  }

  BT::NodeStatus onStart() override
  {
    double duration = 1.0;
    if (!getInput("duration", duration))
    {
      return BT::NodeStatus::FAILURE;
    }

    wait_end_time_ = ros::Time::now() + ros::Duration(duration);
    ROS_DEBUG("Wait: Starting wait for %.2f seconds", duration);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (ros::Time::now() >= wait_end_time_)
    {
      ROS_DEBUG("Wait: Wait completed");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    ROS_DEBUG("Wait: Halted");
  }

private:
  ros::Time wait_end_time_;
};

// 从 strategy_tree.yaml 读取目标点坐标
// 支持红蓝方切换：根据 robot_id 自动选择 goals.red 或 goals.blue
class SetGoalFromParams : public BT::SyncActionNode
{
public:
  SetGoalFromParams(const std::string& name, const BT::NodeConfiguration& config, YAML::Node* yaml_root)
    : BT::SyncActionNode(name, config), yaml_root_(yaml_root)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ns", "push", "parameter namespace: e.g. push/occupy/supply"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string ns;
    if (!getInput("ns", ns))
    {
      ROS_WARN("SetGoalFromParams: Failed to get 'ns' input");
      return BT::NodeStatus::FAILURE;
    }

    // ----- 确定队伍 -----
    auto bb = config().blackboard;
    std::string team = "red";  // 默认红方
    if (yaml_root_ && (*yaml_root_)["goals"]["blue"].IsDefined())
    {
      try
      {
        uint8_t robot_id = bb->get<uint8_t>("ref.robot_id");
        if (robot_id == 107)
          team = "blue";
        else
          team = "red";
      }
      catch (...) { /* 无法读取 robot_id，使用默认红方 */ }
    }

    double x = 0.0, y = 0.0;
    bool found = false;

    if (yaml_root_ && yaml_root_->IsDefined())
    {
      try
      {
        const YAML::Node goals = (*yaml_root_)["goals"];
        const YAML::Node pt = goals[team][ns];
        if (pt && pt["x"] && pt["y"])
        {
          x = pt["x"].as<double>();
          y = pt["y"].as<double>();
          found = true;
        }
      }
      catch (...) { found = false; }
    }

    if (!found)
    {
      ROS_ERROR("SetGoalFromParams: namespace '%s' not found (team=%s) in strategy_tree.yaml", ns.c_str(), team.c_str());
      return BT::NodeStatus::FAILURE;
    }

    ROS_DEBUG("SetGoalFromParams: team=%s, ns=%s, goal=(%f, %f)", team.c_str(), ns.c_str(), x, y);

    geometry_msgs::PointStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.point.x = x;
    goal.point.y = y;
    goal.point.z = 0.0;

    bb->set("goal.point", goal);
    bb->set("goal.valid", true);
    ROS_DEBUG("SetGoalFromParams: goal.valid set to TRUE");
    return BT::NodeStatus::SUCCESS;
  }

private:
  YAML::Node* yaml_root_;
};

class SetGoalFromParamsCyclic : public BT::SyncActionNode
{
public:
  SetGoalFromParamsCyclic(const std::string& name,
                           const BT::NodeConfiguration& config,
                           YAML::Node* yaml_root)
    : BT::SyncActionNode(name, config), yaml_root_(yaml_root)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ns", "push", "parameter namespace: e.g. push/occupy"),
      BT::InputPort<int>("point_count", 4, "number of points to cycle through"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string ns;
    int point_count = 4;
    if (!getInput("ns", ns))
    {
      ROS_WARN("SetGoalFromParamsCyclic: Failed to get 'ns' input");
      return BT::NodeStatus::FAILURE;
    }
    (void)getInput("point_count", point_count);

    auto bb = config().blackboard;

    // ----- 确定队伍 -----
    std::string team = "red";  // 默认红方
    if (yaml_root_ && (*yaml_root_)["goals"]["blue"].IsDefined())
    {
      try
      {
        uint8_t robot_id = bb->get<uint8_t>("ref.robot_id");
        if (robot_id == 107)
          team = "blue";
        else
          team = "red";
      }
      catch (...) { /* 无法读取 robot_id，使用默认红方 */ }
    }

    const std::string idx_key        = "cycle_idx_" + ns;
    const std::string last_action_key = "cycle_last_action_" + ns;

    std::string cur_action, last_action;
    try { cur_action  = bb->get<std::string>("action"); }       catch (...) {}
    try { last_action = bb->get<std::string>(last_action_key); } catch (...) {}
    if (cur_action != last_action)
    {
      bb->set(idx_key, 0);
      bb->set(last_action_key, cur_action);
    }

    int cycle_index = 0;
    try { cycle_index = bb->get<int>(idx_key); } catch (...) { bb->set(idx_key, 0); }

    double x = 0.0, y = 0.0;
    bool found = false;

    if (yaml_root_ && yaml_root_->IsDefined())
    {
      try
      {
        const YAML::Node goals = (*yaml_root_)["goals"];
        const YAML::Node goal_node = goals[team][ns];
        if (goal_node && goal_node["point_" + std::to_string(cycle_index)])
        {
          const auto& pt = goal_node["point_" + std::to_string(cycle_index)];
          x = pt["x"].as<double>();
          y = pt["y"].as<double>();
          found = true;
        }
      }
      catch (...) { found = false; }
    }

    if (!found)
    {
      ROS_ERROR("SetGoalFromParamsCyclic[%s]: point_%d not found (team=%s) in strategy_tree.yaml",
                ns.c_str(), cycle_index, team.c_str());
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::PointStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.point.x = x;
    goal.point.y = y;
    goal.point.z = 0.0;

    bb->set("goal.point", goal);
    bb->set("goal.valid", true);
    ROS_DEBUG("SetGoalFromParamsCyclic[%s]: index=%d, goal=(%.3f, %.3f)",
              ns.c_str(), cycle_index, x, y);
    return BT::NodeStatus::SUCCESS;
  }

private:
  YAML::Node* yaml_root_;
};

class AdvanceCycleIndex : public BT::SyncActionNode
{
public:
  AdvanceCycleIndex(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ns", "", "parameter namespace, must match SetGoalFromParamsCyclic"),
      BT::InputPort<int>("point_count", 4, "number of points to cycle through"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    std::string ns;
    int point_count = 4;
    (void)getInput("ns", ns);
    (void)getInput("point_count", point_count);

    // 与 SetGoalFromParamsCyclic 保持相同的 per-ns key
    const std::string idx_key = "cycle_idx_" + ns;

    int cycle_index = 0;
    try {
      cycle_index = bb->get<int>(idx_key);
    } catch (...) {
      cycle_index = 0;
    }

    // Advance to next point and wrap around
    cycle_index = (cycle_index + 1) % point_count;
    bb->set(idx_key, cycle_index);
    ROS_DEBUG("AdvanceCycleIndex[%s]: advanced to index=%d", ns.c_str(), cycle_index);

    return BT::NodeStatus::SUCCESS;
  }
};


class RecordVisionAnchor : public BT::SyncActionNode
{
public:
  RecordVisionAnchor(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    std::string cur_action;
    try { cur_action = bb->get<std::string>("action"); }
    catch (...) { return BT::NodeStatus::FAILURE; }

    if (cur_action != last_action_)
    {
      last_action_ = cur_action;
      if (cur_action == "VISION")
      {
        try {
          double x = bb->get<double>("odom.raw_x");
          double y = bb->get<double>("odom.raw_y");
          bb->set("vision.anchor_x", x);
          bb->set("vision.anchor_y", y);
          ROS_INFO("[RecordVisionAnchor] Anchor set at (%.2f, %.2f)", x, y);
        } catch (...) {
          ROS_WARN_THROTTLE(2.0, "[RecordVisionAnchor] Failed to read odom.raw_x/y");
          return BT::NodeStatus::FAILURE;
        }
      }
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::string last_action_;
};


class CheckVisionAnchorDistance : public BT::ConditionNode
{
public:
  CheckVisionAnchorDistance(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_distance", 1.5, "Max allowed distance from anchor (m)"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    double cur_x = 0.0, cur_y = 0.0;
    double anchor_x = 0.0, anchor_y = 0.0;
    try {
      cur_x    = bb->get<double>("odom.raw_x");
      cur_y    = bb->get<double>("odom.raw_y");
      anchor_x = bb->get<double>("vision.anchor_x");
      anchor_y = bb->get<double>("vision.anchor_y");
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }

    double max_dist = 1.5;
    (void)getInput("max_distance", max_dist);

    double dist = std::hypot(cur_x - anchor_x, cur_y - anchor_y);
    if (dist > max_dist)
    {
      ROS_WARN_THROTTLE(1.0, "[CheckVisionAnchorDistance] %.2fm > limit %.2fm, stopping",
                        dist, max_dist);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

class SetVisionTarget : public BT::SyncActionNode
{
public:
  SetVisionTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("offset_distance", 0.4, "Distance to stop before the target (m)"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    // 1. 读取哨兵自身位置（原始坐标系）
    double odom_x = 0.0, odom_y = 0.0;
    try {
      odom_x = bb->get<double>("odom.raw_x");
      odom_y = bb->get<double>("odom.raw_y");
    } catch (...) {
      ROS_WARN_THROTTLE(2.0, "[SetVisionTarget] odom.raw_x / odom.raw_y not in blackboard");
      return BT::NodeStatus::FAILURE;
    }

    // 2. 读取目标距离
    float target_distance = 0.0f;
    try {
      target_distance = bb->get<float>("vision.target_distance");
    } catch (...) {
      ROS_WARN_THROTTLE(2.0, "[SetVisionTarget] vision.target_distance not in blackboard");
      return BT::NodeStatus::FAILURE;
    }

    // 3. 读取目标角度（世界系，弧度）
    double stable_yaw = 0.0;
    try {
      stable_yaw = bb->get<double>("odom.stable_yaw");
    } catch (...) {
      ROS_WARN_THROTTLE(2.0, "[SetVisionTarget] odom.stable_yaw not in blackboard");
      return BT::NodeStatus::FAILURE;
    }

    // 4. 读取回退距离（默认 0.4m）
    double offset = 0.4;
    (void)getInput("offset_distance", offset);

    // 5. 计算目标点的绝对坐标
    //    目标在: (odom_x + target_distance * cos(yaw), odom_y + target_distance * sin(yaw))
    //    我们想到达目标前方 offset 处，即:
    double effective_distance = std::max(0.0, static_cast<double>(target_distance) - offset);
    double goal_x = odom_x + effective_distance * std::cos(stable_yaw);
    double goal_y = odom_y + effective_distance * std::sin(stable_yaw);

    // 6. 写入黑板
    geometry_msgs::PointStamped goal;
    goal.header.stamp    = ros::Time::now();
    goal.header.frame_id = "map";
    goal.point.x = goal_x;
    goal.point.y = goal_y;
    goal.point.z = 0.0;

    bb->set("goal.point", goal);
    bb->set("goal.valid", true);

    ROS_INFO_THROTTLE(2.0, "[SetVisionTarget] dist=%.2fm, yaw=%.2f°, offset=%.2fm → goal=(%.2f, %.2f)",
                      target_distance, stable_yaw * 180.0 / M_PI, offset, goal_x, goal_y);

    return BT::NodeStatus::SUCCESS;
  }
};

class PublishGoalPoint : public BT::SyncActionNode
{
public:
  PublishGoalPoint(const std::string& name,
                   const BT::NodeConfiguration& config,
                   ros::Publisher* pub,
                   bool* publish_on_change_only)
    : BT::SyncActionNode(name, config), pub_(pub), publish_on_change_only_(publish_on_change_only)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "clicked_point", "target topic"),
    };
  }

  BT::NodeStatus tick() override
  {
    (void)getInput("topic", last_topic_);  // retained only for XML readability

    auto bb = config().blackboard;
    const bool valid = bb->get<bool>("goal.valid");
    if (!valid)
    {
      return BT::NodeStatus::SUCCESS;
    }

    auto goal = bb->get<geometry_msgs::PointStamped>("goal.point");

    // ==== 逆 TF 变换：目标坐标系 → 原始坐标系 ====
    // 正向: x_tgt = cosθ·x_src - sinθ·y_src + tx
    // 逆向: x_src =  cosθ·(x_tgt - tx) + sinθ·(y_tgt - ty)
    //        y_src = -sinθ·(x_tgt - tx) + cosθ·(y_tgt - ty)
    try
    {
      double theta = bb->get<double>("tf.src_to_tgt.theta");
      double tx    = bb->get<double>("tf.src_to_tgt.tx");
      double ty    = bb->get<double>("tf.src_to_tgt.ty");

      double dx = goal.point.x - tx;
      double dy = goal.point.y - ty;
      double cos_t = std::cos(theta);
      double sin_t = std::sin(theta);

      double x_inv =  cos_t * dx + sin_t * dy;
      double y_inv = -sin_t * dx + cos_t * dy;

      goal.point.x = x_inv;
      goal.point.y = y_inv;
      // goal.point.z, header 不变
    }
    catch (...) { /* TF 参数不存在，直接用原值发布 */ }

    if (*publish_on_change_only_)
    {
      bool have_last = false;
      double last_x = 0.0, last_y = 0.0;
      try
      {
        last_x = bb->get<double>("goal.last_x");
        last_y = bb->get<double>("goal.last_y");
        have_last = true;
      }
      catch (...)
      {
        have_last = false;
      }

      if (have_last && goal.point.x == last_x && goal.point.y == last_y)
      {
        return BT::NodeStatus::SUCCESS;
      }
      bb->set("goal.last_x", goal.point.x);
      bb->set("goal.last_y", goal.point.y);
    }

    pub_->publish(goal);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::Publisher* pub_;
  bool* publish_on_change_only_;
  std::string last_topic_;
};

// ---------------------------
// Main
// ---------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "strategy_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  RefereeState ref;
  NavigationState nav;
  OdomState odom;
  VisionState vis;

  auto blackboard = BT::Blackboard::create();
  
  // Odom 订阅：获取原始坐标系下的位姿（position + orientation）
  auto sub_odom = nh.subscribe<nav_msgs::Odometry>("/odom", 1, [&](const nav_msgs::Odometry::ConstPtr& msg) {
    // 原始坐标系下的位置（后续由 UpdateOdomBB 做 TF 转换）
    odom.raw_x = msg->pose.pose.position.x;
    odom.raw_y = msg->pose.pose.position.y;
    
    odom.qx = msg->pose.pose.orientation.x;
    odom.qy = msg->pose.pose.orientation.y;
    odom.qz = msg->pose.pose.orientation.z;
    odom.qw = msg->pose.pose.orientation.w;
    

    // yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2))
    // 返回范围: [-π, π] 弧度制
    odom.gimbal_angle = std::atan2(
      2.0 * (odom.qw * odom.qz + odom.qx * odom.qy),
      1.0 - 2.0 * (odom.qy * odom.qy + odom.qz * odom.qz)
    );
  });

  // MCU云台yaw角订阅
  auto sub_mcu_yaw = nh.subscribe<std_msgs::Float32>("/mcu/yaw_angle", 1, [&](const std_msgs::Float32::ConstPtr& msg) {
    odom.yaw_angle = static_cast<double>(msg->data);
  });

  // 裁判系统数据
  auto sub_game_progress = nh.subscribe<std_msgs::UInt8>("/referee/game_progress", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.game_progress = msg->data;
  });
  auto sub_stage_remain_time = nh.subscribe<std_msgs::UInt16>("/referee/stage_remain_time", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.stage_remain_time = msg->data;
  });
  auto sub_ally_base_hp = nh.subscribe<std_msgs::UInt16>("/referee/ally_base_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.ally_base_HP = msg->data;
  });
  auto sub_ally_1_hp = nh.subscribe<std_msgs::UInt16>("/referee/ally_1_robot_HP", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.ally_1_robot_HP = msg->data;
  });
  auto sub_ally_2_hp = nh.subscribe<std_msgs::UInt16>("/referee/ally_2_robot_HP", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.ally_2_robot_HP = msg->data;
  });
  auto sub_ally_3_hp = nh.subscribe<std_msgs::UInt16>("/referee/ally_3_robot_HP", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.ally_3_robot_HP = msg->data;
  });
  auto sub_ally_4_hp = nh.subscribe<std_msgs::UInt16>("/referee/ally_4_robot_HP", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.ally_4_robot_HP = msg->data;
  });
  auto sub_central_ground = nh.subscribe<std_msgs::UInt8>("/referee/central_ground_status", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.central_elevated_ground_status = msg->data;
  });
  auto sub_trap_ground = nh.subscribe<std_msgs::UInt8>("/referee/trap_ground_status", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.trapezoidal_elevated_ground_status = msg->data;
  });
  auto sub_fortress = nh.subscribe<std_msgs::UInt8>("/referee/fortress_status", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.fortress_status = msg->data;
  });
  auto sub_outpost = nh.subscribe<std_msgs::UInt8>("/referee/outpost_status", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.outpost_status = msg->data;
  });
  auto sub_ally_outpost_hp = nh.subscribe<std_msgs::UInt16>("/referee/ally_outpost_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.ally_outpost_HP = msg->data;
  });

  // 自身状态
  auto sub_robot_id = nh.subscribe<std_msgs::UInt8>("/robot/robot_id", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.robot_id = msg->data;
  });
  auto sub_self_hp = nh.subscribe<std_msgs::UInt16>("/robot/self_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.current_HP = msg->data;
  });

  // 弹药物资
  auto sub_proj_17mm = nh.subscribe<std_msgs::UInt16>("/referee/projectile_17mm", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.projectile_allowance_17mm = msg->data;
  });
  auto sub_proj_fort = nh.subscribe<std_msgs::UInt16>("/referee/projectile_fortress", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.projectile_allowance_fortress = msg->data;
  });
  auto sub_gold = nh.subscribe<std_msgs::UInt16>("/referee/remaining_gold", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.remaining_gold_coin = msg->data;
  });

  // 哨兵特殊
  auto sub_acc_bullet = nh.subscribe<std_msgs::UInt16>("/referee/accumulated_bullet", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.accumulated_bullet_conversion = msg->data;
  });
  auto sub_can_respawn = nh.subscribe<std_msgs::Bool>("/referee/can_exchange_respawn", 1, [&](const std_msgs::Bool::ConstPtr& msg) {
    ref.can_exchange_respawn = msg->data;
  });
  auto sub_respawn_money = nh.subscribe<std_msgs::UInt16>("/referee/respawn_money", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.respawn_money = msg->data;
  });
  auto sub_combat = nh.subscribe<std_msgs::Bool>("/referee/out_of_combat", 1, [&](const std_msgs::Bool::ConstPtr& msg) {
    ref.out_of_combat = msg->data;
  });
  auto sub_proj_allow = nh.subscribe<std_msgs::UInt16>("/referee/projectile_allowance", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.projectile_allowance = msg->data;
  });
  auto sub_power_rune = nh.subscribe<std_msgs::Bool>("/referee/power_rune_available", 1, [&](const std_msgs::Bool::ConstPtr& msg) {
    ref.power_rune_available = msg->data;
  });

  // 敌方位置（geometry_msgs::Point，单位 m → 直接存 float）
  auto sub_enemy_hero = nh.subscribe<geometry_msgs::Point>("/enemy/hero_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_hero_x = msg->x;
    ref.enemy_hero_y = msg->y;
  });
  auto sub_enemy_eng = nh.subscribe<geometry_msgs::Point>("/enemy/engineer_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_engineer_x = msg->x;
    ref.enemy_engineer_y = msg->y;
  });
  auto sub_enemy_std3 = nh.subscribe<geometry_msgs::Point>("/enemy/standard_3_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_std3_x = msg->x;
    ref.enemy_std3_y = msg->y;
  });
  auto sub_enemy_std4 = nh.subscribe<geometry_msgs::Point>("/enemy/standard_4_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_std4_x = msg->x;
    ref.enemy_std4_y = msg->y;
  });
  auto sub_enemy_sentry = nh.subscribe<geometry_msgs::Point>("/enemy/sentry_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_sentry_x = msg->x;
    ref.enemy_sentry_y = msg->y;
  });

  // 雷达
  auto sub_suggested = nh.subscribe<std_msgs::UInt8>("/radar/suggested_target", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.suggested_target = msg->data;
  });
  auto sub_radar_flags = nh.subscribe<std_msgs::UInt16>("/radar/radar_flags", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.radar_flags = msg->data;
  });

  // 敌方基地血量
  auto sub_enemy_base_hp = nh.subscribe<std_msgs::UInt16>("/referee/enemy_base_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.enemy_base_HP = msg->data;
  });

  // 操作手坐标 (geometry_msgs::Point, m → mm)
  auto sub_operator = nh.subscribe<geometry_msgs::Point>("/referee/operator", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.operator_x = msg->x;
    ref.operator_y = msg->y;
  });

  // 补弹资源
  auto sub_supplement_resource = nh.subscribe<std_msgs::Bool>("/referee/supplement_resource", 1, [&](const std_msgs::Bool::ConstPtr& msg) {
    ref.supplement_resource = msg->data;
  });
  auto sub_supplement_nonresource = nh.subscribe<std_msgs::Bool>("/referee/supplement_nonresource", 1, [&](const std_msgs::Bool::ConstPtr& msg) {
    ref.supplement_nonresource = msg->data;
  });
  auto sub_ally_fortress_rfid = nh.subscribe<std_msgs::Bool>("ally_fortress_rfid", 1, [&](const std_msgs::Bool::ConstPtr& msg) {
    ref.ally_fortress_rfid = msg->data;
  });
  auto sub_target_distance = nh.subscribe<std_msgs::Float32>("/vision/target_distance", 1, [&](const std_msgs::Float32::ConstPtr& msg) {
    vis.target_distance = msg->data;
  });
  auto sub_vision_detected = nh.subscribe<std_msgs::Bool>("/vision/detected", 1, [&](const std_msgs::Bool::ConstPtr& msg) {
    vis.detected = msg->data;
  });

  // Navigation arrived (复用现有语义)
  auto sub_arrived = nh.subscribe<std_msgs::Bool>("/dstar_status", 10, 
    [&](const std_msgs::Bool::ConstPtr& msg) {
      nav.arrived = msg->data;
    });

  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 1);
  ros::Publisher motion_pub = nh.advertise<std_msgs::UInt8>("motion", 1);

  // ============================================================
  // 加载 strategy_tree.yaml（目标点配置）
  // ============================================================
  YAML::Node yaml_root;
  std::string yaml_path;
  if (pnh.getParam("goal_yaml", yaml_path) || true)
  {
    if (yaml_path.empty())
      yaml_path = ros::package::getPath("decision_node") + "/config/strategy_tree.yaml";
    try
    {
      yaml_root = YAML::LoadFile(yaml_path);
      ROS_INFO("Loaded goal definitions from %s", yaml_path.c_str());
    }
    catch (const std::exception& e)
    {
      ROS_WARN("Failed to load %s: %s, falling back to ROS params", yaml_path.c_str(), e.what());
    }
  }
  ros::Publisher spin_pub = nh.advertise<std_msgs::UInt8>("spin", 1);
  ros::Publisher spin_velo_pub = nh.advertise<std_msgs::UInt8>("spin_velo", 1);
  ros::Publisher recover_pub = nh.advertise<std_msgs::UInt8>("recover", 1);
  ros::Publisher bullet_num_pub = nh.advertise<std_msgs::UInt8>("bullet_num", 1);
  ros::Publisher target_yaw_pub = nh.advertise<std_msgs::Float32>("/target_yaw", 1);

  int tick_hz = kDefaultTickHz;
  pnh.param("tick_hz", tick_hz, tick_hz);

  bool publish_on_change_only = true;
  pnh.param("publish_on_change_only", publish_on_change_only, publish_on_change_only);

  BT::BehaviorTreeFactory factory;

  // Register custom nodes.
  factory.registerBuilder<UpdateRefereeBB>(
    "UpdateRefereeBB", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<UpdateRefereeBB>(name, config, &ref);
    });

  factory.registerBuilder<UpdateNavigationBB>(
    "UpdateNavigationBB", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<UpdateNavigationBB>(name, config, &nav);
    });

  factory.registerBuilder<UpdateOdomBB>(
    "UpdateOdomBB", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<UpdateOdomBB>(name, config, &odom);
    });

  factory.registerBuilder<UpdateVisionBB>(
    "UpdateVisionBB", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<UpdateVisionBB>(name, config, &vis);
    });
  factory.registerNodeType<CalculateVisionYaw>("CalculateVisionYaw");
  factory.registerNodeType<UpdateTimersBB>("UpdateTimersBB");
  factory.registerNodeType<UpdateBulletBB>("UpdateBulletBB");
  factory.registerNodeType<IsGameStarted>("IsGameStarted");
  factory.registerNodeType<IsSentryDead>("IsSentryDead");
  factory.registerNodeType<IsSentryAlive>("IsSentryAlive");
  factory.registerNodeType<IsSentryInDanger>("IsSentryInDanger");
  factory.registerNodeType<IntenseHarm>("IntenseHarm");
  factory.registerNodeType<NotBulletSufficient>("NotBulletSufficient");
  factory.registerNodeType<IsAction>("IsAction");

  factory.registerNodeType<SetAction>("SetAction");
  factory.registerNodeType<SetBBValue>("SetBBValue");
  factory.registerNodeType<ClearGoal>("ClearGoal");
  factory.registerNodeType<Wait>("Wait");
  
  RegisterMotionChangeNodes(factory, &motion_pub, &spin_pub, &spin_velo_pub, &publish_on_change_only);

  factory.registerBuilder<SetGoalFromParams>(
    "SetGoalFromParams", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<SetGoalFromParams>(name, config, &yaml_root);
    });

  factory.registerBuilder<SetGoalFromParamsCyclic>(
    "SetGoalFromParamsCyclic", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<SetGoalFromParamsCyclic>(name, config, &yaml_root);
    });

  factory.registerNodeType<AdvanceCycleIndex>("AdvanceCycleIndex");

  factory.registerNodeType<RecordVisionAnchor>("RecordVisionAnchor");
  factory.registerNodeType<CheckVisionAnchorDistance>("CheckVisionAnchorDistance");
  factory.registerNodeType<SetVisionTarget>("SetVisionTarget");

  factory.registerBuilder<PublishGoalPoint>(
    "PublishGoalPoint", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<PublishGoalPoint>(name, config, &goal_pub, &publish_on_change_only);
    });

  RegisterRecoverChangeNodes(factory, &recover_pub, nullptr);
  RegisterBulletSupplyNodes(factory, &bullet_num_pub);
  RegisterBattleFieldNodes(factory);
  RegisterChaseNodes(factory, &goal_pub, &publish_on_change_only);
  RegisterBaseMoveNodes(factory, &nh, &target_yaw_pub);

  // ---------------------------
  // Decision Parameters (集中定义)
  // ---------------------------
  struct DecisionParams {
    int danger_hp = 100;
    int max_hp = 400;
    int sufficient_bullet = 10;
    int max_bullet = 150;
    int fixed_supply = 50;
    int occupy_threshold = 30;
    int aggressive_threshold = 50;
    int attack_threshold = 5;
    int harm_threshold_on = 50;
    int harm_threshold_off = 10;
  } params;


  pnh.param("danger_hp", params.danger_hp, params.danger_hp);
  pnh.param("max_hp", params.max_hp, params.max_hp);
  pnh.param("sufficient_bullet", params.sufficient_bullet, params.sufficient_bullet);
  pnh.param("max_bullet", params.max_bullet, params.max_bullet);
  pnh.param("fixed_supply", params.fixed_supply, params.fixed_supply);
  pnh.param("occupy_threshold", params.occupy_threshold, params.occupy_threshold);
  pnh.param("aggressive_threshold", params.aggressive_threshold, params.aggressive_threshold);
  pnh.param("attack_threshold", params.attack_threshold, params.attack_threshold);
  pnh.param("harm_threshold_on", params.harm_threshold_on, params.harm_threshold_on);
  pnh.param("harm_threshold_off", params.harm_threshold_off, params.harm_threshold_off);

  blackboard->set("danger_hp", params.danger_hp);
  blackboard->set("max_hp", params.max_hp);
  blackboard->set("sufficient_bullet", params.sufficient_bullet);
  blackboard->set("max_bullet", params.max_bullet);
  blackboard->set("fixed_supply", params.fixed_supply);
  blackboard->set("occupy_threshold", params.occupy_threshold);
  blackboard->set("aggressive_threshold", params.aggressive_threshold);
  blackboard->set("attack_threshold", params.attack_threshold);
  blackboard->set("harm_threshold_on", params.harm_threshold_on);
  blackboard->set("harm_threshold_off", params.harm_threshold_off);

  // 日志输出参数值
  ROS_INFO("Decision Parameters loaded:");
  ROS_INFO("  danger_hp=%d, max_hp=%d, sufficient_bullet=%d, max_bullet=%d",
           params.danger_hp, params.max_hp, params.sufficient_bullet, params.max_bullet);
  ROS_INFO("  fixed_supply=%d, occupy_threshold=%d, aggressive_threshold=%d",
           params.fixed_supply, params.occupy_threshold, params.aggressive_threshold);
  ROS_INFO("  attack_threshold=%d, harm_on=%d, harm_off=%d", 
           params.attack_threshold, params.harm_threshold_on, params.harm_threshold_off);

  // Default action
  blackboard->set("action", std::string("INIT"));
  blackboard->set("goal.valid", false);
  blackboard->set("goal.cycle_index", 0);
  blackboard->set("motion_flag", 0);  // 默认为0
  blackboard->set("motion.last_flag", 0);  // 初始化motion发送记录，防止冷启动后motion不变
  blackboard->set("attack_cooldown_end_time", ros::Time(0));
  blackboard->set("central_occupiable", false);
  blackboard->set("is_enemy_occupied", false);  
  blackboard->set("central_accumulate_count", 0);
  blackboard->set("occupy_reached", false);  // 占领阈值是否到达
  blackboard->set("recover", 0);  // 回血标志，默认为0
  blackboard->set("bullet_up", 0);  // 补弹标志，默认为0
  blackboard->set("bullet_num", 0);  // 补弹数量，默认为0
  blackboard->set("spin_flag", 0);   // 自旋模式，0=关闭，1=开启
  blackboard->set("spin.last_flag", -1);
  blackboard->set("server_yaw_flag", 0);  // 服务端yaw标志，默认为0
  blackboard->set("odom.target_angle", 0.0);  // 目标绝对角度，FindAngle计算
  blackboard->set("odom.target_yaw", 0.0);    // 目标相对角度，CalculateAngle计算
  blackboard->set("odom.x", 0.0);             // 当前经TF转换后的x坐标
  blackboard->set("odom.y", 0.0);             // 当前经TF转换后的y坐标
  blackboard->set("odom.raw_x", 0.0);         // 原始坐标系下x坐标（来自 /odom）
  blackboard->set("odom.raw_y", 0.0);         // 原始坐标系下y坐标（来自 /odom）
  blackboard->set("vision.anchor_x", 0.0);    // VISION 锚点 x（RecordVisionAnchor 写入）
  blackboard->set("vision.anchor_y", 0.0);    // VISION 锚点 y（RecordVisionAnchor 写入）
  blackboard->set("odom.yaw_ema", 0.0);       // CalculateVisionYaw: EMA平滑后的yaw
  blackboard->set("odom.stable_yaw", 0.0);    // CalculateVisionYaw: 死区稳定后的中心线yaw
  blackboard->set("vision.target_distance", 0.0f);  // /vision/target_distance
  blackboard->set("vision.detected", false);        // 视觉检测标志，默认无目标

  // ============================================================
  // TF 标定：从 launch 参数读取 4 组点对 (src → tgt)，
  // 计算原始坐标系到目标坐标系的 2D 刚体变换，存入黑板（只算一次）
  // ============================================================
  {
    std::vector<std::pair<double, double>> src_pts(4);
    std::vector<std::pair<double, double>> tgt_pts(4);
    bool calib_ok = true;

    for (int i = 0; i < 4; ++i)
    {
      std::string idx = std::to_string(i + 1);
      calib_ok = calib_ok
        && pnh.getParam("tf_calib/src_pt" + idx + "_x", src_pts[i].first)
        && pnh.getParam("tf_calib/src_pt" + idx + "_y", src_pts[i].second)
        && pnh.getParam("tf_calib/tgt_pt" + idx + "_x", tgt_pts[i].first)
        && pnh.getParam("tf_calib/tgt_pt" + idx + "_y", tgt_pts[i].second);
    }

    Rigid2D tf;
    if (calib_ok)
    {
      tf = computeRigid2D(src_pts, tgt_pts);
      ROS_INFO("TF Calibration OK: theta=%.4f rad (%.2f deg), tx=%.4f, ty=%.4f",
               tf.theta, tf.theta * 180.0 / M_PI, tf.tx, tf.ty);
    }
    else
    {
      ROS_WARN("TF Calibration: params incomplete, using identity transform");
    }

    blackboard->set("tf.src_to_tgt.theta", tf.theta);
    blackboard->set("tf.src_to_tgt.tx",    tf.tx);
    blackboard->set("tf.src_to_tgt.ty",    tf.ty);
  }

  // Referee 状态默认值——在 ROS 话题数据到来前使用
  blackboard->set("ref.game_progress", uint8_t(0));
  blackboard->set("ref.stage_remain_time", uint16_t(420));
  blackboard->set("ref.ally_base_HP", uint16_t(5000));
  blackboard->set("ref.ally_1_robot_HP", uint16_t(200));
  blackboard->set("ref.ally_2_robot_HP", uint16_t(250));
  blackboard->set("ref.ally_3_robot_HP", uint16_t(150));
  blackboard->set("ref.ally_4_robot_HP", uint16_t(150));
  blackboard->set("ref.infantry_dead", 0);
  blackboard->set("ref.central_elevated_ground_status", uint8_t(0));
  blackboard->set("ref.trapezoidal_elevated_ground_status", uint8_t(0));
  blackboard->set("ref.fortress_status", uint8_t(0));
  blackboard->set("ref.outpost_status", uint8_t(0));
  blackboard->set("ref.ally_outpost_HP", uint16_t(1500));
  blackboard->set("ref.robot_id", uint8_t(7));
  blackboard->set("ref.current_HP", uint16_t(400));
  blackboard->set("ref.projectile_allowance_17mm", uint16_t(300));
  blackboard->set("ref.projectile_allowance_fortress", uint16_t(100));
  blackboard->set("ref.remaining_gold_coin", uint16_t(400));
  blackboard->set("ref.accumulated_bullet_conversion", uint16_t(0));
  blackboard->set("ref.can_exchange_respawn", false);
  blackboard->set("ref.respawn_money", uint16_t(760));
  blackboard->set("ref.out_of_combat", true);
  blackboard->set("ref.projectile_allowance", uint16_t(400));
  blackboard->set("ref.power_rune_available", false);
  blackboard->set("ref.enemy_hero_x", -88.88f);
  blackboard->set("ref.enemy_hero_y", -88.88f);
  blackboard->set("ref.enemy_engineer_x", -88.88f);
  blackboard->set("ref.enemy_engineer_y", -88.88f);
  blackboard->set("ref.enemy_std3_x", -88.88f);
  blackboard->set("ref.enemy_std3_y", -88.88f);
  blackboard->set("ref.enemy_std4_x", -88.88f);
  blackboard->set("ref.enemy_std4_y", -88.88f);
  blackboard->set("ref.enemy_sentry_x", -88.88f);
  blackboard->set("ref.enemy_sentry_y", -88.88f);
  blackboard->set("ref.suggested_target", uint8_t(0));
  blackboard->set("ref.radar_flags", uint16_t(0));
  blackboard->set("ref.enemy_base_HP", uint16_t(5000));
  blackboard->set("ref.operator_x", -88.88f);
  blackboard->set("ref.operator_y", -88.88f);

  // Chase mode initialization
  blackboard->set("chase.target_id", uint8_t(0));
  blackboard->set("chase.target_x", -88.88f);
  blackboard->set("chase.target_y", -88.88f);
  blackboard->set("chase.initialized", false);
  blackboard->set("count.free_bullet", 0);
  
  std::string bt_xml_path;
  pnh.param<std::string>("bt_xml", bt_xml_path, std::string(""));
  if (bt_xml_path.empty())
  {
    //修改路径
    bt_xml_path = ros::package::getPath("decision_node") + "/config/strategy_tree.xml";
  }

  std::ifstream xml_file(bt_xml_path);
  if (!xml_file.is_open())
  {
    ROS_FATAL_STREAM("Failed to open bt_xml file: " << bt_xml_path);
    return 1;
  }
  std::stringstream xml_buffer;
  xml_buffer << xml_file.rdbuf();
  const std::string xml_text = xml_buffer.str();

  BT::Tree tree = factory.createTreeFromText(xml_text, blackboard);

  // ============================================================
  // Groot2 可视化调试支持（ZMQ + 文件日志，V3 协议）
  // ============================================================
  BT::PublisherZMQ publisher(tree, 25, 1666, 1667);
  BT::FileLogger file_logger(tree, "behavior_log.fbl");
  ROS_INFO("ZMQ publisher started on port 1666 (publisher) / 1667 (server), logging to behavior_log.fbl");

  // 初始化时发送一次默认数据到下位机
  {
    std_msgs::UInt8 motion_msg;
    motion_msg.data = 0;   
    motion_pub.publish(motion_msg);
    
    std_msgs::UInt8 recover_msg;
    recover_msg.data = 0;   
    recover_pub.publish(recover_msg);
    
    std_msgs::UInt8 bullet_num_msg;
    bullet_num_msg.data = 0;  
    bullet_num_pub.publish(bullet_num_msg);
    
    ROS_INFO("Initialization: Sent default values - motion=0, recover=0, bullet_num=0");
  }

  ros::Rate rate(std::max(1, tick_hz));
  while (ros::ok())
  {
    ros::spinOnce();
    tree.tickRoot();
    rate.sleep();
  }

  return 0;
}
