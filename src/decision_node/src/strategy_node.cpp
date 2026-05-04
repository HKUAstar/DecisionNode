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
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <ros/package.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
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

}  // namespace

// ---------------------------
// Shared state (ROS callbacks)
// ---------------------------
struct RefereeState
{
  float yaw_angle;          // 云台yaw角 (rad)
  
  uint8_t game_progress;    // 比赛阶段
  uint16_t stage_remain_time;

  uint16_t ally_base_HP;    //基地血量

  
  uint8_t central_elevated_ground_status; // 中央高地状态（bit 7-8）
  uint8_t trapezoidal_elevated_ground_status; // 梯形高地状态（bit 9-10）
  uint8_t fortress_status; // 堡垒状态（bit 25-26）
  uint8_t outpost_status; // 前哨战状态（bit 27-28）

  uint8_t robot_id;         // 机器人ID
  uint16_t current_HP;

  uint16_t projectile_allowance_17mm;
  uint16_t projectile_allowance_fortress;
  uint16_t remaining_gold_coin;

  uint16_t accumulated_bullet_conversion; // 累计哨兵远程兑换弹量（bit 0-10）
  bool can_exchange_respawn;     // 哨兵是否可兑换复活（bit 20）
  uint16_t respawn_money; // 哨兵复活所需金币（bit 21-31）

  bool out_of_combat;       // 脱战状态（bit 0）
  uint16_t projectile_allowance; //全队可兑换17mm弹量（bit 1-11）
  bool power_rune_available; // 是否有可用的能量符（bit 14)

  int16_t enemy_hero_x;     // 敌方英雄X (cm)
  int16_t enemy_hero_y;     // 敌方英雄Y (cm)
  int16_t enemy_engineer_x; // 敌方工程X (cm)
  int16_t enemy_engineer_y; // 敌方工程Y (cm)

  int16_t enemy_std3_x;     // 敌方步兵3 X 
  int16_t enemy_std3_y;     // 敌方步兵3 Y 
  int16_t enemy_std4_x;     // 敌方步兵4 X 
  int16_t enemy_std4_y;     // 敌方步兵4 Y 

  int16_t enemy_sentry_x;   // 敌方哨兵X 
  int16_t enemy_sentry_y;   // 敌方哨兵Y 
  uint8_t suggested_target; // 雷达建议目标
  uint16_t radar_flags;     // 雷达标记信息

  uint16_t enemy_base_HP;     //敌方基地血量
  




};

struct NavigationState
{
  bool arrived = false;
};

struct OdomState
{
  double gimbal_angle = 0.0;  // 世界系中的yaw角（弧度）
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;  // 四元数
  double yaw_angle = 0.0;  // MCU上报的云台yaw角（弧度）
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
    bb->set("ref.central_elevated_ground_status", state_->central_elevated_ground_status);
    bb->set("ref.trapezoidal_elevated_ground_status", state_->trapezoidal_elevated_ground_status);
    bb->set("ref.fortress_status", state_->fortress_status);
    bb->set("ref.outpost_status", state_->outpost_status);
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

class UpdateVisionBB : public BT::SyncActionNode
{
public:
  UpdateVisionBB(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    // V1: explicitly弃用视觉模块。这里保持节点存在，但不写入任何视觉字段。[TODO] 后续可接真实视觉数据
    return BT::NodeStatus::SUCCESS;
  }
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
    // 预留：[TODO] 后续可把 Strategy_Task.c 里的计数器/超时机制迁移到这里
    return BT::NodeStatus::SUCCESS;
  }
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
  int last_hp_ = -1;
  std::deque<std::pair<ros::Time, int>> damage_history_;
};

class IntenseHarm : public BT::ConditionNode
{
public:
  IntenseHarm(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), is_active_(false)
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
    int damage_2s = 0;
    try {
        damage_2s = bb->get<int>("derived.damage_2s");
    } catch (...) {
        damage_2s = 0;
    }

    int t_on = 100;
    int t_off = 50;
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
    const bool is_dead = config().blackboard->get<bool>("is_dead");
    return is_dead ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
    const bool is_dead = config().blackboard->get<bool>("is_dead");
    return (!is_dead) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const bool bullet_sufficient = config().blackboard->get<bool>("bullet_sufficient");
    return (!bullet_sufficient) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
    // ROS_INFO("[SetAction] action set to %s", toUpper(action).c_str());
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
class Wait : public BT::SyncActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), wait_end_time_(ros::Time(0))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", 1.0, "等待的秒数"),
    };
  }

  BT::NodeStatus tick() override
  {
    double duration = 1.0;
    if (!getInput("duration", duration))
    {
      return BT::NodeStatus::FAILURE;
    }

    auto bb = config().blackboard;
    
    // 第一次进入时，记录开始时间
    if (wait_end_time_ == ros::Time(0))
    {
      wait_end_time_ = ros::Time::now() + ros::Duration(duration);
      ROS_DEBUG("Wait: Starting wait for %.2f seconds", duration);
      return BT::NodeStatus::RUNNING;
    }

    // 检查是否超过等待时间
    if (ros::Time::now() >= wait_end_time_)
    {
      ROS_DEBUG("Wait: Wait completed");
      wait_end_time_ = ros::Time(0);  // 重置计时器
      return BT::NodeStatus::SUCCESS;
    }

    // 还没等够，继续等待
    return BT::NodeStatus::RUNNING;
  }

private:
  ros::Time wait_end_time_;
};

//[TODO]: 这里的目标点是从参数服务器读取的，
class SetGoalFromParams : public BT::SyncActionNode
{
public:
  SetGoalFromParams(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle* nh)
    : BT::SyncActionNode(name, config), nh_(nh)
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

    const std::string base = std::string("goals/") + ns;

    double x = 0.0, y = 0.0;
    (void)nh_->param(base + "/x", x, 0.0);
    (void)nh_->param(base + "/y", y, 0.0);

    ROS_DEBUG("SetGoalFromParams: ns=%s, goal=(%f, %f)", ns.c_str(), x, y);

    geometry_msgs::PointStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.point.x = x;
    goal.point.y = y;
    goal.point.z = 0.0;

    auto bb = config().blackboard;
    bb->set("goal.point", goal);
    bb->set("goal.valid", true);  
    ROS_DEBUG("SetGoalFromParams: goal.valid set to TRUE");
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle* nh_;
};

class SetGoalFromParamsCyclic : public BT::SyncActionNode
{
public:
  SetGoalFromParamsCyclic(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle* nh)
    : BT::SyncActionNode(name, config), nh_(nh)
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

    // per-ns 独立索引 key，避免多个 action 互相干扰
    const std::string idx_key        = "cycle_idx_" + ns;
    const std::string last_action_key = "cycle_last_action_" + ns;

    // 检测 action 是否切换 → 切换则将该 ns 的索引重置为 0
    std::string cur_action, last_action;
    try { cur_action  = bb->get<std::string>("action"); }       catch (...) {}
    try { last_action = bb->get<std::string>(last_action_key); } catch (...) {}
    if (cur_action != last_action)
    {
      bb->set(idx_key, 0);
      bb->set(last_action_key, cur_action);
      ROS_DEBUG("SetGoalFromParamsCyclic[%s]: action changed (%s->%s), resetting index",
                ns.c_str(), last_action.c_str(), cur_action.c_str());
    }

    int cycle_index = 0;
    try {
      cycle_index = bb->get<int>(idx_key);
    } catch (...) {
      bb->set(idx_key, 0);
    }

    // Read goal from parameters: goals/<ns>/point_<index>/{x,y}
    const std::string base = std::string("goals/") + ns + "/point_" + std::to_string(cycle_index);

    double x = 0.0, y = 0.0;
    (void)nh_->param(base + "/x", x, 0.0);
    (void)nh_->param(base + "/y", y, 0.0);

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
  ros::NodeHandle* nh_;
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
    // ROS_INFO("PublishGoalPoint: goal.valid=%d", valid);
    if (!valid)
    {
      // ROS_INFO("PublishGoalPoint: goal.valid is FALSE, skipping publish");
      return BT::NodeStatus::SUCCESS;
    }

    const auto goal = bb->get<geometry_msgs::PointStamped>("goal.point");
    // ROS_INFO("PublishGoalPoint: goal position=(%f, %f)", goal.point.x, goal.point.y);

    if (*publish_on_change_only_)
    {
      // crude de-dup: compare x/y only
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
        // ROS_INFO("PublishGoalPoint: Position unchanged, skipping publish");
        return BT::NodeStatus::SUCCESS;
      }
      bb->set("goal.last_x", goal.point.x);
      bb->set("goal.last_y", goal.point.y);
    }

    // ROS_INFO("PublishGoalPoint: Publishing goal at (%f, %f)", goal.point.x, goal.point.y);
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

  auto blackboard = BT::Blackboard::create();
  
  // Odom 订阅：获取世界系中的四元数，计算yaw角
  auto sub_odom = nh.subscribe<nav_msgs::Odometry>("/odom", 1, [&](const nav_msgs::Odometry::ConstPtr& msg) {
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

  // 敌方位置（geometry_msgs::Point → int16_t cm，注意单位转换 m → cm）
  auto sub_enemy_hero = nh.subscribe<geometry_msgs::Point>("/enemy/hero_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_hero_x = static_cast<int16_t>(msg->x * 100.0f);
    ref.enemy_hero_y = static_cast<int16_t>(msg->y * 100.0f);
  });
  auto sub_enemy_eng = nh.subscribe<geometry_msgs::Point>("/enemy/engineer_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_engineer_x = static_cast<int16_t>(msg->x * 100.0f);
    ref.enemy_engineer_y = static_cast<int16_t>(msg->y * 100.0f);
  });
  auto sub_enemy_std3 = nh.subscribe<geometry_msgs::Point>("/enemy/standard_3_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_std3_x = static_cast<int16_t>(msg->x * 100.0f);
    ref.enemy_std3_y = static_cast<int16_t>(msg->y * 100.0f);
  });
  auto sub_enemy_std4 = nh.subscribe<geometry_msgs::Point>("/enemy/standard_4_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_std4_x = static_cast<int16_t>(msg->x * 100.0f);
    ref.enemy_std4_y = static_cast<int16_t>(msg->y * 100.0f);
  });
  auto sub_enemy_sentry = nh.subscribe<geometry_msgs::Point>("/enemy/sentry_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_sentry_x = static_cast<int16_t>(msg->x * 100.0f);
    ref.enemy_sentry_y = static_cast<int16_t>(msg->y * 100.0f);
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

  // Navigation arrived (复用现有语义)
  auto sub_arrived = nh.subscribe<std_msgs::Bool>("/dstar_status", 10, 
    [&](const std_msgs::Bool::ConstPtr& msg) {
      nav.arrived = msg->data;
    });

  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 1);
  ros::Publisher motion_pub = nh.advertise<std_msgs::UInt8>("motion", 1);
  ros::Publisher spin_pub = nh.advertise<std_msgs::UInt8>("spin", 1);
  ros::Publisher recover_pub = nh.advertise<std_msgs::UInt8>("recover", 1);
  ros::Publisher bullet_up_pub = nh.advertise<std_msgs::UInt8>("bullet_up", 1);
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

  factory.registerNodeType<UpdateVisionBB>("UpdateVisionBB");
  factory.registerNodeType<UpdateTimersBB>("UpdateTimersBB");
  factory.registerNodeType<UpdateDerivedFlags>("UpdateDerivedFlags");

  factory.registerNodeType<IsGameStarted>("IsGameStarted");
  factory.registerNodeType<IsSentryDead>("IsSentryDead");
  factory.registerNodeType<IsSentryAlive>("IsSentryAlive");
  factory.registerNodeType<IsSentryInDanger>("IsSentryInDanger");
  factory.registerNodeType<IntenseHarm>("IntenseHarm");
  factory.registerNodeType<NotBulletSufficient>("NotBulletSufficient");
  factory.registerNodeType<IsAction>("IsAction");

  factory.registerNodeType<SetAction>("SetAction");
  factory.registerNodeType<ClearGoal>("ClearGoal");
  factory.registerNodeType<Wait>("Wait");
  
  RegisterMotionChangeNodes(factory, &motion_pub, &spin_pub, &publish_on_change_only);

  factory.registerBuilder<SetGoalFromParams>(
    "SetGoalFromParams", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<SetGoalFromParams>(name, config, &nh);
    });

  factory.registerBuilder<SetGoalFromParamsCyclic>(
    "SetGoalFromParamsCyclic", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<SetGoalFromParamsCyclic>(name, config, &nh);
    });

  factory.registerNodeType<AdvanceCycleIndex>("AdvanceCycleIndex");

  factory.registerBuilder<PublishGoalPoint>(
    "PublishGoalPoint", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<PublishGoalPoint>(name, config, &goal_pub, &publish_on_change_only);
    });

  RegisterRecoverChangeNodes(factory, &recover_pub, &bullet_up_pub);
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

  // Chase mode initialization
  blackboard->set("chase.target_id", uint8_t(0));
  blackboard->set("chase.target_x", 0.0f);
  blackboard->set("chase.target_y", 0.0f);
  blackboard->set("chase.initialized", false);
  
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

  // 初始化时发送一次默认数据到下位机
  {
    std_msgs::UInt8 motion_msg;
    motion_msg.data = 0;   
    motion_pub.publish(motion_msg);
    
    std_msgs::UInt8 recover_msg;
    recover_msg.data = 0;   
    recover_pub.publish(recover_msg);
    
    std_msgs::UInt8 bullet_up_msg;
    bullet_up_msg.data = 0;  
    bullet_up_pub.publish(bullet_up_msg);
    
    std_msgs::UInt8 bullet_num_msg;
    bullet_num_msg.data = 0;  
    bullet_num_pub.publish(bullet_num_msg);
    
    ROS_INFO("Initialization: Sent default values - motion=0, recover=0, bullet_up=0, bullet_num=0");
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
