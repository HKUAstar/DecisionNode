#include "decision_node/chase.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <string>
#include <utility>

// ============================================================
// 射线法：判断点 (px, py) 是否在多边形内部
// ============================================================
static bool pointInPolygon(double px, double py,
                           const std::vector<std::pair<double, double>>& polygon)
{
    int n = static_cast<int>(polygon.size());
    if (n < 3) return false;
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        double xi = polygon[i].first,  yi = polygon[i].second;
        double xj = polygon[j].first,  yj = polygon[j].second;
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
        {
            inside = !inside;
        }
    }
    return inside;
}

// ============================================================
// 决策图数据结构：保存 YAML 中所有 areas, nodes, zones
// ============================================================
struct DecisionGraph
{
    std::vector<std::vector<std::pair<double, double>>> areas;
    std::vector<std::pair<double, double>> nodes;
    std::vector<int> zones;  // zones[i] = 区域编号（与 areas/nodes 索引一一对应）
};

// ============================================================
// 从 YAML 加载决策图
// ============================================================
static DecisionGraph loadDecisionGraph(const std::string& yaml_path)
{
    DecisionGraph graph;
    try
    {
        YAML::Node root = YAML::LoadFile(yaml_path);

        if (root["areas"])
        {
            for (const auto& area_node : root["areas"])
            {
                std::vector<std::pair<double, double>> polygon;
                for (const auto& pt : area_node)
                {
                    double x = pt[0].as<double>();
                    double y = pt[1].as<double>();
                    polygon.emplace_back(x, y);
                }
                graph.areas.push_back(std::move(polygon));
            }
        }

        if (root["nodes"])
        {
            for (const auto& node : root["nodes"])
            {
                double x = node[0].as<double>();
                double y = node[1].as<double>();
                graph.nodes.emplace_back(x, y);
            }
        }

        if (root["zones"])
        {
            for (const auto& z : root["zones"])
            {
                graph.zones.push_back(z.as<int>());
            }
        }

        ROS_INFO("[chase] Loaded %zu areas, %zu nodes and %zu zones from %s",
                 graph.areas.size(), graph.nodes.size(), graph.zones.size(), yaml_path.c_str());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[chase] Failed to load decision graph: %s", e.what());
    }
    return graph;
}

class CheckRadarStatus : public BT::ConditionNode
{
public:
    CheckRadarStatus(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        auto bb = config().blackboard;

        // 1. 读取 suggested_target
        uint8_t target_id = 0;
        try {
            target_id = bb->get<uint8_t>("ref.suggested_target");
        } catch (...) {
            ROS_WARN_THROTTLE(2.0, "[CheckRadarStatus] ref.suggested_target not in blackboard");
            return BT::NodeStatus::FAILURE;
        }

        // 2. 根据 target_id 确定坐标 key
        std::string key_x, key_y;
        const char* target_name = nullptr;

        switch (target_id)
        {
            case 1:  // 英雄
                key_x = "ref.enemy_hero_x";
                key_y = "ref.enemy_hero_y";
                target_name = "hero";
                break;
            case 2:  // 工程
                key_x = "ref.enemy_engineer_x";
                key_y = "ref.enemy_engineer_y";
                target_name = "engineer";
                break;
            case 3:  // 步兵3
                key_x = "ref.enemy_std3_x";
                key_y = "ref.enemy_std3_y";
                target_name = "std3";
                break;
            case 4:  // 步兵4
                key_x = "ref.enemy_std4_x";
                key_y = "ref.enemy_std4_y";
                target_name = "std4";
                break;
            case 7:  // 哨兵
                key_x = "ref.enemy_sentry_x";
                key_y = "ref.enemy_sentry_y";
                target_name = "sentry";
                break;
            default:
                ROS_WARN_THROTTLE(2.0, "[CheckRadarStatus] Unsupported target_id=%d", target_id);
                return BT::NodeStatus::FAILURE;
        }

        // 3. 读取坐标，检查是否为有效值（不是 -88.88）
        float pos_x = -88.88f, pos_y = -88.88f;
        try {
            pos_x = bb->get<float>(key_x);
            pos_y = bb->get<float>(key_y);
        } catch (...) {
            ROS_WARN_THROTTLE(2.0, "[CheckRadarStatus] Failed to read %s / %s from blackboard",
                              key_x.c_str(), key_y.c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (std::abs(pos_x - (-88.88f)) < 0.01f && std::abs(pos_y - (-88.88f)) < 0.01f)
        {
            ROS_INFO_THROTTLE(2.0, "[CheckRadarStatus] Target %s (id=%d) position is invalid (-88.88,-88.88)",
                              target_name, target_id);
            return BT::NodeStatus::FAILURE;
        }

        // 4. 写入黑板供后续 SetChaseGoal 使用
        bb->set("chase.target_id", target_id);

        ROS_INFO_THROTTLE(2.0, "[CheckRadarStatus] Target %s (id=%d) valid at (%.2f, %.2f)",
                          target_name, target_id, pos_x, pos_y);
        return BT::NodeStatus::SUCCESS;
    }
};

class CheckOperatorValid : public BT::ConditionNode
{
public:
    CheckOperatorValid(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        auto bb = config().blackboard;

        // 黑板中 operator_x/y 由通讯节点存入，单位已是 m，无效值为 -88.88
        double op_x = -88.88, op_y = -88.88;
        try {
            op_x = bb->get<double>("ref.operator_x");
            op_y = bb->get<double>("ref.operator_y");
        } catch (...) {
            ROS_WARN_THROTTLE(2.0, "[CheckOperatorValid] ref.operator_x/y not in blackboard");
            return BT::NodeStatus::FAILURE;
        }

        if (std::abs(op_x - (-88.88)) < 0.01 && std::abs(op_y - (-88.88)) < 0.01)
        {
            ROS_INFO_THROTTLE(2.0, "[CheckOperatorValid] Operator position is invalid (-88.88,-88.88)");
            return BT::NodeStatus::FAILURE;
        }

        ROS_INFO_THROTTLE(2.0, "[CheckOperatorValid] Operator position valid at (%.2f, %.2f)", op_x, op_y);
        return BT::NodeStatus::SUCCESS;
    }
};

class SetOperatorGoal : public BT::SyncActionNode
{
public:
    SetOperatorGoal(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        auto bb = config().blackboard;

        double op_x = -88.88, op_y = -88.88;
        try {
            op_x = bb->get<double>("ref.operator_x");
            op_y = bb->get<double>("ref.operator_y");
        } catch (...) {
            ROS_WARN_THROTTLE(2.0, "[SetOperatorGoal] ref.operator_x/y not in blackboard");
            return BT::NodeStatus::FAILURE;
        }

        if (std::abs(op_x - (-88.88)) < 0.01 && std::abs(op_y - (-88.88)) < 0.01)
        {
            ROS_WARN_THROTTLE(2.0, "[SetOperatorGoal] Operator position is invalid (-88.88,-88.88)");
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::PointStamped goal;
        goal.header.stamp    = ros::Time::now();
        goal.header.frame_id = "map";
        goal.point.x = op_x;
        goal.point.y = op_y;
        goal.point.z = 0.0;

        bb->set("goal.point", goal);
        bb->set("goal.valid", true);

        ROS_INFO_THROTTLE(2.0, "[SetOperatorGoal] goal=(%.2f, %.2f)", op_x, op_y);
        return BT::NodeStatus::SUCCESS;
    }
};

// ============================================================
// BT Node: SetChaseGoal
//   根据 ref.suggested_target 选择追击目标，判断所在区域，
//   将对应 node 坐标写入 goal.point
// ============================================================
class SetChaseGoal : public BT::SyncActionNode
{
public:
    SetChaseGoal(const std::string& name, const BT::NodeConfiguration& config,
                 const DecisionGraph* graph)
        : BT::SyncActionNode(name, config), graph_(graph)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("zone_num", -1, "Target zone (from YAML zones list). -1 = disable zone filter."),
        };
    }

    BT::NodeStatus tick() override
    {
        auto bb = config().blackboard;

        // ---- 0. 读取 zone_num ----
        int zone_num = -1;
        (void)getInput("zone_num", zone_num);

        // ---- 1. 读取 suggested_target ----
        uint8_t target_id = 0;
        try {
            target_id = bb->get<uint8_t>("ref.suggested_target");
        } catch (...) {
            ROS_WARN_THROTTLE(2.0, "[SetChaseGoal] ref.suggested_target not in blackboard, using 0");
        }

        // ---- 2. 根据 target_id 读取对应坐标（黑板 float，单位：m） ----
        float target_x = -99.0f, target_y = -99.0f;
        bool valid = false;

        switch (target_id)
        {
            case 0:  // 英雄
                try {
                    target_x = bb->get<float>("ref.enemy_hero_x");
                    target_y = bb->get<float>("ref.enemy_hero_y");
                    valid = true;
                } catch (...) {}
                break;
            case 1:  // 工程
                try {
                    target_x = bb->get<float>("ref.enemy_engineer_x");
                    target_y = bb->get<float>("ref.enemy_engineer_y");
                    valid = true;
                } catch (...) {}
                break;
            case 2:  // 步兵3
                try {
                    target_x = bb->get<float>("ref.enemy_std3_x");
                    target_y = bb->get<float>("ref.enemy_std3_y");
                    valid = true;
                } catch (...) {}
                break;
            case 3:  // 步兵4
                try {
                    target_x = bb->get<float>("ref.enemy_std4_x");
                    target_y = bb->get<float>("ref.enemy_std4_y");
                    valid = true;
                } catch (...) {}
                break;
            case 4:  // 哨兵
                try {
                    target_x = bb->get<float>("ref.enemy_sentry_x");
                    target_y = bb->get<float>("ref.enemy_sentry_y");
                    valid = true;
                } catch (...) {}
                break;
            default:
                ROS_WARN("[SetChaseGoal] Unknown target_id=%d", target_id);
                return BT::NodeStatus::FAILURE;
        }

        if (!valid)
        {
            ROS_WARN_THROTTLE(2.0, "[SetChaseGoal] Target %d coordinates not available", target_id);
            return BT::NodeStatus::FAILURE;
        }

        // ---- 3. 判断落在哪个 area 内 ----
        int area_idx = -1;
        for (size_t i = 0; i < graph_->areas.size(); ++i)
        {
            if (pointInPolygon(target_x, target_y, graph_->areas[i]))
            {
                area_idx = static_cast<int>(i);
                break;
            }
        }

        if (area_idx < 0)
        {
            ROS_WARN_THROTTLE(2.0, "[SetChaseGoal] Target %d at (%.2f, %.2f) not in any area",
                              target_id, target_x, target_y);
            return BT::NodeStatus::FAILURE;
        }

        // ---- 3.5. 检查 zone ----
        if (zone_num >= 0 && area_idx < static_cast<int>(graph_->zones.size()))
        {
            if (graph_->zones[area_idx] != zone_num)
            {
                ROS_WARN_THROTTLE(2.0,
                    "[SetChaseGoal] Target %d at area=%d belongs to zone=%d, but requested zone_num=%d",
                    target_id, area_idx, graph_->zones[area_idx], zone_num);
                return BT::NodeStatus::FAILURE;
            }
        }

        // ---- 4. 取对应 node 坐标，写入 goal.point ----
        if (area_idx >= static_cast<int>(graph_->nodes.size()))
        {
            ROS_ERROR("[SetChaseGoal] Area %d has no corresponding node (%zu nodes total)",
                      area_idx, graph_->nodes.size());
            return BT::NodeStatus::FAILURE;
        }

        double node_x = graph_->nodes[area_idx].first;
        double node_y = graph_->nodes[area_idx].second;

        geometry_msgs::PointStamped goal;
        goal.header.stamp    = ros::Time::now();
        goal.header.frame_id = "map";
        goal.point.x = node_x;
        goal.point.y = node_y;
        goal.point.z = 0.0;

        bb->set("goal.point", goal);
        bb->set("goal.valid", true);

        ROS_INFO("[SetChaseGoal] target_id=%d at area=%d, goal=(%.2f, %.2f)",
                 target_id, area_idx, node_x, node_y);

        return BT::NodeStatus::SUCCESS;
    }

private:
    const DecisionGraph* graph_;
};

// ============================================================
// 注册到 BehaviorTreeFactory
// ============================================================
void RegisterChaseNodes(BT::BehaviorTreeFactory& factory,
                        ros::Publisher* /*goal_pub*/,
                        bool* /*publish_on_change_only*/)
{
    // 加载决策图（单例，所有 SetChaseGoal 节点共享同一份数据）
    std::string yaml_path = ros::package::getPath("decision_node")
                            + "/map/RMUC2026_decision_graph.yaml";
    static auto graph = std::make_shared<DecisionGraph>(loadDecisionGraph(yaml_path));

    factory.registerNodeType<CheckRadarStatus>("CheckRadarStatus");
    factory.registerNodeType<CheckOperatorValid>("CheckOperatorValid");
    factory.registerNodeType<SetOperatorGoal>("SetOperatorGoal");

    factory.registerBuilder<SetChaseGoal>(
        "SetChaseGoal",
        [](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<SetChaseGoal>(name, config, graph.get());
        });
}
