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
// 决策图数据结构：保存 YAML 中所有 areas 和 nodes
// ============================================================
struct DecisionGraph
{
    std::vector<std::vector<std::pair<double, double>>> areas;
    std::vector<std::pair<double, double>> nodes;
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

        ROS_INFO("[chase] Loaded %zu areas and %zu nodes from %s",
                 graph.areas.size(), graph.nodes.size(), yaml_path.c_str());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[chase] Failed to load decision graph: %s", e.what());
    }
    return graph;
}

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

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        auto bb = config().blackboard;

        // ---- 1. 读取 suggested_target ----
        uint8_t target_id = 0;
        try {
            target_id = bb->get<uint8_t>("ref.suggested_target");
        } catch (...) {
            ROS_WARN_THROTTLE(2.0, "[SetChaseGoal] ref.suggested_target not in blackboard, using 0");
        }

        // ---- 2. 根据 target_id 读取对应坐标（黑板 int16_t，单位：米） ----
        double target_x = -999.0, target_y = -999.0;
        bool valid = false;

        switch (target_id)
        {
            case 0:  // 英雄
                try {
                    target_x = static_cast<double>(bb->get<int16_t>("ref.enemy_hero_x"));
                    target_y = static_cast<double>(bb->get<int16_t>("ref.enemy_hero_y"));
                    valid = true;
                } catch (...) {}
                break;
            case 1:  // 工程
                try {
                    target_x = static_cast<double>(bb->get<int16_t>("ref.enemy_engineer_x"));
                    target_y = static_cast<double>(bb->get<int16_t>("ref.enemy_engineer_y"));
                    valid = true;
                } catch (...) {}
                break;
            case 2:  // 步兵3
                try {
                    target_x = static_cast<double>(bb->get<int16_t>("ref.enemy_std3_x"));
                    target_y = static_cast<double>(bb->get<int16_t>("ref.enemy_std3_y"));
                    valid = true;
                } catch (...) {}
                break;
            case 3:  // 步兵4
                try {
                    target_x = static_cast<double>(bb->get<int16_t>("ref.enemy_std4_x"));
                    target_y = static_cast<double>(bb->get<int16_t>("ref.enemy_std4_y"));
                    valid = true;
                } catch (...) {}
                break;
            case 4:  // 哨兵
                try {
                    target_x = static_cast<double>(bb->get<int16_t>("ref.enemy_sentry_x"));
                    target_y = static_cast<double>(bb->get<int16_t>("ref.enemy_sentry_y"));
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

    factory.registerBuilder<SetChaseGoal>(
        "SetChaseGoal",
        [](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<SetChaseGoal>(name, config, graph.get());
        });
}
