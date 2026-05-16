/**
 * visualize.cpp - 可视化决策图
 * 读取 RMUC2026_decision_graph.yaml 中的 nodes / edges / zones，
 * 发布 visualization_msgs::Marker 到 Rviz，支持分 zone 着色
 */
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <utility>

// ============================================================
// 决策图数据
// ============================================================
struct DecisionGraph
{
    std::vector<std::vector<std::pair<double, double>>> areas;
    std::vector<std::pair<double, double>> nodes;
    std::vector<int> zones;
    std::vector<std::pair<int, int>> edges;  // (from_node_idx, to_node_idx)
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

        if (root["edges"])
        {
            for (const auto& edge : root["edges"])
            {
                int from = edge[0].as<int>();
                int to   = edge[1].as<int>();
                graph.edges.emplace_back(from, to);
            }
        }

        if (root["zones"])
        {
            for (const auto& z : root["zones"])
            {
                graph.zones.push_back(z.as<int>());
            }
        }

        ROS_INFO("[visualize] Loaded %zu nodes, %zu edges, %zu zones, %zu areas from %s",
                 graph.nodes.size(), graph.edges.size(),
                 graph.zones.size(), graph.areas.size(), yaml_path.c_str());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[visualize] Failed to load decision graph: %s", e.what());
    }
    return graph;
}

// ============================================================
// 发布决策图到 Rviz
// ============================================================
class DecisionGraphVisualizer
{
public:
    DecisionGraphVisualizer()
        : nh_("~")
    {
        pub_nodes_  = nh_.advertise<visualization_msgs::Marker>("graph_nodes", 1, true);
        pub_edges_  = nh_.advertise<visualization_msgs::Marker>("graph_edges", 1, true);
        pub_labels_ = nh_.advertise<visualization_msgs::Marker>("graph_labels", 1, true);
        pub_areas_  = nh_.advertise<visualization_msgs::Marker>("graph_areas", 1, true);

        std::string yaml_path;
        nh_.param("yaml_path", yaml_path,
                  ros::package::getPath("decision_node") + "/map/RMUC2026_decision_graph.yaml");

        graph_ = loadDecisionGraph(yaml_path);
    }

    void publish()
    {
        publishNodes();
        publishEdges();
        publishLabels();
        publishAreas();
        ROS_INFO("[visualize] Published all markers to /graph_*");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_nodes_;
    ros::Publisher pub_edges_;
    ros::Publisher pub_labels_;
    ros::Publisher pub_areas_;
    DecisionGraph graph_;

    // ---------- 节点（球体） ----------
    void publishNodes()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp    = ros::Time::now();
        marker.ns     = "decision_graph";
        marker.id     = 0;
        marker.type   = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;

        for (size_t i = 0; i < graph_.nodes.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = graph_.nodes[i].first;
            p.y = graph_.nodes[i].second;
            p.z = 0.0;
            marker.points.push_back(p);

            // 按 zone 着色
            int zone = (i < graph_.zones.size()) ? graph_.zones[i] : -1;
            std_msgs::ColorRGBA c;
            switch (zone)
            {
                case 0:  // zone 0 → 蓝色
                    c.r = 0.2f; c.g = 0.4f; c.b = 1.0f; c.a = 0.9f; break;
                case 1:  // zone 1 → 绿色
                    c.r = 0.2f; c.g = 0.9f; c.b = 0.3f; c.a = 0.9f; break;
                case 2:  // zone 2 → 橙色
                    c.r = 1.0f; c.g = 0.6f; c.b = 0.1f; c.a = 0.9f; break;
                case -1: // 无效 → 灰色
                default:
                    c.r = 0.5f; c.g = 0.5f; c.b = 0.5f; c.a = 0.5f; break;
            }
            marker.colors.push_back(c);
        }

        pub_nodes_.publish(marker);
    }

    // ---------- 边（线条） ----------
    void publishEdges()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp    = ros::Time::now();
        marker.ns     = "decision_graph";
        marker.id     = 1;
        marker.type   = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.03;  // 线宽
        marker.color.a = 0.6f;
        marker.color.r = 0.8f;
        marker.color.g = 0.8f;
        marker.color.b = 0.8f;

        for (size_t i = 0; i < graph_.edges.size(); ++i)
        {
            int from = graph_.edges[i].first;
            int to   = graph_.edges[i].second;

            if (from >= (int)graph_.nodes.size() || to >= (int)graph_.nodes.size())
                continue;

            geometry_msgs::Point p1, p2;
            p1.x = graph_.nodes[from].first;
            p1.y = graph_.nodes[from].second;
            p1.z = 0.0;
            p2.x = graph_.nodes[to].first;
            p2.y = graph_.nodes[to].second;
            p2.z = 0.0;

            marker.points.push_back(p1);
            marker.points.push_back(p2);
        }

        pub_edges_.publish(marker);
    }

    // ---------- 标签（节点 ID + zone） ----------
    void publishLabels()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp    = ros::Time::now();
        marker.ns     = "decision_graph";
        marker.id     = 2;
        marker.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.z = 0.2;  // 文字高度
        marker.color.a = 1.0f;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;

        for (size_t i = 0; i < graph_.nodes.size(); ++i)
        {
            marker.id = static_cast<int>(i + 1000);
            marker.pose.position.x = graph_.nodes[i].first;
            marker.pose.position.y = graph_.nodes[i].second;
            marker.pose.position.z = 0.15;
            marker.pose.orientation.w = 1.0;

            int zone = (i < graph_.zones.size()) ? graph_.zones[i] : -1;
            marker.text = std::to_string(i) + " (z" + std::to_string(zone) + ")";

            pub_labels_.publish(marker);
        }
    }

    // ---------- 多边形区域（半透明） ----------
    void publishAreas()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp    = ros::Time::now();
        marker.ns     = "decision_graph";
        marker.id     = 3;
        marker.type   = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.02;
        marker.color.a = 0.4f;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.3f;

        for (size_t i = 0; i < graph_.areas.size(); ++i)
        {
            const auto& poly = graph_.areas[i];
            if (poly.size() < 2) continue;

            // 按 zone 着色
            int zone = (i < graph_.zones.size()) ? graph_.zones[i] : -1;
            std_msgs::ColorRGBA c;
            switch (zone)
            {
                case 0:  c.r = 0.3f; c.g = 0.6f; c.b = 1.0f; c.a = 0.3f; break;
                case 1:  c.r = 0.3f; c.g = 1.0f; c.b = 0.4f; c.a = 0.3f; break;
                case 2:  c.r = 1.0f; c.g = 0.7f; c.b = 0.2f; c.a = 0.3f; break;
                default: c.r = 0.5f; c.g = 0.5f; c.b = 0.5f; c.a = 0.2f; break;
            }

            for (size_t j = 0; j < poly.size(); ++j)
            {
                size_t k = (j + 1) % poly.size();
                geometry_msgs::Point p1, p2;
                p1.x = poly[j].first;  p1.y = poly[j].second;  p1.z = -0.01;
                p2.x = poly[k].first;  p2.y = poly[k].second;  p2.z = -0.01;

                marker.points.push_back(p1);
                marker.points.push_back(p2);
                marker.colors.push_back(c);
                marker.colors.push_back(c);
            }
        }

        pub_areas_.publish(marker);
    }
};

// ============================================================
// Main
// ============================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "decision_visualizer");
    DecisionGraphVisualizer visualizer;

    // 发布一次即可（latch=true publisher 会保留最后一条消息）
    visualizer.publish();

    // 也可以定期重发（如每 5 秒），防止 Rviz 重启后丢失
    ros::Rate rate(0.2);  // 0.2 Hz = 每 5 秒
    while (ros::ok())
    {
        visualizer.publish();
        rate.sleep();
    }

    return 0;
}
