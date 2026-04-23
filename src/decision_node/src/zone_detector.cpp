/**
 * zone_detector.cpp
 *
 * 功能：订阅哨兵世界坐标（geometry_msgs::PointStamped），
 *       根据手动绘制的 PGM 区域地图（不同灰度值 = 不同区域），
 *       发布哨兵所在区域编号（std_msgs::Int32）。
 *
 * ROS 参数（均带 ~ 私有命名空间）：
 *   ~map_yaml        : PGM 对应的 yaml 配置文件路径（默认 "map.yaml"）
 *   ~position_topic  : 输入坐标话题（默认 "/sentinel_nav_pos"）
 *   ~zone_topic      : 输出区域话题（默认 "/sentinel_zone"）
 *   ~zone_map        : 自定义像素值→区域号映射列表（可选，见下文）
 *
 * zone_map 参数示例（launch 文件中）：
 *   <rosparam param="zone_map">
 *     - {pixel: 255, zone: 0}   # 白色：自由区域 / 默认区
 *     - {pixel: 200, zone: 1}   # 浅灰：区域 1
 *     - {pixel: 150, zone: 2}   # 中灰：区域 2
 *     - {pixel: 100, zone: 3}   # 深灰：区域 3
 *     - {pixel:   0, zone: -1}  # 黑色：障碍物
 *   </rosparam>
 *
 * 若不提供 ~zone_map，节点会自动从 PGM 图像中检测所有不同灰度值，
 * 按升序依次分配 0、1、2… 作为区域编号，并在启动时打印映射表。
 *
 * 编译依赖：roscpp  std_msgs  geometry_msgs  yaml-cpp
 * 无需 OpenCV（自带轻量 P2/P5 PGM 解析器）。
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <string>
#include <stdexcept>
#include <cctype>
#include <cstdint>

// ===========================================================================
// 轻量 PGM (P2/P5) 读取器
// ===========================================================================

struct PGMImage {
    int width  = 0;
    int height = 0;
    int max_val = 255;
    std::vector<uint8_t> data;  // 行优先，data[row * width + col]

    uint8_t at(int row, int col) const {
        return data[static_cast<size_t>(row) * width + col];
    }
};

// 跳过 PGM 头部的空白字符及注释行
static void skipWsAndComments(std::ifstream& f) {
    while (f.good()) {
        int c = f.peek();
        if (c == '#') {
            std::string line;
            std::getline(f, line);
        } else if (std::isspace(c)) {
            f.get();
        } else {
            break;
        }
    }
}

static bool loadPGM(const std::string& path, PGMImage& img) {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) {
        ROS_ERROR("[zone_detector] Cannot open PGM file: %s", path.c_str());
        return false;
    }

    // 读魔数
    std::string magic;
    f >> magic;
    if (magic != "P2" && magic != "P5") {
        ROS_ERROR("[zone_detector] Unsupported PGM magic: '%s'. Only P2/P5 supported.", magic.c_str());
        return false;
    }

    // 读宽、高、最大值
    skipWsAndComments(f); f >> img.width;
    skipWsAndComments(f); f >> img.height;
    skipWsAndComments(f); f >> img.max_val;

    if (img.width <= 0 || img.height <= 0 || img.max_val <= 0) {
        ROS_ERROR("[zone_detector] Invalid PGM dimensions or max_val.");
        return false;
    }

    // 魔数后恰好一个空白字符
    char dummy;
    f.get(dummy);

    img.data.resize(static_cast<size_t>(img.width) * img.height);

    if (magic == "P5") {
        // 二进制模式
        if (img.max_val <= 255) {
            f.read(reinterpret_cast<char*>(img.data.data()), img.data.size());
        } else {
            // 16-bit P5（max_val > 255），取高字节简化
            for (auto& px : img.data) {
                uint8_t hi, lo;
                f.read(reinterpret_cast<char*>(&hi), 1);
                f.read(reinterpret_cast<char*>(&lo), 1);
                px = hi;
            }
        }
        if (!f) {
            ROS_ERROR("[zone_detector] Unexpected EOF while reading P5 binary data.");
            return false;
        }
    } else {
        // ASCII 模式 (P2)
        for (auto& px : img.data) {
            int v = 0;
            f >> v;
            px = static_cast<uint8_t>(v);
        }
        if (!f) {
            ROS_ERROR("[zone_detector] Unexpected EOF while reading P2 ASCII data.");
            return false;
        }
    }

    return true;
}

// ===========================================================================
// 地图 YAML 配置
// ===========================================================================

struct MapConfig {
    std::string image_path;         // PGM 图像绝对路径
    double      resolution  = 0.05; // 米/像素
    double      origin_x    = 0.0;  // 地图原点 x（米）
    double      origin_y    = 0.0;  // 地图原点 y（米）
    int         negate      = 0;    // 0=黑色为障碍，1=白色为障碍
    double      occupied_thresh = 0.65;
    double      free_thresh     = 0.196;
};

static bool loadMapConfig(const std::string& yaml_path, MapConfig& cfg) {
    try {
        YAML::Node node = YAML::LoadFile(yaml_path);

        std::string image_file = node["image"].as<std::string>();

        // 若图像路径为相对路径，则相对于 yaml 文件所在目录
        if (!image_file.empty() && image_file[0] != '/') {
            auto sep = yaml_path.find_last_of("/\\");
            std::string dir = (sep != std::string::npos)
                              ? yaml_path.substr(0, sep + 1) : "./";
            cfg.image_path = dir + image_file;
        } else {
            cfg.image_path = image_file;
        }

        cfg.resolution = node["resolution"].as<double>();
        auto origin    = node["origin"].as<std::vector<double>>();
        cfg.origin_x   = origin.at(0);
        cfg.origin_y   = origin.at(1);

        if (node["negate"])          cfg.negate          = node["negate"].as<int>();
        if (node["occupied_thresh"]) cfg.occupied_thresh = node["occupied_thresh"].as<double>();
        if (node["free_thresh"])     cfg.free_thresh     = node["free_thresh"].as<double>();

        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("[zone_detector] Failed to parse YAML '%s': %s", yaml_path.c_str(), e.what());
        return false;
    }
}

// ===========================================================================
// 区域检测节点
// ===========================================================================

class ZoneDetectorNode {
public:
    ZoneDetectorNode() : nh_("~") {

        // ---------- ROS 参数 ----------
        std::string yaml_path, pos_topic, zone_topic;
        nh_.param<std::string>("map_yaml",       yaml_path,  "config/zone_map/zone_map.yaml");
        nh_.param<std::string>("position_topic", pos_topic,  "/sentinel_nav_pos");
        nh_.param<std::string>("zone_topic",     zone_topic, "/sentinel_zone");

        // ---------- 加载地图 ----------
        if (!loadMapConfig(yaml_path, map_cfg_)) {
            ROS_FATAL("[zone_detector] Map config load failed. Shutting down.");
            ros::shutdown();
            return;
        }
        if (!loadPGM(map_cfg_.image_path, pgm_)) {
            ROS_FATAL("[zone_detector] PGM load failed. Shutting down.");
            ros::shutdown();
            return;
        }

        ROS_INFO("[zone_detector] Map loaded: %s  [%d x %d px]  %.4f m/px  origin=(%.2f, %.2f)",
                 map_cfg_.image_path.c_str(),
                 pgm_.width, pgm_.height,
                 map_cfg_.resolution,
                 map_cfg_.origin_x, map_cfg_.origin_y);

        // ---------- 建立像素值→区域号映射 ----------
        buildZoneMap();
        printZoneMap();

        // ---------- 发布 / 订阅 ----------
        zone_pub_ = nh_.advertise<std_msgs::Int32>(zone_topic, 10);
        pos_sub_  = nh_.subscribe(pos_topic, 10, &ZoneDetectorNode::posCallback, this);

        ROS_INFO("[zone_detector] Ready. sub='%s'  pub='%s'",
                 pos_topic.c_str(), zone_topic.c_str());
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher  zone_pub_;
    ros::Subscriber pos_sub_;

    MapConfig map_cfg_;
    PGMImage  pgm_;

    // 像素灰度值 → 区域编号
    std::map<uint8_t, int> zone_map_;

    // ------------------------------------------------------------------
    // 构建 zone_map_：优先使用 ROS 参数，否则自动检测
    // ------------------------------------------------------------------
    void buildZoneMap() {
        if (nh_.hasParam("zone_map")) {
            XmlRpc::XmlRpcValue list;
            nh_.getParam("zone_map", list);
            if (list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (int i = 0; i < list.size(); ++i) {
                    int pv = static_cast<int>(list[i]["pixel"]);
                    int zn = static_cast<int>(list[i]["zone"]);
                    zone_map_[static_cast<uint8_t>(pv)] = zn;
                }
                ROS_INFO("[zone_detector] Zone map from ~zone_map param: %zu entries.",
                         zone_map_.size());
                return;
            }
        }

        // 自动检测：从图像中收集所有不同灰度值，按升序编号
        std::set<uint8_t> distinct(pgm_.data.begin(), pgm_.data.end());
        int zone_id = 0;
        for (uint8_t v : distinct) {
            zone_map_[v] = zone_id++;
        }
        ROS_WARN("[zone_detector] No ~zone_map param found. "
                 "Auto-assigned zone IDs to %zu distinct pixel values (sorted ascending).",
                 distinct.size());
    }

    void printZoneMap() const {
        ROS_INFO("[zone_detector] ===== Pixel → Zone Mapping =====");
        for (const auto& kv : zone_map_) {
            ROS_INFO("  pixel_value = %3d  -->  zone = %d", (int)kv.first, kv.second);
        }
        ROS_INFO("[zone_detector] ================================");
    }

    // ------------------------------------------------------------------
    // 世界坐标（米）→ 图像像素坐标
    //
    // ROS 地图约定：
    //   origin 是图像左下角对应的世界坐标
    //   图像行 0 在顶部，行 (height-1) 在底部
    //
    //   col = floor( (wx - origin_x) / resolution )
    //   row = height - 1 - floor( (wy - origin_y) / resolution )
    // ------------------------------------------------------------------
    bool worldToPixel(double wx, double wy, int& col, int& row) const {
        col = static_cast<int>((wx - map_cfg_.origin_x) / map_cfg_.resolution);
        row = pgm_.height - 1 -
              static_cast<int>((wy - map_cfg_.origin_y) / map_cfg_.resolution);
        return (col >= 0 && col < pgm_.width &&
                row >= 0 && row < pgm_.height);
    }

    // ------------------------------------------------------------------
    // 坐标回调
    // ------------------------------------------------------------------
    void posCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        const double wx = msg->point.x;
        const double wy = msg->point.y;

        int col, row;
        if (!worldToPixel(wx, wy, col, row)) {
            ROS_WARN_THROTTLE(2.0,
                "[zone_detector] Position (%.3f, %.3f) is outside map bounds (map: %dx%d px).",
                wx, wy, pgm_.width, pgm_.height);
            return;
        }

        uint8_t pv = pgm_.at(row, col);

        // negate 标志：翻转灰度（使黑白语义对调）
        if (map_cfg_.negate) {
            pv = static_cast<uint8_t>(pgm_.max_val) - pv;
        }

        auto it = zone_map_.find(pv);
        int zone = (it != zone_map_.end()) ? it->second : -1;

        if (zone == -1) {
            ROS_WARN_THROTTLE(5.0,
                "[zone_detector] Pixel value %d at (%.2f, %.2f) not in zone_map.", (int)pv, wx, wy);
        }

        ROS_DEBUG("[zone_detector] (%.3f, %.3f) -> pixel(%d, %d) val=%d -> zone=%d",
                  wx, wy, col, row, (int)pv, zone);

        std_msgs::Int32 out;
        out.data = zone;
        zone_pub_.publish(out);
    }
};

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "zone_detector");
    ZoneDetectorNode node;
    ros::spin();
    return 0;
}
