#ifndef PATH_FINDER_H
#define PATH_FINDER_H

#include "OsmParser.h"
#include <vector>
#include <memory>
#include <string>

namespace routing {

// 路径导航中的转向指示
struct RouteStep {
    enum class Direction {
        STRAIGHT,      // 直行
        TURN_LEFT,     // 左转
        TURN_RIGHT,    // 右转
        SLIGHT_LEFT,   // 偏左
        SLIGHT_RIGHT,  // 偏右
        SHARP_LEFT,    // 急左转
        SHARP_RIGHT,   // 急右转
        UTURN         // 掉头
    };

    double lat, lon;           // 转向点的坐标
    Direction direction;       // 转向方向
    std::string instruction;   // 转向提示（如："在XX路口左转"）
    double distance;           // 当前路段距离（米）
    std::string roadName;      // 当前道路名称
    std::string roadType;      // 当前道路类型
    double duration;           // 预计时间（秒）
};

// 完整路径信息，设计用于前端显示和导航
struct RoutePath {
    std::vector<std::pair<double, double>> coordinates; // 路径点坐标序列
    std::vector<RouteStep> steps;                       // 导航步骤
    double totalDistance;    // 总距离（米）
    double totalDuration;    // 总时间（秒）
    bool success;           // 是否找到有效路径
    std::string startRoadName;  // 起始道路名称
    std::string endRoadName;    // 终点道路名称
};

// PathFinder类：负责路径查找和路径信息生成
class PathFinder {
public:
    explicit PathFinder(const road_network::RoadNetwork& network);

    // 核心接口：根据经纬度查找路径
    [[nodiscard]] RoutePath findPath(double startLat, double startLon,
                      double endLat, double endLon) const;

private:
    // A*搜索节点数据结构
    struct SearchNode {
        long long nodeId;      // 节点ID
        double gScore;         // 从起点到当前节点的实际距离
        double fScore;         // gScore + 预估距离
        long long cameFrom;    // 前驱节点ID
        std::string roadName;  // 当前所在道路名称
        std::string roadType;  // 当前道路类型

        // 用于优先队列的比较操作
        bool operator>(const SearchNode& other) const {
            return fScore > other.fScore;
        }
    };

    // 路径查找方法
    [[nodiscard]] std::vector<long long> findShortestPath(long long startId, long long endId) const;

    // 计算启发式距离（直线距离）
    static double calculateHeuristic(const road_network::RoadNode* node1,
                                     const road_network::RoadNode* node2);

    // 构建路径结果
    [[nodiscard]] RoutePath constructRoutePath(const std::vector<long long>& pathNodes) const;

    [[nodiscard]] const road_network::RoadSegment *findRoadForNodes(long long node1Id, long long node2Id) const;

    // 计算转向角度和方向
    [[nodiscard]] static double calculateBearing(double lat1, double lon1, double lat2, double lon2);
    [[nodiscard]] static RouteStep::Direction calculateTurnDirection(double prevBearing, double nextBearing);

    // 生成导航指示
    [[nodiscard]] static std::string generateInstruction(RouteStep::Direction direction,
                                                         const std::string& roadName,
                                                         const std::string& nextRoadName);

    // 计算道路段的预计时间
    [[nodiscard]] static double calculateSegmentDuration(double distance, const std::string& roadType);

    // 成员变量
    const road_network::RoadNetwork& network_;    // 路网数据的引用
    static constexpr double TURN_THRESHOLD = 20;  // 转向角度阈值（度）
};

} // namespace routing

#endif // PATH_FINDER_H