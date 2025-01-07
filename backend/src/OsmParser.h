#ifndef OSM_PARSER_H
#define OSM_PARSER_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include "tinyxml2.h"

namespace road_network {

// 前向声明
class RoadNetwork;

// 封装坐标信息的结构体，便于前端显示和距离计算
struct GeoPoint {
    double lat;    // 纬度
    double lon;    // 经度

    // 计算到另一个点的距离(m)
    [[nodiscard]] double distanceTo(const GeoPoint& other) const;
};

// 路网节点结构
struct RoadNode {
    long long id;      // OSM节点ID
    GeoPoint location; // 地理位置

    // 邻接节点信息，pair中包含目标节点ID和距离
    std::vector<std::pair<long long, double>> neighbors;

    // 节点类型，用于识别重要路口
    enum class Type {
        NORMAL,         // 普通道路节点
        INTERSECTION,   // 路口节点(多条道路相交)
        ENDPOINT        // 道路端点
    } type;

    RoadNode() : id(0), location{0.0, 0.0}, type(Type::NORMAL) {}
    // 构造函数
    RoadNode(long long id, double lat, double lon);

    // 添加邻接节点
    void addNeighbor(long long neighborId, double distance);
};

// 道路段信息
struct RoadSegment {
    long long id;                    // OSM way ID
    std::string name;                // 道路名称
    std::string type;                // 道路类型(motorway, primary等)
    bool oneWay;                     // 是否单向道路
    std::vector<long long> nodeIds;  // 构成道路的节点序列
    double speedLimit;               // 速度限制(km/h)
    int priority;                    // 道路等级优先级(用于路径规划)

    // 道路元数据，用于前端显示
    std::string surface;             // 路面类型
    std::string lanes;               // 车道数
    std::unordered_map<std::string, std::string> tags;  // 其他OSM标签

    RoadSegment() : id(0), oneWay(false), speedLimit(50.0), priority(0) {}

    // 构造函数
    explicit RoadSegment(long long id);

    // 计算道路长度（米）
    double calculateLength(const std::unordered_map<long long, RoadNode>& nodes) const;
};

// KD树节点，用于空间索引
struct SpatialNode {
    const RoadNode* node;  // 指向实际节点的指针
    std::unique_ptr<SpatialNode> left;
    std::unique_ptr<SpatialNode> right;

    explicit SpatialNode(const RoadNode* n);
};

struct ProjectedPoint {
    double lat, lon;          // 投影点的坐标
    double distance;          // 到原始点的距离
    long long roadId;         // 所在道路的ID
    std::string roadName;     // 道路名称
    long long segment1;       // 投影点所在线段的起始节点ID
    long long segment2;       // 投影点所在线段的结束节点ID
    double segmentRatio;      // 投影点在线段上的比例（0-1之间）
};

// 路网类
class RoadNetwork {
public:
    RoadNetwork();
    ~RoadNetwork();

    // 核心功能
    bool loadFromOsm(const std::string& filename);  // 加载OSM数据
    long long findNearestNode(double lat, double lon) const;  // 查找最近节点

    // 数据访问器
    const RoadNode* getNode(long long id) const;
    const RoadSegment* getRoad(long long id) const;
    const std::unordered_map<long long, RoadNode>& getAllNodes() const;
    const std::unordered_map<long long, RoadSegment>& getAllRoads() const;

    ProjectedPoint findNearestPointOnRoad(double lat, double lon) const;

private:
    // 数据成员
    std::unordered_map<long long, RoadNode> nodes_;        // 所有节点
    std::unordered_map<long long, RoadSegment> roads_;     // 所有道路
    std::unordered_set<long long> roadNodes_;              // 道路节点ID集合
    std::unique_ptr<SpatialNode> spatialIndex_;            // 空间索引树根节点

    // 数据解析方法
    void parseRoadWay(tinyxml2::XMLElement* wayElement);
    void parseNode(const tinyxml2::XMLElement* nodeElement,
                  const std::unordered_set<long long>& requiredNodes);
    static void parseTags(tinyxml2::XMLElement* element,
                         std::unordered_map<std::string, std::string>& tags);

    // 路网构建方法
    void createRoadConnections();   // 建立节点间的连接关系
    void processIntersections();    // 处理交叉路口
    void buildSpatialIndex();       // 构建空间索引

    // KD树相关方法
    static std::unique_ptr<SpatialNode> buildKDTree(
        std::vector<const RoadNode*>& nodes, int depth = 0);
    static void findNearestNodeImpl(
        const SpatialNode* node, const GeoPoint& target,
        const RoadNode*& best, double& bestDist, int depth);

    // 辅助方法
    static bool isRoadWay(const std::unordered_map<std::string, std::string>& tags);
    static double calculateSpeedLimit(const std::string& highwayType);
    static int calculateRoadPriority(const std::string& highwayType);
    static ProjectedPoint projectPointToSegment(
            double lat, double lon,
            const RoadNode& node1, const RoadNode& node2,
            long long roadId, const std::string& roadName);

};

} // namespace road_network

#endif