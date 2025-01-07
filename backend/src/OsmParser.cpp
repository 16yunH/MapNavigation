#include "OsmParser.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

namespace road_network {

// 常量定义
constexpr double EARTH_RADIUS = 6371000.0;  // 地球半径（米）
constexpr double PI = 3.14159265358979323846;

// GeoPoint方法实现
double GeoPoint::distanceTo(const GeoPoint& other) const {
    // 使用 Haversine 公式计算两点间的球面距离
    double lat1Rad = lat * PI / 180.0;
    double lon1Rad = lon * PI / 180.0;
    double lat2Rad = other.lat * PI / 180.0;
    double lon2Rad = other.lon * PI / 180.0;

    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;

    double a = std::sin(dLat/2) * std::sin(dLat/2) +
               std::cos(lat1Rad) * std::cos(lat2Rad) *
               std::sin(dLon/2) * std::sin(dLon/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    
    return EARTH_RADIUS * c;
}

// RoadNode实现
RoadNode::RoadNode(long long id, double lat, double lon)
    : id(id), location{lat, lon}, type(Type::NORMAL) {}

void RoadNode::addNeighbor(long long neighborId, double distance) {
    // 检查是否已存在该邻居节点，避免重复添加
    auto it = std::find_if(neighbors.begin(), neighbors.end(),
        [neighborId](const auto& pair) { return pair.first == neighborId; });
    
    if (it == neighbors.end()) {
        neighbors.emplace_back(neighborId, distance);
    }
}

// RoadSegment实现
RoadSegment::RoadSegment(long long id)
    : id(id), oneWay(false), speedLimit(50.0), priority(0) {}

double RoadSegment::calculateLength(const std::unordered_map<long long, RoadNode>& nodes) const {
    double totalLength = 0.0;
    // 遍历所有相邻的节点对，计算总长度
    for (size_t i = 0; i < nodeIds.size() - 1; ++i) {
        const auto& node1 = nodes.at(nodeIds[i]);
        const auto& node2 = nodes.at(nodeIds[i + 1]);
        totalLength += node1.location.distanceTo(node2.location);
    }
    return totalLength;
}

// SpatialNode实现
SpatialNode::SpatialNode(const RoadNode* n) : node(n), left(nullptr), right(nullptr) {}

// RoadNetwork实现
RoadNetwork::RoadNetwork() : spatialIndex_(nullptr) {}
RoadNetwork::~RoadNetwork() = default;

bool RoadNetwork::loadFromOsm(const std::string& filename) {
    tinyxml2::XMLDocument doc;
    // 尝试加载OSM文件
    if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
        return false;
    }

    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root) {
        return false;
    }

    // 第一次遍历：解析所有道路，收集需要的节点ID
    for (tinyxml2::XMLElement* element = root->FirstChildElement();
         element != nullptr;
         element = element->NextSiblingElement()) {
        
        if (std::string(element->Name()) == "way") {
            parseRoadWay(element);
        }
    }

    // 第二次遍历：只解析道路相关的节点
    for (tinyxml2::XMLElement* element = root->FirstChildElement();
         element != nullptr;
         element = element->NextSiblingElement()) {
        
        if (std::string(element->Name()) == "node") {
            parseNode(element, roadNodes_);
        }
    }

    // 构建路网结构
    createRoadConnections();
    processIntersections();
    buildSpatialIndex();

    return true;
}

void RoadNetwork::parseRoadWay(tinyxml2::XMLElement* wayElement) {
    // 创建新的道路段落
    long long id = wayElement->Int64Attribute("id");
    RoadSegment segment(id);
    
    // 解析标签
    parseTags(wayElement, segment.tags);
    
    // 检查是否为道路
    if (!isRoadWay(segment.tags)) {
        return;
    }

    // 设置道路属性
    auto it = segment.tags.find("highway");
    if (it != segment.tags.end()) {
        segment.type = it->second;
        segment.speedLimit = calculateSpeedLimit(segment.type);
        segment.priority = calculateRoadPriority(segment.type);
    }

    // 设置道路名称
    it = segment.tags.find("name");
    segment.name = (it != segment.tags.end()) ? it->second : "";

    // 检查是否为单向道路
    it = segment.tags.find("oneway");
    segment.oneWay = (it != segment.tags.end() && (it->second == "yes" || it->second == "1"));

    // 获取车道数
    it = segment.tags.find("lanes");
    segment.lanes = (it != segment.tags.end()) ? it->second : "1";

    // 获取路面类型
    it = segment.tags.find("surface");
    segment.surface = (it != segment.tags.end()) ? it->second : "unknown";

    // 收集构成道路的节点ID
    for (tinyxml2::XMLElement* ndElement = wayElement->FirstChildElement("nd");
         ndElement != nullptr;
         ndElement = ndElement->NextSiblingElement("nd")) {
        
        long long nodeId = ndElement->Int64Attribute("ref");
        segment.nodeIds.push_back(nodeId);
        roadNodes_.insert(nodeId);  // 记录这是道路节点
    }

    // 只保存有效的道路（至少有两个节点）
    if (segment.nodeIds.size() >= 2) {
        roads_[id] = std::move(segment);
    }
}

void RoadNetwork::parseNode(const tinyxml2::XMLElement* nodeElement,
                          const std::unordered_set<long long>& requiredNodes) {
    long long id = nodeElement->Int64Attribute("id");
    // 只处理道路上的节点
    if (requiredNodes.count(id)) {
        double lat = nodeElement->DoubleAttribute("lat");
        double lon = nodeElement->DoubleAttribute("lon");
        nodes_.emplace(id, RoadNode(id, lat, lon));
    }
}

void RoadNetwork::parseTags(tinyxml2::XMLElement* element,
                          std::unordered_map<std::string, std::string>& tags) {
    for (tinyxml2::XMLElement* tagElement = element->FirstChildElement("tag");
         tagElement != nullptr;
         tagElement = tagElement->NextSiblingElement("tag")) {
        
        const char* key = tagElement->Attribute("k");
        const char* value = tagElement->Attribute("v");
        if (key && value) {
            tags[key] = value;
        }
    }
}

ProjectedPoint RoadNetwork::findNearestPointOnRoad(double lat, double lon) const {
    ProjectedPoint nearestPoint;
    nearestPoint.distance = std::numeric_limits<double>::max();

    // 首先找到最近的节点来缩小搜索范围
    long long nearestNodeId = findNearestNode(lat, lon);
    if (nearestNodeId == -1) return nearestPoint;

    // 创建一个已检查道路的集合，避免重复检查
    std::unordered_set<long long> checkedRoads;

    // 从最近的节点开始，检查所有相连的道路
    std::queue<long long> nodeQueue;
    nodeQueue.push(nearestNodeId);
    std::unordered_set<long long> visitedNodes;

    while (!nodeQueue.empty()) {
        long long currentNodeId = nodeQueue.front();
        nodeQueue.pop();

        if (!visitedNodes.insert(currentNodeId).second) {
            continue;  // 节点已访问
        }

        const RoadNode& currentNode = nodes_.at(currentNodeId);

        // 检查与当前节点相连的所有道路
        for (const auto& roadPair : roads_) {
            const RoadSegment& road = roadPair.second;

            // 如果已经检查过这条路，跳过
            if (!checkedRoads.insert(road.id).second) {
                continue;
            }

            // 检查这条路的所有线段
            for (size_t i = 0; i < road.nodeIds.size() - 1; ++i) {
                long long node1Id = road.nodeIds[i];
                long long node2Id = road.nodeIds[i + 1];

                const RoadNode& node1 = nodes_.at(node1Id);
                const RoadNode& node2 = nodes_.at(node2Id);

                // 计算点到当前线段的投影
                ProjectedPoint projected = projectPointToSegment(
                    lat, lon, node1, node2, road.id, road.name);

                // 更新最近点
                if (projected.distance < nearestPoint.distance) {
                    nearestPoint = projected;
                }
            }
        }

        // 如果当前最近点的距离小于到下一层节点的最小距离，可以提前结束搜索
        // 这是一个优化，避免不必要的搜索
        double minNextDistance = std::numeric_limits<double>::max();
        for (const auto& neighbor : currentNode.neighbors) {
            const RoadNode& neighborNode = nodes_.at(neighbor.first);
            double dist = GeoPoint{lat, lon}.distanceTo(neighborNode.location);
            minNextDistance = std::min(minNextDistance, dist);
        }

        if (nearestPoint.distance <= minNextDistance * 0.5) {
            break;  // 已找到最近点，可以结束搜索
        }

        // 将相邻节点加入队列
        for (const auto& neighbor : currentNode.neighbors) {
            if (!visitedNodes.count(neighbor.first)) {
                nodeQueue.push(neighbor.first);
            }
        }
    }

    return nearestPoint;
}

ProjectedPoint RoadNetwork::projectPointToSegment(
    double lat, double lon,
    const RoadNode& node1, const RoadNode& node2,
    long long roadId, const std::string& roadName) {

    ProjectedPoint result;
    result.roadId = roadId;
    result.roadName = roadName;
    result.segment1 = node1.id;
    result.segment2 = node2.id;

    // 转换到平面坐标进行计算（简化版本）
    double x = lon;
    double y = lat;
    double x1 = node1.location.lon;
    double y1 = node1.location.lat;
    double x2 = node2.location.lon;
    double y2 = node2.location.lat;

    // 计算向量
    double dx = x2 - x1;
    double dy = y2 - y1;
    double length_sq = dx * dx + dy * dy;

    if (length_sq == 0.0) {
        // 如果线段长度为0，返回端点
        result.lat = y1;
        result.lon = x1;
        result.segmentRatio = 0.0;
        result.distance = GeoPoint{lat, lon}.distanceTo(GeoPoint{y1, x1});
        return result;
    }

    // 计算投影点在线段上的比例
    double t = ((x - x1) * dx + (y - y1) * dy) / length_sq;

    // 将t限制在[0,1]范围内，确保点在线段上
    t = std::max(0.0, std::min(1.0, t));
    result.segmentRatio = t;

    // 计算投影点的坐标
    result.lon = x1 + t * dx;
    result.lat = y1 + t * dy;

    // 计算实际的地理距离
    result.distance = GeoPoint{lat, lon}.distanceTo(GeoPoint{result.lat, result.lon});

    return result;
}

bool RoadNetwork::isRoadWay(const std::unordered_map<std::string, std::string>& tags) {
    auto it = tags.find("highway");
    if (it == tags.end()) {
        return false;
    }

    // 定义有效的道路类型
    static const std::unordered_set<std::string> validTypes = {
        "motorway", "trunk", "primary", "secondary", "tertiary",
        "residential", "service", "unclassified", "road",
        "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link"
    };

    return validTypes.count(it->second) > 0;
}

    // RoadNetwork.cpp (续)

void RoadNetwork::createRoadConnections() {
    // 遍历所有道路，建立节点之间的连接关系
    for (const auto& [roadId, road] : roads_) {
        const auto& nodeIds = road.nodeIds;

        // 遍历道路上的相邻节点对
        for (size_t i = 0; i < nodeIds.size() - 1; ++i) {
            long long fromId = nodeIds[i];
            long long toId = nodeIds[i + 1];

            // 确保节点存在
            if (nodes_.count(fromId) && nodes_.count(toId)) {
                RoadNode& fromNode = nodes_[fromId];
                RoadNode& toNode = nodes_[toId];

                // 计算节点间距离
                double distance = fromNode.location.distanceTo(toNode.location);

                // 建立双向连接（对于单向道路只建立正向连接）
                fromNode.addNeighbor(toId, distance);
                if (!road.oneWay) {
                    toNode.addNeighbor(fromId, distance);
                }
            }
        }
    }
}

void RoadNetwork::processIntersections() {
    // 遍历所有节点，根据邻居数量确定节点类型
    for (auto& [nodeId, node] : nodes_) {
        if (node.neighbors.size() > 2) {
            // 超过两个邻居节点的为交叉路口
            node.type = RoadNode::Type::INTERSECTION;
        } else if (node.neighbors.size() == 1) {
            // 只有一个邻居的是道路端点
            node.type = RoadNode::Type::ENDPOINT;
        } else {
            // 其他为普通道路节点
            node.type = RoadNode::Type::NORMAL;
        }
    }
}

void RoadNetwork::buildSpatialIndex() {
    // 收集所有需要建立索引的节点
    // 优化：只对交叉路口和端点建立索引，可以显著减少空间开销
    std::vector<const RoadNode*> indexNodes;
    for (const auto& [nodeId, node] : nodes_) {
        if (node.type != RoadNode::Type::NORMAL) {
            indexNodes.push_back(&node);
        }
    }

    // 构建KD树
    spatialIndex_ = buildKDTree(indexNodes);
}

std::unique_ptr<SpatialNode> RoadNetwork::buildKDTree(
    std::vector<const RoadNode*>& nodes, int depth) {

    // 基本情况：空节点集
    if (nodes.empty()) {
        return nullptr;
    }

    // 确定当前维度：深度为偶数时使用纬度，奇数时使用经度
    int axis = depth % 2;

    // 根据当前轴对节点进行排序
    std::sort(nodes.begin(), nodes.end(),
        [axis](const RoadNode* a, const RoadNode* b) {
            if (axis == 0) {
                return a->location.lat < b->location.lat;
            } else {
                return a->location.lon < b->location.lon;
            }
        });

    // 选择中位数节点作为当前节点
    size_t medianIdx = nodes.size() / 2;
    auto node = std::make_unique<SpatialNode>(nodes[medianIdx]);

    // 递归构建左子树（包含较小值的节点）
    std::vector<const RoadNode*> leftNodes(nodes.begin(), nodes.begin() + medianIdx);
    node->left = buildKDTree(leftNodes, depth + 1);

    // 递归构建右子树（包含较大值的节点）
    std::vector<const RoadNode*> rightNodes(nodes.begin() + medianIdx + 1, nodes.end());
    node->right = buildKDTree(rightNodes, depth + 1);

    return node;
}

void RoadNetwork::findNearestNodeImpl(
    const SpatialNode* node, const GeoPoint& target,
    const RoadNode*& best, double& bestDist, int depth) {

    if (!node) return;

    // 计算当前节点到目标点的距离
    double dist = node->node->location.distanceTo(target);
    if (dist < bestDist) {
        bestDist = dist;
        best = node->node;
    }

    // 确定搜索顺序
    int axis = depth % 2;
    double delta = axis == 0 ?
        (target.lat - node->node->location.lat) :
        (target.lon - node->node->location.lon);

    // 根据目标点位置决定优先搜索哪个子树
    const SpatialNode* firstSide = delta < 0 ? node->left.get() : node->right.get();
    const SpatialNode* secondSide = delta < 0 ? node->right.get() : node->left.get();

    // 递归搜索更可能包含最近点的子树
    findNearestNodeImpl(firstSide, target, best, bestDist, depth + 1);

    // 判断是否需要搜索另一个子树
    // 如果垂直距离小于当前最佳距离，另一个子树可能包含更近的点
    GeoPoint testPoint = target;
    if (axis == 0) {
        testPoint.lat = node->node->location.lat;
    } else {
        testPoint.lon = node->node->location.lon;
    }

    if (target.distanceTo(testPoint) < bestDist) {
        findNearestNodeImpl(secondSide, target, best, bestDist, depth + 1);
    }
}

double RoadNetwork::calculateSpeedLimit(const std::string& highwayType) {
    // 根据道路类型返回默认速度限制（km/h）
    static const std::unordered_map<std::string, double> speedLimits = {
        {"motorway", 120.0},
        {"trunk", 100.0},
        {"primary", 80.0},
        {"secondary", 60.0},
        {"tertiary", 50.0},
        {"residential", 30.0},
        {"service", 20.0},
        {"motorway_link", 60.0},
        {"trunk_link", 50.0},
        {"primary_link", 40.0},
        {"secondary_link", 30.0},
        {"tertiary_link", 30.0}
    };

    auto it = speedLimits.find(highwayType);
    return it != speedLimits.end() ? it->second : 50.0;  // 默认50km/h
}

int RoadNetwork::calculateRoadPriority(const std::string& highwayType) {
    // 根据道路类型返回优先级（值越大优先级越高）
    static const std::unordered_map<std::string, int> priorities = {
        {"motorway", 10},
        {"trunk", 9},
        {"primary", 8},
        {"secondary", 7},
        {"tertiary", 6},
        {"residential", 4},
        {"service", 2},
        {"motorway_link", 5},
        {"trunk_link", 5},
        {"primary_link", 4},
        {"secondary_link", 4},
        {"tertiary_link", 3}
    };

    auto it = priorities.find(highwayType);
    return it != priorities.end() ? it->second : 1;  // 默认最低优先级
}

long long RoadNetwork::findNearestNode(double lat, double lon) const {
    if (!spatialIndex_) return -1;

    const RoadNode* best = nullptr;
    double bestDist = std::numeric_limits<double>::max();
    GeoPoint target{lat, lon};

    findNearestNodeImpl(spatialIndex_.get(), target, best, bestDist, 0);

    return best ? best->id : -1;
}

// 访问器方法实现
const RoadNode* RoadNetwork::getNode(long long id) const {
    auto it = nodes_.find(id);
    return it != nodes_.end() ? &it->second : nullptr;
}

const RoadSegment* RoadNetwork::getRoad(long long id) const {
    auto it = roads_.find(id);
    return it != roads_.end() ? &it->second : nullptr;
}

const std::unordered_map<long long, RoadNode>& RoadNetwork::getAllNodes() const {
    return nodes_;
}

const std::unordered_map<long long, RoadSegment>& RoadNetwork::getAllRoads() const {
    return roads_;
}

} // namespace road_network