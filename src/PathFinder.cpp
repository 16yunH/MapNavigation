#include "PathFinder.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <queue>
#include <functional>
#include <set>

using namespace std;

// 构造函数：初始化路径规划器
PathFinder::PathFinder(const OsmParser& parser, const PathFinderConfig& config)
    : m_parser(parser), m_config(config) {
    // 从 OsmParser 获取所有节点
    const auto& nodes = parser.getNodes();
    for (const auto& node : nodes) {
        PathNode newNode{};
        newNode.id = node.id;
        newNode.lat = node.lat;
        newNode.lon = node.lon;
        newNode.cost = INFINITY;
        newNode.parent = nullptr;
        m_nodes.push_back(newNode);
        m_nodeMap[node.id] = &m_nodes.back();
    }
}

// 计算两节点之间的代价（距离）
double PathFinder::calculateCost(const PathNode& from, const PathNode& to) {
    double dx = to.lat - from.lat;
    double dy = to.lon - from.lon;
    return sqrt(dx * dx + dy * dy);  // 计算欧几里得距离
}

// 生成一个随机节点（模拟采样）
PathNode PathFinder::generateRandomNode() const {
    // 这里只是简单生成一个随机位置，可以根据实际需要进行调整
    double lat = m_nodes[0].lat + (rand() % 1000) / 10000.0; // 随机纬度
    double lon = m_nodes[0].lon + (rand() % 1000) / 10000.0; // 随机经度
    PathNode randomNode{};
    randomNode.lat = lat;
    randomNode.lon = lon;
    return randomNode;
}

// 找到与给定节点最近的节点
PathNode* PathFinder::findNearestNode(const PathNode& randomNode) const {
    PathNode* nearestNode = nullptr;
    double minCost = INFINITY;
    for (const auto& node : m_nodes) {
        double cost = calculateCost(randomNode, node);
        if (cost < minCost) {
            minCost = cost;
            nearestNode = const_cast<PathNode*>(&node); // 获取节点的指针
        }
    }
    return nearestNode;
}

// 判断两个节点之间的路径是否有效（这里可以添加障碍检测）
bool PathFinder::isValidEdge(const PathNode& from, const PathNode& to) {
    // 简单的有效性判断，只是检查两点间的直线距离
    return true; // 假设没有障碍物
}

// 添加一个新的节点到树中
void PathFinder::addNode(const PathNode& node) {
    m_nodes.push_back(node);
    m_nodeMap[node.id] = &m_nodes.back();
}

// 主路径规划函数：使用 Informed RRT* 算法
bool PathFinder::planPath(long long startNodeId, long long goalNodeId) {
    PathNode* startNode = m_nodeMap[startNodeId];
    PathNode* goalNode = m_nodeMap[goalNodeId];

    // 初始化起点
    startNode->cost = 0.0;
    startNode->parent = nullptr;

    // 启动随机树
    for (int iter = 0; iter < m_config.maxIterations; ++iter) {
        // 1. 生成一个随机节点
        PathNode randomNode = generateRandomNode();

        // 2. 找到距离随机节点最近的节点
        PathNode* nearestNode = findNearestNode(randomNode);

        // 3. 计算从最近节点到随机节点的步长
        double cost = calculateCost(*nearestNode, randomNode);
        if (cost > m_config.maxStepSize) {
            // 限制最大步长
            randomNode.lat = nearestNode->lat + m_config.maxStepSize * (randomNode.lat - nearestNode->lat) / cost;
            randomNode.lon = nearestNode->lon + m_config.maxStepSize * (randomNode.lon - nearestNode->lon) / cost;
        }

        // 4. 检查路径是否有效
        if (!isValidEdge(*nearestNode, randomNode)) {
            continue;
        }

        // 5. 计算新的代价，并添加节点
        randomNode.cost = nearestNode->cost + calculateCost(*nearestNode, randomNode);
        randomNode.parent = nearestNode;
        addNode(randomNode);

        // 6. 如果到达目标区域，停止搜索
        if (calculateCost(randomNode, *goalNode) < m_config.goalRadius) {
            goalNode->parent = &randomNode;
            break;
        }
    }

    // 从目标节点回溯路径
    PathNode* currentNode = goalNode;
    m_path.clear();
    while (currentNode != nullptr) {
        m_path.push_back(*currentNode);
        currentNode = currentNode->parent;
    }

    // 如果路径为空，则返回 false
    reverse(m_path.begin(), m_path.end());
    return !m_path.empty();
}

// 获取计算得到的路径
std::vector<PathNode> PathFinder::getPath() const {
    return m_path;
}
