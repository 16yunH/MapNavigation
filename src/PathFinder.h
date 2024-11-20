#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>
#include <map>
#include <cmath>
#include "OsmParser.h"

// 结构体：表示路径规划中的一个节点
struct PathNode {
    long long id;
    double lat, lon; // 经纬度
    double cost;     // 到起点的代价（路径长度）
    PathNode* parent;    // 父节点，帮助重建路径
};

// 结构体：表示路径规划的参数设置
struct PathFinderConfig {
    double maxStepSize; // 最大步长
    double epsilon;     // 惩罚项
    double goalRadius;  // 目标半径
    double maxIterations; // 最大迭代次数
};

// PathFinder 类：实现路径规划逻辑
class PathFinder {
public:
    PathFinder(const OsmParser& parser, const PathFinderConfig& config); // 构造函数
    bool planPath(long long startNodeId, long long goalNodeId);          // 路径规划主函数
    [[nodiscard]] std::vector<PathNode> getPath() const; // 获取路径

private:
    // 工具函数
    static double calculateCost(const PathNode& from, const PathNode& to) ;       // 计算两节点间的距离
    [[nodiscard]] PathNode* findNearestNode(const PathNode& randomNode) const;                // 找到距离随机节点最近的节点
    static bool isValidEdge(const PathNode& from, const PathNode& to) ;           // 判断路径是否有效（不与障碍物相交）
    void addNode(const PathNode& node);                                     // 添加节点到树中

    const OsmParser& m_parser; // OsmParser 用于获取地图数据
    PathFinderConfig m_config; // 路径规划参数
    std::vector<PathNode> m_nodes; // 存储所有的节点（从 OSM 数据解析得到）
    std::vector<PathNode> m_path;  // 规划出来的路径
    std::map<long long, PathNode*> m_nodeMap; // 节点 ID 到 PathNode 对象的映射

    // 随机采样生成节点
    [[nodiscard]] PathNode generateRandomNode() const;
};

#endif // PATHFINDER_H
