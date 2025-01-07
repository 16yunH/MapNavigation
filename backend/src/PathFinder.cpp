#include "PathFinder.h"
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <limits>
#include <sstream>

namespace routing {

PathFinder::PathFinder(const road_network::RoadNetwork& network) 
    : network_(network) {}

RoutePath PathFinder::findPath(double startLat, double startLon,
                             double endLat, double endLon) const {
    // 找到最近的道路点
    auto startPoint = network_.findNearestPointOnRoad(startLat, startLon);
    auto endPoint = network_.findNearestPointOnRoad(endLat, endLon);

    if (!startPoint.distance || !endPoint.distance) {
        return RoutePath{.success = false};
    }

    // 考虑起点和终点所在道路段的两个端点，找到最佳组合
    std::vector<std::pair<long long, long long>> candidatePairs = {
        {startPoint.segment1, endPoint.segment1},
        {startPoint.segment1, endPoint.segment2},
        {startPoint.segment2, endPoint.segment1},
        {startPoint.segment2, endPoint.segment2}
    };

    double bestDistance = std::numeric_limits<double>::max();
    std::vector<long long> bestPath;

    // 尝试所有可能的组合，找到最短路径
    for (const auto& [start, end] : candidatePairs) {
        auto path = findShortestPath(start, end);
        if (!path.empty()) {
            double pathDist = 0;
            // 计算路径总长度
            for (size_t i = 0; i < path.size() - 1; ++i) {
                const auto* node1 = network_.getNode(path[i]);
                const auto* node2 = network_.getNode(path[i + 1]);
                pathDist += node1->location.distanceTo(node2->location);
            }
            
            // 加上到实际起终点的距离
            pathDist += startPoint.distance + endPoint.distance;

            if (pathDist < bestDistance) {
                bestDistance = pathDist;
                bestPath = std::move(path);
            }
        }
    }

    if (bestPath.empty()) {
        return RoutePath{.success = false};
    }

    // 构建完整的路径信息
    auto result = constructRoutePath(bestPath);
    result.startRoadName = startPoint.roadName;
    result.endRoadName = endPoint.roadName;

    return result;
}

std::vector<long long> PathFinder::findShortestPath(
    long long startId, long long endId) const {
    // 初始化双向A*搜索的数据结构
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<>> 
        openSetForward, openSetBackward;
    std::unordered_map<long long, double> gScoreForward, gScoreBackward;
    std::unordered_map<long long, long long> cameFromForward, cameFromBackward;
    std::unordered_set<long long> closedSetForward, closedSetBackward;

    const auto* startNode = network_.getNode(startId);
    const auto* goalNode = network_.getNode(endId);

    if (!startNode || !goalNode) return {};

    // 初始化前向搜索
    gScoreForward[startId] = 0;
    openSetForward.push({
        startId, 
        0.0, 
        calculateHeuristic(startNode, goalNode),
        -1
    });

    // 初始化后向搜索
    gScoreBackward[endId] = 0;
    openSetBackward.push({
        endId,
        0.0,
        calculateHeuristic(goalNode, startNode),
        -1
    });

    double bestPathLength = std::numeric_limits<double>::infinity();
    long long meetingNode = -1;

    // 主搜索循环
    while (!openSetForward.empty() && !openSetBackward.empty()) {
        constexpr double heuristicWeight = 1.1;
        // 检查是否可以提前终止
        if (bestPathLength <= 
            openSetForward.top().fScore + openSetBackward.top().fScore) {
            break;
        }

        // 前向搜索一步
        if (!openSetForward.empty()) {
            auto current = openSetForward.top();
            openSetForward.pop();

            if (closedSetForward.count(current.nodeId)) {
                continue;
            }

            closedSetForward.insert(current.nodeId);

            // 检查是否找到相遇点
            if (closedSetBackward.count(current.nodeId)) {
                double pathLength = gScoreForward[current.nodeId] + 
                                  gScoreBackward[current.nodeId];
                if (pathLength < bestPathLength) {
                    bestPathLength = pathLength;
                    meetingNode = current.nodeId;
                }
            }

            // 扩展邻居节点
            const auto* node = network_.getNode(current.nodeId);
            for (const auto& [neighborId, distance] : node->neighbors) {
                if (closedSetForward.count(neighborId)) {
                    continue;
                }

                double tentativeGScore = gScoreForward[current.nodeId] + distance;

                if (!gScoreForward.count(neighborId) || 
                    tentativeGScore < gScoreForward[neighborId]) {
                    
                    gScoreForward[neighborId] = tentativeGScore;
                    cameFromForward[neighborId] = current.nodeId;

                    const auto* neighborNode = network_.getNode(neighborId);
                    double hScore = calculateHeuristic(neighborNode, goalNode);
                    
                    openSetForward.push({
                        neighborId,
                        tentativeGScore,
                        tentativeGScore + heuristicWeight * hScore,
                        current.nodeId
                    });
                }
            }
        }

        // 后向搜索一步
        if (!openSetBackward.empty()) {
            auto current = openSetBackward.top();
            openSetBackward.pop();

            if (closedSetBackward.count(current.nodeId)) {
                continue;
            }

            closedSetBackward.insert(current.nodeId);

            if (closedSetForward.count(current.nodeId)) {
                double pathLength = gScoreForward[current.nodeId] + 
                                  gScoreBackward[current.nodeId];
                if (pathLength < bestPathLength) {
                    bestPathLength = pathLength;
                    meetingNode = current.nodeId;
                }
            }

            const auto* node = network_.getNode(current.nodeId);
            for (const auto& [neighborId, distance] : node->neighbors) {
                if (closedSetBackward.count(neighborId)) {
                    continue;
                }

                double tentativeGScore = gScoreBackward[current.nodeId] + distance;

                if (!gScoreBackward.count(neighborId) || 
                    tentativeGScore < gScoreBackward[neighborId]) {
                    
                    gScoreBackward[neighborId] = tentativeGScore;
                    cameFromBackward[neighborId] = current.nodeId;

                    const auto* neighborNode = network_.getNode(neighborId);
                    double hScore = calculateHeuristic(neighborNode, startNode);
                    
                    openSetBackward.push({
                        neighborId,
                        tentativeGScore,
                        tentativeGScore + heuristicWeight * hScore,
                        current.nodeId
                    });
                }
            }
        }
    }

    // 如果没有找到路径
    if (meetingNode == -1) {
        return {};
    }

    // 构建完整路径
    std::vector<long long> path;
    
    // 构建前向部分
    for (long long current = meetingNode; current != -1;
         current = cameFromForward.count(current) ? cameFromForward[current] : -1) {
        path.push_back(current);
    }
    std::reverse(path.begin(), path.end());

    // 构建后向部分（跳过重复的meeting node）
    for (long long current = cameFromBackward[meetingNode];
         current != -1;
         current = cameFromBackward.count(current) ? cameFromBackward[current] : -1) {
        path.push_back(current);
    }

    return path;
}

double PathFinder::calculateHeuristic(
    const road_network::RoadNode* node1,
    const road_network::RoadNode* node2) {
    return node1->location.distanceTo(node2->location);
}

double PathFinder::calculateBearing(
    double lat1, double lon1, double lat2, double lon2) {
    // 将角度转换为弧度
    double lat1Rad = lat1 * M_PI / 180.0;
    double lon1Rad = lon1 * M_PI / 180.0;
    double lat2Rad = lat2 * M_PI / 180.0;
    double lon2Rad = lon2 * M_PI / 180.0;

    // 计算方位角
    double y = std::sin(lon2Rad - lon1Rad) * std::cos(lat2Rad);
    double x = std::cos(lat1Rad) * std::sin(lat2Rad) -
               std::sin(lat1Rad) * std::cos(lat2Rad) * std::cos(lon2Rad - lon1Rad);
    double bearing = std::atan2(y, x);

    // 转换为度数并归一化到0-360°范围
    return std::fmod((bearing * 180.0 / M_PI + 360.0), 360.0);
}

RouteStep::Direction PathFinder::calculateTurnDirection(
    double prevBearing, double nextBearing) {
    // 计算转向角度
    double angle = nextBearing - prevBearing;
    if (angle < -180) angle += 360;
    if (angle > 180) angle -= 360;

    // 根据角度确定转向方向
    if (std::abs(angle) < TURN_THRESHOLD) {
        return RouteStep::Direction::STRAIGHT;
    } else if (angle < -135) {
        return RouteStep::Direction::SHARP_LEFT;
    } else if (angle < -45) {
        return RouteStep::Direction::TURN_LEFT;
    } else if (angle < -10) {
        return RouteStep::Direction::SLIGHT_LEFT;
    } else if (angle < 10) {
        return RouteStep::Direction::STRAIGHT;
    } else if (angle < 45) {
        return RouteStep::Direction::SLIGHT_RIGHT;
    } else if (angle < 135) {
        return RouteStep::Direction::TURN_RIGHT;
    } else {
        return RouteStep::Direction::SHARP_RIGHT;
    }
}

std::string PathFinder::generateInstruction(
    RouteStep::Direction direction,
    const std::string& roadName,
    const std::string& nextRoadName) {

    std::stringstream instruction;

    // 根据转向方向生成基础指令
    switch (direction) {
        case RouteStep::Direction::STRAIGHT:
            if (!nextRoadName.empty() && nextRoadName != roadName) {
                instruction << "继续直行进入" << nextRoadName;
            } else {
                instruction << "继续直行";
            }
            break;
        case RouteStep::Direction::TURN_LEFT:
            if (!nextRoadName.empty()) {
                instruction << "左转进入" << nextRoadName;
            } else {
                instruction << "左转";
            }
            break;
        case RouteStep::Direction::TURN_RIGHT:
            if (!nextRoadName.empty()) {
                instruction << "右转进入" << nextRoadName;
            } else {
                instruction << "右转";
            }
            break;
        case RouteStep::Direction::SLIGHT_LEFT:
            if (!nextRoadName.empty()) {
                instruction << "偏左行驶进入" << nextRoadName;
            } else {
                instruction << "偏左行驶";
            }
            break;
        case RouteStep::Direction::SLIGHT_RIGHT:
            if (!nextRoadName.empty()) {
                instruction << "偏右行驶进入" << nextRoadName;
            } else {
                instruction << "偏右行驶";
            }
            break;
        case RouteStep::Direction::SHARP_LEFT:
            if (!nextRoadName.empty()) {
                instruction << "急左转进入" << nextRoadName;
            } else {
                instruction << "急左转";
            }
            break;
        case RouteStep::Direction::SHARP_RIGHT:
            if (!nextRoadName.empty()) {
                instruction << "急右转进入" << nextRoadName;
            } else {
                instruction << "急右转";
            }
            break;
        case RouteStep::Direction::UTURN:
            instruction << "掉头";
            break;
    }

    return instruction.str();
}

double PathFinder::calculateSegmentDuration(double distance, const std::string& roadType) {
    // 根据道路类型确定预估速度(m/s)
    double speed;
    if (roadType == "motorway") {
        speed = 27.8;
    } else if (roadType == "trunk") {
        speed = 22.2;
    } else if (roadType == "primary") {
        speed = 16.7;
    } else if (roadType == "secondary") {
        speed = 13.9;
    } else if (roadType == "residential") {
        speed = 8.3;
    } else {
        speed = 11.1;  // 默认速度
    }

    // 计算时间(s)
    return distance/speed ;
}

RoutePath PathFinder::constructRoutePath(const std::vector<long long>& pathNodes) const {
    RoutePath result;
    result.success = true;
    result.totalDistance = 0;
    result.totalDuration = 0;

    if (pathNodes.empty()) {
        result.success = false;
        return result;
    }

    // 构建路径坐标序列
    for (const auto& nodeId : pathNodes) {
        const auto* node = network_.getNode(nodeId);
        result.coordinates.emplace_back(
            node->location.lat,
            node->location.lon
        );
    }

    // 构建导航步骤
    for (size_t i = 0; i < pathNodes.size() - 1; ++i) {
        const auto* currentNode = network_.getNode(pathNodes[i]);
        const auto* nextNode = network_.getNode(pathNodes[i + 1]);

        // 创建导航步骤
        RouteStep step;
        step.lat = currentNode->location.lat;
        step.lon = currentNode->location.lon;

        // 计算当前路段的距离
        step.distance = currentNode->location.distanceTo(nextNode->location);
        result.totalDistance += step.distance;

        // 获取当前和下一个道路的信息
        if (const auto* currentRoad = findRoadForNodes(pathNodes[i], pathNodes[i + 1])) {
            step.roadName = currentRoad->name;
            step.roadType = currentRoad->type;
            step.duration = calculateSegmentDuration(step.distance, step.roadType);
            result.totalDuration += step.duration;
        }

        // 如果不是最后一段，计算转向信息
        if (i < pathNodes.size() - 2) {
            const auto* nextNextNode = network_.getNode(pathNodes[i + 2]);

            // 计算前后路段的方位角
            double currentBearing = calculateBearing(
                currentNode->location.lat, currentNode->location.lon,
                nextNode->location.lat, nextNode->location.lon
            );
            double nextBearing = calculateBearing(
                nextNode->location.lat, nextNode->location.lon,
                nextNextNode->location.lat, nextNextNode->location.lon
            );

            // 确定转向方向
            step.direction = calculateTurnDirection(currentBearing, nextBearing);

            // 获取下一段路的信息用于生成导航提示
            const auto* nextRoad = findRoadForNodes(pathNodes[i + 1], pathNodes[i + 2]);
            std::string nextRoadName = nextRoad ? nextRoad->name : "";

            // 生成导航提示
            step.instruction = generateInstruction(
                step.direction,
                step.roadName,
                nextRoadName
            );
        } else {
            // 最后一段路不需要转向信息
            step.direction = RouteStep::Direction::STRAIGHT;
            step.instruction = "到达目的地";
        }

        result.steps.push_back(step);
    }

    return result;
}

const road_network::RoadSegment* PathFinder::findRoadForNodes(
    long long node1Id, long long node2Id) const {
    // 遍历所有道路，查找包含这两个相邻节点的道路
    const auto& roads = network_.getAllRoads();
    for (const auto& [roadId, road] : roads) {
        auto& nodeIds = road.nodeIds;
        for (size_t i = 0; i < nodeIds.size() - 1; ++i) {
            if ((nodeIds[i] == node1Id && nodeIds[i + 1] == node2Id) ||
                (nodeIds[i] == node2Id && nodeIds[i + 1] == node1Id)) {
                return &road;
            }
        }
    }
    return nullptr;
}

} // namespace routing