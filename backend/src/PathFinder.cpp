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
    auto startPoint = network_.findNearestPointOnRoad(startLat, startLon);
    auto endPoint = network_.findNearestPointOnRoad(endLat, endLon);

    if (startPoint.roadId < 0 || endPoint.roadId < 0 ||
        !std::isfinite(startPoint.distance) || !std::isfinite(endPoint.distance)) {
        return {};
    }

    const auto* startRoad = network_.getRoad(startPoint.roadId);
    const auto* endRoad = network_.getRoad(endPoint.roadId);
    const auto* startSegment1 = network_.getNode(startPoint.segment1);
    const auto* startSegment2 = network_.getNode(startPoint.segment2);
    const auto* endSegment1 = network_.getNode(endPoint.segment1);
    const auto* endSegment2 = network_.getNode(endPoint.segment2);
    if (!startRoad || !endRoad || !startSegment1 || !startSegment2 ||
        !endSegment1 || !endSegment2) {
        return {};
    }

    const double startSegmentLength =
        startSegment1->location.distanceTo(startSegment2->location);
    const double endSegmentLength =
        endSegment1->location.distanceTo(endSegment2->location);

    struct Connector {
        long long nodeId;
        double distance;
    };

    std::vector<Connector> startConnectors;
    std::vector<Connector> endConnectors;
    if (!startRoad->oneWay) {
        startConnectors.push_back({
            startPoint.segment1, startPoint.segmentRatio * startSegmentLength});
    }
    startConnectors.push_back({
        startPoint.segment2, (1.0 - startPoint.segmentRatio) * startSegmentLength});

    endConnectors.push_back({
        endPoint.segment1, endPoint.segmentRatio * endSegmentLength});
    if (!endRoad->oneWay) {
        endConnectors.push_back({
            endPoint.segment2, (1.0 - endPoint.segmentRatio) * endSegmentLength});
    }

    const bool sameSegment = startPoint.roadId == endPoint.roadId &&
        startPoint.segment1 == endPoint.segment1 &&
        startPoint.segment2 == endPoint.segment2;
    const bool directTravelAllowed = sameSegment &&
        (!startRoad->oneWay || startPoint.segmentRatio <= endPoint.segmentRatio);

    double bestDistance = std::numeric_limits<double>::infinity();
    std::vector<long long> bestPath;
    double bestStartConnector = 0.0;
    double bestEndConnector = 0.0;
    bool useDirectSegment = false;

    if (directTravelAllowed) {
        bestDistance = startPoint.distance + endPoint.distance +
            std::abs(endPoint.segmentRatio - startPoint.segmentRatio) *
                startSegmentLength;
        useDirectSegment = true;
    }

    for (const auto& startConnector : startConnectors) {
        for (const auto& endConnector : endConnectors) {
            auto path = findShortestPath(startConnector.nodeId, endConnector.nodeId);
            if (path.empty()) {
                continue;
            }

            double pathDistance = startPoint.distance + startConnector.distance +
                endConnector.distance + endPoint.distance;
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                const auto* node1 = network_.getNode(path[i]);
                const auto* node2 = network_.getNode(path[i + 1]);
                if (!node1 || !node2) {
                    pathDistance = std::numeric_limits<double>::infinity();
                    break;
                }
                pathDistance += node1->location.distanceTo(node2->location);
            }

            if (pathDistance < bestDistance) {
                bestDistance = pathDistance;
                bestPath = std::move(path);
                bestStartConnector = startPoint.distance + startConnector.distance;
                bestEndConnector = endConnector.distance + endPoint.distance;
                useDirectSegment = false;
            }
        }
    }

    if (!std::isfinite(bestDistance)) {
        return {};
    }

    auto appendCoordinate = [](std::vector<std::pair<double, double>>& coordinates,
                               double lat, double lon) {
        constexpr double epsilon = 1e-10;
        if (coordinates.empty() ||
            std::abs(coordinates.back().first - lat) > epsilon ||
            std::abs(coordinates.back().second - lon) > epsilon) {
            coordinates.emplace_back(lat, lon);
        }
    };

    if (useDirectSegment) {
        RoutePath result;
        result.success = true;
        result.totalDistance = bestDistance;
        result.totalDuration = calculateSegmentDuration(bestDistance, startRoad->type);
        result.startRoadName = startRoad->name;
        result.endRoadName = startRoad->name;
        appendCoordinate(result.coordinates, startLat, startLon);
        appendCoordinate(result.coordinates, startPoint.lat, startPoint.lon);
        appendCoordinate(result.coordinates, endPoint.lat, endPoint.lon);
        appendCoordinate(result.coordinates, endLat, endLon);

        RouteStep step;
        step.lat = startLat;
        step.lon = startLon;
        step.distance = result.totalDistance;
        step.duration = result.totalDuration;
        step.roadName = startRoad->name;
        step.roadType = startRoad->type;
        step.instruction = "沿" +
            (startRoad->name.empty() ? std::string("当前道路") : startRoad->name) +
            "行驶并到达目的地";
        result.steps.push_back(std::move(step));
        return result;
    }

    if (bestPath.empty()) {
        return {};
    }

    auto result = constructRoutePath(bestPath);
    result.startRoadName = startPoint.roadName;
    result.endRoadName = endPoint.roadName;
    result.totalDistance += bestStartConnector + bestEndConnector;
    const double startConnectorDuration =
        calculateSegmentDuration(bestStartConnector, startRoad->type);
    const double endConnectorDuration =
        calculateSegmentDuration(bestEndConnector, endRoad->type);
    result.totalDuration += startConnectorDuration + endConnectorDuration;

    std::vector<std::pair<double, double>> coordinates;
    appendCoordinate(coordinates, startLat, startLon);
    appendCoordinate(coordinates, startPoint.lat, startPoint.lon);
    for (const auto& [lat, lon] : result.coordinates) {
        appendCoordinate(coordinates, lat, lon);
    }
    appendCoordinate(coordinates, endPoint.lat, endPoint.lon);
    appendCoordinate(coordinates, endLat, endLon);
    result.coordinates = std::move(coordinates);

    constexpr double connectorEpsilon = 0.01;
    if (bestStartConnector > connectorEpsilon) {
        RouteStep step;
        step.lat = startLat;
        step.lon = startLon;
        step.distance = bestStartConnector;
        step.duration = startConnectorDuration;
        step.roadName = startRoad->name;
        step.roadType = startRoad->type;
        step.instruction = "前往" +
            (startRoad->name.empty() ? std::string("最近道路") : startRoad->name);
        result.steps.insert(result.steps.begin(), std::move(step));
    }

    if (bestEndConnector > connectorEpsilon) {
        if (!result.steps.empty()) {
            result.steps.back().instruction = "继续直行";
        }
        RouteStep step;
        step.lat = endPoint.lat;
        step.lon = endPoint.lon;
        step.distance = bestEndConnector;
        step.duration = endConnectorDuration;
        step.roadName = endRoad->name;
        step.roadType = endRoad->type;
        step.instruction = "到达目的地";
        result.steps.push_back(std::move(step));
    }

    return result;
}

std::vector<long long> PathFinder::findShortestPath(
    long long startId, long long endId) const {
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<>> openSet;
    std::unordered_map<long long, double> gScore;
    std::unordered_map<long long, long long> cameFrom;
    std::unordered_set<long long> closedSet;

    const auto* startNode = network_.getNode(startId);
    const auto* goalNode = network_.getNode(endId);

    if (!startNode || !goalNode) return {};

    gScore[startId] = 0.0;
    openSet.push({startId, 0.0, calculateHeuristic(startNode, goalNode)});

    while (!openSet.empty()) {
        const auto current = openSet.top();
        openSet.pop();
        if (!closedSet.insert(current.nodeId).second) {
            continue;
        }

        if (current.nodeId == endId) {
            std::vector<long long> path;
            for (long long nodeId = endId;; nodeId = cameFrom.at(nodeId)) {
                path.push_back(nodeId);
                if (nodeId == startId) {
                    break;
                }
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        const auto* node = network_.getNode(current.nodeId);
        if (!node) {
            continue;
        }

        for (const auto& [neighborId, distance] : node->neighbors) {
            if (closedSet.count(neighborId)) {
                continue;
            }

            const double tentativeGScore = gScore[current.nodeId] + distance;
            const auto existingScore = gScore.find(neighborId);
            if (existingScore != gScore.end() && tentativeGScore >= existingScore->second) {
                continue;
            }

            const auto* neighborNode = network_.getNode(neighborId);
            if (!neighborNode) {
                continue;
            }
            cameFrom[neighborId] = current.nodeId;
            gScore[neighborId] = tentativeGScore;
            openSet.push({
                neighborId,
                tentativeGScore,
                tentativeGScore + calculateHeuristic(neighborNode, goalNode)
            });
        }
    }

    return {};
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
