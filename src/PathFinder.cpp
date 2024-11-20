#include "PathFinder.h"
#include <unordered_map>
#include <queue>
#include <cmath>
#include <algorithm>

PathFinder::PathFinder() : osmParser_(nullptr) {}

PathFinder::~PathFinder() = default;

void PathFinder::setOsmParser(OsmParser* parser) {
    osmParser_ = parser;
    nodes_ = parser->getNodes();

    // Build adjacency list based on OsmWay
    adjList_.clear();
    const auto& ways = parser->getWays();
    for (const auto& wayPair : ways) {
        const OsmWay& way = wayPair.second;
        for (size_t i = 0; i < way.nodeIds.size() - 1; ++i) {
            adjList_[way.nodeIds[i]].push_back(way.nodeIds[i + 1]);
            adjList_[way.nodeIds[i + 1]].push_back(way.nodeIds[i]);  // Undirected graph
        }
    }
}

// Heuristic function: Manhattan distance between two nodes
double PathFinder::heuristic(long long fromId, long long toId) {
    const OsmNode& fromNode = nodes_[fromId];
    const OsmNode& toNode = nodes_[toId];
    return std::abs(fromNode.lat - toNode.lat) + std::abs(fromNode.lon - toNode.lon); // Manhattan distance
}

// A* search function (forward or backward)
// A* search function (forward or backward)
std::unordered_map<long long, AStarNode*> PathFinder::aStarSearch(long long startId, long long goalId, bool isForward) {
    std::priority_queue<AStarNode> openSet;
    std::unordered_map<long long, AStarNode*> cameFrom;
    std::unordered_map<long long, double> gScore; // Cost from start node
    std::unordered_map<long long, double> fScore; // Estimated total cost

    openSet.emplace(startId, 0.0, heuristic(startId, goalId));
    gScore[startId] = 0.0;
    fScore[startId] = heuristic(startId, goalId);

    while (!openSet.empty()) {
        AStarNode currentNode = openSet.top();  // Get the node from openSet
        openSet.pop();  // Remove it from the queue

        long long currentId = currentNode.id;

        // If we reached the goal node, return the search path
        if (currentId == goalId) {
            return cameFrom;
        }

        // Explore neighbors
        for (long long neighborId : adjList_[currentId]) {
            double tentativeGScore = gScore[currentId] + 1; // assuming weight is 1 for simplicity

            // If we found a better path to the neighbor, update the gScore and fScore
            if (tentativeGScore < gScore[neighborId]) {
                cameFrom[neighborId] = new AStarNode(currentId, tentativeGScore, tentativeGScore + heuristic(neighborId, goalId));
                gScore[neighborId] = tentativeGScore;
                fScore[neighborId] = tentativeGScore + heuristic(neighborId, goalId);
                openSet.emplace(neighborId, tentativeGScore, fScore[neighborId]);
            }
        }
    }

    return {}; // Return empty map if no path found
}


// Reconstruct the path from start to goal using the meeting point
std::vector<long long> PathFinder::reconstructPath(AStarNode* meetingPoint,
                                                    const std::unordered_map<long long, AStarNode*>& forwardSearch,
                                                    const std::unordered_map<long long, AStarNode*>& reverseSearch) {
    std::vector<long long> path;

    // Reconstruct path from start to meeting point
    AStarNode* currentNode = meetingPoint;
    while (currentNode != nullptr) {
        path.push_back(currentNode->id);
        currentNode = currentNode->parent;
    }
    std::reverse(path.begin(), path.end());

    // Reconstruct path from meeting point to goal
    currentNode = reverseSearch.at(meetingPoint->id);
    while (currentNode != nullptr) {
        path.push_back(currentNode->id);
        currentNode = currentNode->parent;
    }

    return path;
}


// Bidirectional A* Search to find the shortest path
std::vector<long long> PathFinder::findShortestPath(long long startId, long long goalId) {
    if (startId == goalId) return {startId}; // No need to search if start == goal

    // Perform A* search from both directions
    std::unordered_map<long long, AStarNode*> forwardSearch = aStarSearch(startId, goalId, true);
    std::unordered_map<long long, AStarNode*> reverseSearch = aStarSearch(goalId, startId, false);

    // Try to find the meeting point between forward and reverse searches
    AStarNode* meetingPoint = nullptr;
    for (const auto& nodePair : forwardSearch) {
        if (reverseSearch.find(nodePair.first) != reverseSearch.end()) {
            meetingPoint = nodePair.second;
            break;
        }
    }

    // If no meeting point is found, return empty path
    if (meetingPoint == nullptr) return {};

    // Reconstruct the path by combining the forward and reverse paths
    return reconstructPath(meetingPoint, forwardSearch, reverseSearch);
}
