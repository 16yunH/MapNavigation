#include "PathFinder.h"
#include <unordered_set>
#include <queue>
#include <iostream>
#include <algorithm>

// Constructor
PathFinder::PathFinder() = default;

// Add a node to the graph
void PathFinder::addNode(int id, double x, double y) {
    nodes[id] = new PathNode(id, x, y);
}

// Add an edge to the graph
void PathFinder::addEdge(int fromId, int toId, double weight) {
    PathNode* fromNode = nodes[fromId];
    PathNode* toNode = nodes[toId];
    graph[fromId].emplace_back(fromNode, toNode, weight);
    graph[toId].emplace_back(toNode, fromNode, weight); // assuming undirected graph
}

// Heuristic function for A* (Manhattan distance)
double PathFinder::heuristic(PathNode* a, PathNode* b) {
    // Manhattan Distance: |x1 - x2| + |y1 - y2|
    return std::abs(a->x - b->x) + std::abs(a->y - b->y);
}

// Reconstruct the path once the goal is found
std::vector<PathNode*> PathFinder::reconstructPath(PathNode* start, PathNode* meetingPoint, PathNode* goal) {
    std::vector<PathNode*> path;

    // Reconstruct path from the start to the meeting point
    PathNode* current = meetingPoint;
    while (current != start) {
        path.push_back(current);
        current = current->parent;
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    // Reconstruct path from the meeting point to the goal
    current = meetingPoint;
    while (current != goal) {
        path.push_back(current);
        current = current->parent;
    }
    path.push_back(goal);

    return path;
}

// A* search function for a single direction (either forward or reverse)
std::unordered_map<int, PathNode*> PathFinder::aStarSearch(int startId, int goalId) {
    PathNode* startNode = nodes[startId];
    PathNode* goalNode = nodes[goalId];

    // Priority queue for A* (min-heap)
    std::priority_queue<PathNode*, std::vector<PathNode*>, std::function<bool(PathNode*, PathNode*)>> openSet(
        [](PathNode* a, PathNode* b) {
            return a->fScore > b->fScore; // prioritize by fScore
        }
    );

    // Maps to track the shortest path and costs
    std::unordered_map<int, PathNode*> cameFrom;  // Tracks the path (parent nodes)
    std::unordered_map<int, double> gScore;       // Cost from start to node
    std::unordered_map<int, double> fScore;       // Estimated cost from start to goal

    // Initialize the open set and scores
    openSet.push(startNode);
    gScore[startId] = 0.0;
    fScore[startId] = heuristic(startNode, goalNode);

    while (!openSet.empty()) {
        PathNode* currentNode = openSet.top();
        openSet.pop();

        // If we reached the goal
        if (currentNode == goalNode) {
            return cameFrom;
        }

        // Explore neighbors
        for (const Edge& edge : graph[currentNode->id]) {
            PathNode* neighbor = edge.to;
            double tentativeGScore = gScore[currentNode->id] + edge.weight;

            // If this is a better path
            if (tentativeGScore < gScore[neighbor->id]) {
                neighbor->parent = currentNode;  // Set the parent of the neighbor
                cameFrom[neighbor->id] = currentNode;
                gScore[neighbor->id] = tentativeGScore;
                fScore[neighbor->id] = gScore[neighbor->id] + heuristic(neighbor, goalNode);
                openSet.push(neighbor);
            }
        }
    }

    return {};  // Return an empty map if no path found
}

// Bidirectional A* Search to find the shortest path
std::vector<PathNode*> PathFinder::findShortestPath(int startId, int goalId) {
    if (startId == goalId) return {}; // No path needed if start and goal are the same

    // Perform A* search from both directions
    std::unordered_map<int, PathNode*> forwardPath = aStarSearch(startId, goalId);
    std::unordered_map<int, PathNode*> reversePath = aStarSearch(goalId, startId);

    // Try to find the meeting point between forward and reverse searches
    PathNode* meetingPoint = nullptr;
    for (const auto& node : forwardPath) {
        if (reversePath.find(node.first) != reversePath.end()) {
            meetingPoint = node.second;
            break;
        }
    }

    // If no meeting point found, return empty path
    if (!meetingPoint) return {};

    // Reconstruct the path by combining the forward and reverse paths
    return reconstructPath(nodes[startId], meetingPoint, nodes[goalId]);
}
