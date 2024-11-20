#include "PathFinder.h"
#include <unordered_set>
#include <queue>
#include <iostream>
#include <algorithm>

PathFinder::PathFinder() = default;

// Add a node to the graph
void PathFinder::addNode(int id, double x, double y) {
    nodes[id] = new Node(id, x, y);
}

// Add an edge to the graph
void PathFinder::addEdge(int fromId, int toId, double weight) {
    Node* fromNode = nodes[fromId];
    Node* toNode = nodes[toId];
    graph[fromId].emplace_back(fromNode, toNode, weight);
    graph[toId].emplace_back(toNode, fromNode, weight); // assuming undirected graph
}

// Heuristic function for A* (Manhattan distance)
double PathFinder::heuristic(Node* a, Node* b) {
    // Manhattan Distance: |x1 - x2| + |y1 - y2|
    return std::abs(a->x - b->x) + std::abs(a->y - b->y);
}

// Reconstruct the path once the goal is found
std::vector<Node*> PathFinder::reconstructPath(Node* start, Node* meetingPoint, Node* goal) {
    std::vector<Node*> path;
    Node* current = meetingPoint;

    // Reconstruct path from the start to the meeting point
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

// A* search function for Bidirectional A*
std::unordered_map<int, Node*> PathFinder::aStarSearch(int startId, int goalId, bool reverse) {
    Node* startNode = nodes[startId];
    Node* goalNode = nodes[goalId];

    // Priority queue for A* (min-heap)
    std::priority_queue<Node*, std::vector<Node*>, std::function<bool(Node*, Node*)>> openSet(
        [](Node* a, Node* b) {
            return a->fCost() > b->fCost(); // prioritize by fCost
        }
    );

    std::unordered_map<int, Node*> cameFrom; // Path tracking
    std::unordered_map<int, double> gScore; // Distance from start to each node
    std::unordered_map<int, double> fScore; // Estimated total cost (gScore + heuristic)

    openSet.push(startNode);
    gScore[startId] = 0.0;
    fScore[startId] = heuristic(startNode, goalNode);

    while (!openSet.empty()) {
        Node* currentNode = openSet.top();
        openSet.pop();

        // If we reached the goal
        if (currentNode == goalNode) {
            return cameFrom;
        }

        // Explore neighbors
        for (const Edge& edge : graph[currentNode->id]) {
            Node* neighbor = edge.to;
            double tentativeGScore = gScore[currentNode->id] + edge.weight;

            // If this is a better path
            if (tentativeGScore < gScore[neighbor->id]) {
                neighbor->parent = currentNode; // Set the parent of the neighbor
                cameFrom[neighbor->id] = currentNode;
                gScore[neighbor->id] = tentativeGScore;
                fScore[neighbor->id] = gScore[neighbor->id] + heuristic(neighbor, goalNode);
                openSet.push(neighbor);
            }
        }
    }

    return {}; // Return an empty map if no path found
}

// Bidirectional A* Search to find the shortest path
std::vector<Node*> PathFinder::findShortestPath(int startId, int goalId) {
    if (startId == goalId) return {}; // No path needed

    // Perform A* search from both directions
    std::unordered_map<int, Node*> forwardPath = aStarSearch(startId, goalId);
    std::unordered_map<int, Node*> reversePath = aStarSearch(goalId, startId, true);

    // Try to find the meeting point between forward and reverse searches
    Node* meetingPoint = nullptr;
    for (const auto& node : forwardPath) {
        if (reversePath.find(node.first) != reversePath.end()) {
            meetingPoint = node.second;
            break;
        }
    }

    // If no meeting point found, return empty path
    if (!meetingPoint) return {};

    return reconstructPath(nodes[startId], meetingPoint, nodes[goalId]);
}
