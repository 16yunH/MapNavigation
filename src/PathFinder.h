#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>
#include <unordered_map>
#include <queue>
#include <functional>
#include <cmath>
#include <limits>

// Structure to represent a graph node (e.g., an intersection or point)
struct PathNode {
    int id;                // Node identifier (e.g., intersection ID)
    double x, y;           // Coordinates of the node (longitude, latitude)
    PathNode* parent = nullptr; // Pointer to the parent node (used for path reconstruction)
    double gScore = std::numeric_limits<double>::infinity(); // Cost from start to this node
    double fScore = std::numeric_limits<double>::infinity(); // Estimated total cost (gScore + heuristic)

    PathNode(int id, double x, double y) : id(id), x(x), y(y) {}

    bool operator==(const PathNode &other) const {
        return id == other.id;
    }

    // fScore calculation
    [[nodiscard]] double fCost() const {
        return fScore;
    }

    // Optional: Overload < operator for priority queue comparison
    bool operator<(const PathNode &other) const {
        return fScore > other.fScore;  // Priority queue will prioritize lower fScore
    }
};

// Structure to represent an edge between two nodes
struct Edge {
    PathNode *from, *to;
    double weight; // Edge weight (e.g., distance or travel time)

    Edge(PathNode *from, PathNode *to, double weight) : from(from), to(to), weight(weight) {}
};

// Class for PathFinder that implements A* and Bidirectional A* search
class PathFinder {
public:
    PathFinder();

    // Add a node to the graph
    void addNode(int id, double x, double y);

    // Add an edge between two nodes with a weight
    void addEdge(int fromId, int toId, double weight);

    // Find the shortest path between start and goal using Bidirectional A* search
    std::vector<PathNode*> findShortestPath(int startId, int goalId);

private:
    // Data structures for the graph
    std::unordered_map<int, PathNode*> nodes;           // Node storage by ID
    std::unordered_map<int, std::vector<Edge>> graph; // Graph with edges by node ID

    // Helper functions for A* algorithm
    static double heuristic(PathNode* a, PathNode* b);  // Heuristic function for A* (Manhattan distance)
    static std::vector<PathNode*> reconstructPath(PathNode* start, PathNode* meetingPoint, PathNode* goal); // Reconstruct path from meeting point

    // A* search function
    std::unordered_map<int, PathNode*> aStarSearch(int startId, int goalId, bool reverse = false);
};

#endif // PATHFINDER_H
