#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <functional>
#include <cmath>
#include "OsmParser.h"  // For OsmNode and OsmWay

// A* Node structure
struct AStarNode {
    long long id;  // Node ID
    double gScore; // Cost from start node to this node
    double fScore; // Estimated total cost (gScore + heuristic)
    AStarNode* parent; // Parent node for path reconstruction

    AStarNode(long long id, double gScore, double fScore, AStarNode* parent = nullptr)
        : id(id), gScore(gScore), fScore(fScore), parent(parent) {}

    // Comparator for the priority queue (min-heap based on fScore)
    bool operator<(const AStarNode& other) const {
        return fScore > other.fScore;  // Min-heap, prioritize lower fScore
    }
};

class PathFinder {
public:
    PathFinder();
    ~PathFinder();

    // Method to set the OSM parser (to load map data)
    void setOsmParser(OsmParser* parser);

    // Method to find the shortest path between start and goal using Bidirectional A* search
    std::vector<long long> findShortestPath(long long startId, long long goalId);

private:
    // Helper methods for A* algorithm
    double heuristic(long long fromId, long long toId);
    std::unordered_map<long long, AStarNode*> aStarSearch(long long startId, long long goalId, bool isForward);
    static std::vector<long long> reconstructPath(AStarNode* meetingPoint,
                                            const std::unordered_map<long long, AStarNode*>& forwardSearch,
                                            const std::unordered_map<long long, AStarNode*>& reverseSearch);

    // Data structures for the graph (nodes and ways)
    OsmParser* osmParser_;
    std::unordered_map<long long, OsmNode> nodes_;  // Nodes from OSM
    std::unordered_map<long long, std::vector<long long>> adjList_; // Adjacency list (node ID -> neighbors)
};

#endif // PATHFINDER_H
