/*
#include <iostream>
#include <vector>
#include "lib/tinyxml2.h"

using namespace tinyxml2;
using namespace std;


int main() {
    return 0;
}
*/


//example code for Praser
/*
#include <iostream>
#include "src/OsmParser.h"

int main() {
    OsmParser parser("D:/3.code/CLionProjects/MapNavigation/data/map.osm");

    if (parser.parse()) {
        const std::vector<Node>& nodes = parser.getNodes();
        const std::vector<Way>& ways = parser.getWays();

        std::cout << "Nodes: " << nodes.size() << std::endl;
        std::cout << "Ways: " << ways.size() << std::endl;

        // 输出部分解析的节点和路径信息
        for (const auto& node : nodes) {
            std::cout << "Node ID: " << node.id << ", Lat: " << node.lat << ", Lon: " << node.lon << std::endl;
        }

        for (const auto& way : ways) {
            std::cout << "Way ID: " << way.id << ", Nodes: " << way.nodeIds.size() << std::endl;
        }
    }

    return 0;
}
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "src/OsmParser.h"
#include "src/PathFinder.h"

using namespace std;

int main() {
    // Path to the OSM file (adjust according to your actual path)
    OsmParser osmParser("D:/3.code/CLionProjects/MapNavigation/data/map.osm");
    if (!osmParser.parse()) {
        cerr << "Failed to parse OSM file "  << endl;
        return 1;
    }

    // Step 2: Create a PathFinder instance
    PathFinder pathFinder;

    // Step 3: Build the graph by adding nodes and edges from the OSM file
    const vector<Node>& nodes = osmParser.getNodes();
    const vector<Way>& ways = osmParser.getWays();

    // Adding nodes to the PathFinder graph
    for (const Node& osmNode : nodes) {
        pathFinder.addNode(osmNode.id, osmNode.lon, osmNode.lat);
    }

    // Step 4: Test the pathfinding functionality
    // You can replace startId and goalId with actual node IDs based on your OSM file
    int startId = 29583122;  // Use the first node as the start
    int goalId = 59595666;   // Use the second node as the goal

    // Find the shortest path using Bidirectional A* search
    vector<PathNode*> path = pathFinder.findShortestPath(startId, goalId);

    // Step 5: Print the result path
    if (path.empty()) {
        cout << "No path found between nodes " << startId << " and " << goalId << endl;
    } else {
        cout << "Shortest path between nodes " << startId << " and " << goalId << " is:" << endl;
        for (PathNode* node : path) {
            cout << "Node ID: " << node->id << " (Lat: " << node->y << ", Lon: " << node->x << ")" << endl;
        }
    }

    return 0;
}
