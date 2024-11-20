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
