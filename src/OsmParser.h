#ifndef OSM_PARSER_H
#define OSM_PARSER_H

#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "D:/3.code/CLionProjects/MapNavigation/lib/tinyxml2.h"

// Define a structure to hold node information
struct OsmNode {
    long long id;      // Node ID
    double lat;        // Latitude
    double lon;        // Longitude
};

// Define a structure to hold way information
struct OsmWay {
    long long id;                  // Way ID
    std::vector<long long> nodeIds; // References to nodes that make up the way
    std::string tag;               // Tag associated with the way (e.g., "building", "highway")
};

// Class for parsing OSM XML data
class OsmParser {
public:
    OsmParser();
    ~OsmParser();

    // Method to parse an OSM XML file
    bool parse(const std::string& filename);

    // Method to get the nodes
    const std::unordered_map<long long, OsmNode>& getNodes() const;

    // Method to get the ways
    const std::unordered_map<long long, OsmWay>& getWays() const;

private:
    // Helper methods for parsing nodes and ways
    void parseNode(tinyxml2::XMLElement* nodeElement);
    void parseWay(tinyxml2::XMLElement* wayElement);

    // Data structures to store nodes and ways
    std::unordered_map<long long, OsmNode> nodes_;
    std::unordered_map<long long, OsmWay> ways_;
};

#endif // OSM_PARSER_H
