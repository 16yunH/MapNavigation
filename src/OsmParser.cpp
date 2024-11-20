#include "OsmParser.h"
#include "D:/3.code/CLionProjects/MapNavigation/lib/tinyxml2.h"

OsmParser::OsmParser() = default;

OsmParser::~OsmParser() = default;

bool OsmParser::parse(const std::string& filename) {
    tinyxml2::XMLDocument doc;

    // Load the XML file
    if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Error loading OSM file: " << filename << std::endl;
        return false;
    }

    // Get the root element <osm>
    tinyxml2::XMLElement* osmElement = doc.RootElement();
    if (osmElement == nullptr) {
        std::cerr << "Error: No root element <osm> found in the file." << std::endl;
        return false;
    }

    // Parse nodes
    tinyxml2::XMLElement* nodeElement = osmElement->FirstChildElement("node");
    while (nodeElement != nullptr) {
        parseNode(nodeElement);
        nodeElement = nodeElement->NextSiblingElement("node");
    }

    // Parse ways
    tinyxml2::XMLElement* wayElement = osmElement->FirstChildElement("way");
    while (wayElement != nullptr) {
        parseWay(wayElement);
        wayElement = wayElement->NextSiblingElement("way");
    }

    return true;
}

const std::unordered_map<long long, OsmNode>& OsmParser::getNodes() const {
    return nodes_;
}

const std::unordered_map<long long, OsmWay>& OsmParser::getWays() const {
    return ways_;
}

void OsmParser::parseNode(tinyxml2::XMLElement* nodeElement) {
    // Parse the attributes of the node
    long long id = nodeElement->Int64Attribute("id");
    double lat = nodeElement->DoubleAttribute("lat");
    double lon = nodeElement->DoubleAttribute("lon");

    // Create a new OsmNode and add it to the nodes map
    OsmNode node{id, lat, lon};
    nodes_[id] = node;
}

void OsmParser::parseWay(tinyxml2::XMLElement* wayElement) {
    // Parse the attributes of the way
    long long id = wayElement->Int64Attribute("id");

    // Initialize an OsmWay object
    OsmWay way;
    way.id = id;

    // Parse the <nd> elements inside the <way> element (these represent the nodes in the way)
    tinyxml2::XMLElement* ndElement = wayElement->FirstChildElement("nd");
    while (ndElement != nullptr) {
        long long nodeId = ndElement->Int64Attribute("ref");
        way.nodeIds.push_back(nodeId);
        ndElement = ndElement->NextSiblingElement("nd");
    }

    // Parse the <tag> element inside the <way> (representing the tag of the way)
    tinyxml2::XMLElement* tagElement = wayElement->FirstChildElement("tag");
    if (tagElement != nullptr) {
        const char* tagValue = tagElement->Attribute("v");
        if (tagValue != nullptr) {
            way.tag = tagValue;
        }
    }

    // Add the way to the ways map
    ways_[id] = way;
}
