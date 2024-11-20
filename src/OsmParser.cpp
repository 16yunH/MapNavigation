#include "OsmParser.h"
#include <iostream>
#include <stdexcept>

OsmParser::OsmParser(const std::string& filename) : filename(filename) {}

OsmParser::~OsmParser() = default;

bool OsmParser::parse() {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to load OSM file: " << filename << std::endl;
        return false;
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("osm");
    if (root == nullptr) {
        std::cerr << "Invalid OSM file: " << filename << std::endl;
        return false;
    }

    for (tinyxml2::XMLElement* nodeElement = root->FirstChildElement("node"); nodeElement != nullptr; nodeElement = nodeElement->NextSiblingElement("node")) {
        parseNode(nodeElement);
    }

    return true;
}

const std::vector<Node>& OsmParser::getNodes() const {
    return nodes;
}

void OsmParser::parseNode(tinyxml2::XMLElement* nodeElement) {
    Node node;
    node.id = std::stoi(nodeElement->Attribute("id"));
    node.lat = std::stod(nodeElement->Attribute("lat"));
    node.lon = std::stod(nodeElement->Attribute("lon"));
    node.timestamp = nodeElement->Attribute("timestamp");
    node.version = std::stoi(nodeElement->Attribute("version"));
    node.changeset = std::stoi(nodeElement->Attribute("changeset"));
    node.uid = std::stoi(nodeElement->Attribute("uid"));
    node.user = nodeElement->Attribute("user");

    if (node.id == 0 || node.lat == 0 || node.lon == 0) {
        throw std::runtime_error("Invalid node data.");
    }

    nodes.push_back(node);
}