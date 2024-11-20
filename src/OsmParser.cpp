#include "OsmParser.h"
#include <iostream>
#include <stdexcept>

using namespace std;
using namespace tinyxml2;

OsmParser::OsmParser(const std::string& filename) : filename(filename) {}

OsmParser::~OsmParser() = default;

bool OsmParser::parse() {
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
        cerr << "Failed to load OSM file: " << filename << endl;
        return false;
    }

    XMLElement* root = doc.FirstChildElement("osm");
    if (root == nullptr) {
        cerr << "Invalid OSM file: " << filename << endl;
        return false;
    }

    for (XMLElement* nodeElement = root->FirstChildElement("node"); nodeElement != nullptr; nodeElement = nodeElement->NextSiblingElement("node")) {
        parseNode(nodeElement);
    }

    return true;
}

const vector<Node>& OsmParser::getNodes() const {
    return nodes;
}

void OsmParser::parseNode(XMLElement* nodeElement) {
    Node node;
    node.id = stoi(nodeElement->Attribute("id"));
    node.lat = stod(nodeElement->Attribute("lat"));
    node.lon = stod(nodeElement->Attribute("lon"));
    node.timestamp = nodeElement->Attribute("timestamp");
    node.version = stoi(nodeElement->Attribute("version"));
    node.changeset = stoi(nodeElement->Attribute("changeset"));
    node.uid = stoi(nodeElement->Attribute("uid"));
    node.user = nodeElement->Attribute("user");

    if (node.id == 0 || node.lat == 0 || node.lon == 0) {
        throw runtime_error("Invalid node data.");
    }

    nodes.push_back(node);
}