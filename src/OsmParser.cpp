#include "OsmParser.h"
#include "D:/3.code/CLionProjects/MapNavigation/lib/tinyxml2.h"
#include <iostream>
#include <stdexcept>

using namespace std;
using namespace tinyxml2;

// 构造函数
OsmParser::OsmParser(const string& filename) : m_filename(filename) {}

// 解析 OSM 文件
bool OsmParser::parse() {
    XMLDocument doc;
    if (doc.LoadFile(m_filename.c_str()) != XML_SUCCESS) {
        cerr << "Failed to load OSM file: " << m_filename << endl;
        return false;
    }

    XMLElement* osmElement = doc.FirstChildElement("osm");
    if (!osmElement) {
        cerr << "Invalid OSM file structure" << endl;
        return false;
    }

    // 解析所有的 Node 元素
    for (XMLElement* nodeElement = osmElement->FirstChildElement("node"); nodeElement != nullptr; nodeElement = nodeElement->NextSiblingElement("node")) {
        parseNode(nodeElement);
    }

    // 解析所有的 Way 元素
    for (XMLElement* wayElement = osmElement->FirstChildElement("way"); wayElement != nullptr; wayElement = wayElement->NextSiblingElement("way")) {
        parseWay(wayElement);
    }

    return true;
}

// 获取解析后的节点
const vector<Node>& OsmParser::getNodes() const {
    return m_nodes;
}

// 获取解析后的路径
const vector<Way>& OsmParser::getWays() const {
    return m_ways;
}

// 解析 Node 元素
void OsmParser::parseNode(XMLElement* element) {
    Node node;
    node.id = element->Int64Attribute("id");
    node.lat = element->DoubleAttribute("lat");
    node.lon = element->DoubleAttribute("lon");

    // 解析标签
    for (XMLElement* tagElement = element->FirstChildElement("tag"); tagElement != nullptr; tagElement = tagElement->NextSiblingElement("tag")) {
        const char* key = tagElement->Attribute("k");
        const char* value = tagElement->Attribute("v");
        if (key && value) {
            node.tags[key] = value;
        }
    }

    m_nodes.push_back(node);
}

// 解析 Way 元素
void OsmParser::parseWay(XMLElement* element) {
    Way way;
    way.id = element->Int64Attribute("id");

    // 解析节点 ID
    for (XMLElement* ndElement = element->FirstChildElement("nd"); ndElement != nullptr; ndElement = ndElement->NextSiblingElement("nd")) {
        long long nodeId = ndElement->Int64Attribute("ref");
        way.nodeIds.push_back(nodeId);
    }

    // 解析标签
    for (:XMLElement* tagElement = element->FirstChildElement("tag"); tagElement != nullptr; tagElement = tagElement->NextSiblingElement("tag")) {
        const char* key = tagElement->Attribute("k");
        const char* value = tagElement->Attribute("v");
        if (key && value) {
            way.tags[key] = value;
        }
    }

    m_ways.push_back(way);
}
