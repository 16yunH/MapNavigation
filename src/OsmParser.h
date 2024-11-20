#ifndef OSM_PARSER_H
#define OSM_PARSER_H

#include <string>
#include <vector>
#include <map>
#include "D:/3.code/CLionProjects/MapNavigation/lib/tinyxml2.h"

// 节点（Node）结构体，表示一个 OSM 文件中的节点元素
struct Node {
    long long id;          // 节点 ID
    double lat;            // 纬度
    double lon;            // 经度
    std::map<std::string, std::string> tags; // 节点的标签
};

// 路径（Way）结构体，表示一个 OSM 文件中的路径元素
struct Way {
    long long id;          // 路径 ID
    std::vector<long long> nodeIds; // 路径包含的节点 ID
    std::map<std::string, std::string> tags; // 路径的标签
};

// 解析器类
class OsmParser {
public:
    explicit OsmParser(const std::string& filename);  // 构造函数，指定文件名
    bool parse();                           // 解析 OSM 文件
    [[nodiscard]] const std::vector<Node>& getNodes() const;  // 获取解析后的节点列表
    [[nodiscard]] const std::vector<Way>& getWays() const;    // 获取解析后的路径列表

private:
    std::string m_filename;               // 文件名
    std::vector<Node> m_nodes;            // 存储解析的节点
    std::vector<Way> m_ways;              // 存储解析的路径

    // 辅助函数，解析单个节点
    void parseNode(tinyxml2::XMLElement* element);

    // 辅助函数，解析单个路径
    void parseWay(tinyxml2::XMLElement* element);
};

#endif // OSM_PARSER_H
