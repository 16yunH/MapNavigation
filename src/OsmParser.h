#ifndef OSMPARSER_H
#define OSMPARSER_H

#include <vector>
#include <string>
#include "D:/3.code/CLionProjects/MapNavigation/lib/tinyxml2.h"

struct Node {
    int id;
    double lat;
    double lon;
    std::string timestamp;
    int version;
    int changeset;
    int uid;
    std::string user;
};

class OsmParser {
public:
    explicit OsmParser(const std::string& filename);
    ~OsmParser();

    bool parse();
    [[nodiscard]] const std::vector<Node>& getNodes() const;

private:
    std::string filename;
    std::vector<Node> nodes;

    void parseNode(tinyxml2::XMLElement* nodeElement);
};

#endif