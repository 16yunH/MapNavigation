#include <iostream>
#include "src/OsmParser.h"
#include "src/PathFinder.h"
#include "crow.h"

using namespace std;
using namespace crow;
using namespace road_network;  // 使用新的命名空间
using namespace routing;      // 使用新的路径查找命名空间

// 用于构建JSON响应的辅助函数
json::wvalue buildRouteResponse(const RoutePath& route) {
    json::wvalue result;

    // 设置基本信息
    result["success"] = route.success;
    result["totalDistance"] = route.totalDistance;
    result["totalDuration"] = route.totalDuration;

    // 构建路径坐标数组
    vector<json::wvalue> coordinates;
    for (const auto& coord : route.coordinates) {
        coordinates.push_back({
            {"lat", coord.first},
            {"lng", coord.second}  // 注意：Leaflet使用lng而不是lon
        });
    }
    result["coordinates"] = std::move(coordinates);

    // 构建导航步骤数组
    vector<json::wvalue> steps;
    for (const auto& step : route.steps) {
        steps.push_back({
            {"lat", step.lat},
            {"lng", step.lon},
            {"instruction", step.instruction},
            {"distance", step.distance},
            {"duration", step.duration},
            {"roadName", step.roadName},
            {"roadType", step.roadType}
        });
    }
    result["steps"] = std::move(steps);

    return result;
}

int main() {
    cout << "Starting navigation server..." << endl;

    // 步骤1：初始化路网数据
    RoadNetwork network;
    if (!network.loadFromOsm("D:/3.code/CLionProjects/mapnavigation/backend/data/mapofyt.osm")) {
        cerr << "Failed to load OSM file." << endl;
        return 1;
    }
    cout << "Road network loaded successfully." << endl;

    // 步骤2：初始化路径查找器
    PathFinder pathFinder(network);

    // 步骤3：设置Crow服务器
    SimpleApp app;

    // CORS处理函数
    auto cors_handler = [](response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
    };

    // 获取道路网络信息的端点
    CROW_ROUTE(app, "/network_info")
    .methods(HTTPMethod::OPTIONS, HTTPMethod::GET)
    ([&network, cors_handler](const request& req) -> response {
        response res;
        if (req.method == HTTPMethod::OPTIONS) {
            cors_handler(res);
            return res;
        }

        json::wvalue result;
        result["nodeCount"] = network.getAllNodes().size();
        result["roadCount"] = network.getAllRoads().size();

        res = response(result);
        cors_handler(res);
        return res;
    });

    // 查找路径的端点
    CROW_ROUTE(app, "/find_path")
    .methods(HTTPMethod::OPTIONS, HTTPMethod::POST)
    ([&pathFinder, cors_handler](const request& req) -> response {
        response res;
        if (req.method == HTTPMethod::OPTIONS) {
            cors_handler(res);
            return res;
        }

        try {
            auto data = json::load(req.body);
            if (!data) {
                return response(400, "Invalid JSON");
            }

            // 从请求中获取起点和终点坐标
            double startLat = data["startLat"].d();
            double startLon = data["startLon"].d();
            double endLat = data["endLat"].d();
            double endLon = data["endLon"].d();

            // 计算路径
            auto route = pathFinder.findPath(startLat, startLon, endLat, endLon);

            // 构建响应
            auto result = buildRouteResponse(route);
            res = response(result);

        } catch (const exception& e) {
            res = response(500, string("Server error: ") + e.what());
        }

        cors_handler(res);
        return res;
    });

    // 获取道路信息的端点
    CROW_ROUTE(app, "/roads")
    .methods(HTTPMethod::OPTIONS, HTTPMethod::GET)
    ([&network, cors_handler](const request& req) -> response {
        response res;
        if (req.method == HTTPMethod::OPTIONS) {
            cors_handler(res);
            return res;
        }

        json::wvalue result;
        vector<json::wvalue> roads;

        for (const auto& [roadId, road] : network.getAllRoads()) {
            vector<json::wvalue> coordinates;
            for (const auto& nodeId : road.nodeIds) {
                if (const auto* node = network.getNode(nodeId)) {
                    coordinates.push_back({
                        {"lat", node->location.lat},
                        {"lng", node->location.lon}
                    });
                }
            }

            roads.push_back({
                {"id", roadId},
                {"name", road.name},
                {"type", road.type},
                {"coordinates", std::move(coordinates)}
            });
        }

        result["roads"] = std::move(roads);
        res = response(result);
        cors_handler(res);
        return res;
    });

    // 步骤4：启动服务器
    cout << "Starting server on port 8081..." << endl;
    app.port(8081)
       .multithreaded()
       .run();

    return 0;
}