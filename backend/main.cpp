#include <iostream>
#include <cmath>
#include <cstdlib>
#include <filesystem>
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

namespace {

std::filesystem::path resolveOsmPath(int argc, char* argv[]) {
    if (argc > 1) {
        return argv[1];
    }
    if (const char* configuredPath = std::getenv("MAP_NAVIGATION_OSM_PATH")) {
        return configuredPath;
    }

    const std::filesystem::path fromRepositoryRoot = "backend/data/mapofyt.osm";
    if (std::filesystem::exists(fromRepositoryRoot)) {
        return fromRepositoryRoot;
    }
    return "data/mapofyt.osm";
}

bool readCoordinate(const json::rvalue& data, const char* key,
                    double minimum, double maximum, double& result) {
    if (!data.has(key) || data[key].t() != json::type::Number) {
        return false;
    }
    result = data[key].d();
    return std::isfinite(result) && result >= minimum && result <= maximum;
}

} // namespace

int main(int argc, char* argv[]) {
    cout << "Starting navigation server..." << endl;

    // 步骤1：初始化路网数据
    RoadNetwork network;
    const auto osmPath = resolveOsmPath(argc, argv);
    if (!network.loadFromOsm(osmPath.string())) {
        cerr << "Failed to load OSM file: " << osmPath << endl;
        return 1;
    }
    cout << "Road network loaded successfully." << endl;

    // 步骤2：初始化路径查找器
    PathFinder pathFinder(network);

    // 步骤3：设置Crow服务器
    SimpleApp app;

    const string allowedOrigin = std::getenv("MAP_NAVIGATION_ALLOWED_ORIGIN")
        ? std::getenv("MAP_NAVIGATION_ALLOWED_ORIGIN")
        : "http://localhost:8080";

    // CORS处理函数
    auto cors_handler = [allowedOrigin](response& res) {
        res.set_header("Access-Control-Allow-Origin", allowedOrigin);
        res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        res.set_header("Vary", "Origin");
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
            if (req.body.size() > 4096) {
                res = response(413, "Request body too large");
                cors_handler(res);
                return res;
            }

            auto data = json::load(req.body);
            if (!data) {
                res = response(400, "Invalid JSON");
                cors_handler(res);
                return res;
            }

            double startLat, startLon, endLat, endLon;
            if (!readCoordinate(data, "startLat", -90.0, 90.0, startLat) ||
                !readCoordinate(data, "startLon", -180.0, 180.0, startLon) ||
                !readCoordinate(data, "endLat", -90.0, 90.0, endLat) ||
                !readCoordinate(data, "endLon", -180.0, 180.0, endLon)) {
                res = response(400, "Coordinates must be finite numbers in valid ranges");
                cors_handler(res);
                return res;
            }

            // 计算路径
            auto route = pathFinder.findPath(startLat, startLon, endLat, endLon);

            // 构建响应
            auto result = buildRouteResponse(route);
            res = response(result);

        } catch (const exception& e) {
            CROW_LOG_ERROR << "Route request failed: " << e.what();
            res = response(500, "Internal server error");
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
    const string bindAddress = std::getenv("MAP_NAVIGATION_BIND_ADDRESS")
        ? std::getenv("MAP_NAVIGATION_BIND_ADDRESS")
        : "127.0.0.1";
    cout << "Starting server on " << bindAddress << ":8081..." << endl;
    app.bindaddr(bindAddress)
       .port(8081)
       .multithreaded()
       .run();

    return 0;
}
