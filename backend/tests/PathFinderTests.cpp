#include "PathFinder.h"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int failures = 0;

void expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

void expectNear(double actual, double expected, double tolerance,
                const std::string& message) {
    expect(std::abs(actual - expected) <= tolerance,
           message + " (actual=" + std::to_string(actual) +
               ", expected=" + std::to_string(expected) + ")");
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: MapNavigationTests <fixture.osm>\n";
        return 2;
    }

    road_network::RoadNetwork network;
    expect(network.loadFromOsm(argv[1]), "fixture road network loads");
    if (failures != 0) {
        return 1;
    }

    routing::PathFinder pathFinder(network);

    const auto forward = pathFinder.findPath(0.0, 0.0, 0.0, 0.002);
    expect(forward.success, "forward travel on a one-way road succeeds");
    expectNear(forward.totalDistance, 222.39, 1.0,
               "forward route includes the complete road distance");

    const auto reverse = pathFinder.findPath(0.0, 0.002, 0.0, 0.0);
    expect(!reverse.success, "reverse travel on a one-way road is rejected");

    const auto sameSegment = pathFinder.findPath(0.0001, 0.00025, 0.0001, 0.00075);
    expect(sameSegment.success, "same-segment forward travel succeeds");
    expectNear(sameSegment.totalDistance, 77.84, 1.0,
               "same-segment route includes both off-road connectors");
    expect(!sameSegment.coordinates.empty(), "same-segment route has coordinates");
    if (!sameSegment.coordinates.empty()) {
        expectNear(sameSegment.coordinates.front().first, 0.0001, 1e-9,
                   "route begins at the requested start coordinate");
        expectNear(sameSegment.coordinates.back().second, 0.00075, 1e-9,
                   "route ends at the requested destination coordinate");
    }

    const auto sameSegmentReverse =
        pathFinder.findPath(0.0, 0.00075, 0.0, 0.00025);
    expect(!sameSegmentReverse.success,
           "same-segment reverse travel on a one-way road is rejected");

    if (failures == 0) {
        std::cout << "All PathFinder regression tests passed\n";
    }
    return failures == 0 ? 0 : 1;
}
