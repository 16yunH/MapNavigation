cmake_minimum_required(VERSION 3.16)
project(MapNavigation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(mapnavigation_core
    src/PathFinder.cpp
    src/OsmParser.cpp
    lib/tinyxml2.cpp
)
target_include_directories(mapnavigation_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

add_executable(MapNavigation main.cpp)
target_include_directories(MapNavigation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Crow/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/json/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/asio/include
)
target_link_libraries(MapNavigation PRIVATE mapnavigation_core Threads::Threads)

if(WIN32)
    target_link_libraries(MapNavigation PRIVATE ws2_32 mswsock)
endif()
include(CTest)
if(BUILD_TESTING)
    add_executable(MapNavigationTests tests/PathFinderTests.cpp)
    target_link_libraries(MapNavigationTests PRIVATE mapnavigation_core)
    add_test(
        NAME pathfinder_regressions
        COMMAND MapNavigationTests ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/one_way.osm
    )
endif()
