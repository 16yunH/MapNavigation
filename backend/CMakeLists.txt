cmake_minimum_required(VERSION 3.28)
project(MapNavigation)

link_libraries(ws2_32)
link_libraries(mswsock)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR}/lib)
include_directories(${CMAKE_SOURCE_DIR}/lib/Crow/include)
include_directories(${CMAKE_SOURCE_DIR}/lib/json/include)
include_directories(${CMAKE_SOURCE_DIR}/lib/asio/include)

add_executable(MapNavigation main.cpp
        src/PathFinder.cpp
        src/PathFinder.h
        src/OsmParser.cpp
        src/OsmParser.h
        lib/tinyxml2.h
        lib/tinyxml2.cpp
        )
