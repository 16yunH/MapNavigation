cmake_minimum_required(VERSION 3.28)
project(MapNavigation)

set(CMAKE_CXX_STANDARD 17)

add_executable(MapNavigation main.cpp
        src/PathFinder.cpp
        src/PathFinder.h
        src/OsmParser.cpp
        src/OsmParser.h
        lib/tinyxml2.h)
