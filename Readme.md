# MapNavigation

A navigation and path-planning system focused on Yingtan and Guixi cities in Jiangxi Province, China. This project combines OpenStreetMap data with Gaode Maps API to provide accurate routing and location search capabilities.

## Features

### Core Functionality
- Interactive map display of Yingtan and Guixi cities using vue3-leaflet and OpenStreetMap
- Shortest path routing between two points using bidirectional A* algorithm
- Detailed navigation instructions with turn-by-turn guidance
- Path visualization on the map

### Extended Features
- Location search functionality using Gaode Maps API
- Hybrid point selection (manual map clicks or location search)
- Detailed route information display including:
  - Total distance and estimated time
  - Turn-by-turn navigation instructions
  - Path segments with distance and time estimates
- Support for different road types (motorway, trunk, etc.) with speed limit considerations

## Technical Stack

### Frontend
- Vue.js
- vue3-leaflet for map visualization
- Gaode Maps API for location services

### Backend
- C++
- Crow framework for HTTP server
- JSON for data handling

### Dependencies
- crow
- json
- leaflet
- vue3-leaflet

## Setup and Installation

### Prerequisites (Recommended)
- Windows 11
- MinGW compiler
- CLion IDE
- Node.js and npm

### Backend Setup
1. Clone the repository
2. Open the project in CLion
3. Build the project
4. Run the backend server

### Frontend Setup
1. Navigate to the frontend directory
2. Run the following command:
   ```bash
   npm run serve
   ```
3. Access the application at http://localhost:8080/

## Technical Details

### Path Planning Algorithm
- Implements bidirectional A* algorithm with a heuristic weight of 1.1
- Uses KD-Tree for efficient node searching
- Supports various road types with different speed limits
- Includes coordinate transformation between GCJ-02 and WGS-84 systems

### Data Structures
- Implements comprehensive road network representation
- Supports various road types and attributes
- Handles complex intersection scenarios

## Future Plans
- Integration of traffic light information into path planning
- Implementation of machine learning and deep learning methods for smarter routing
- UI/UX improvements
- Enhanced map visualization

## Project Origin
This project focuses on Guixi, a county-level city in Yingtan, Jiangxi Province. While the current scope is limited by the city's transportation infrastructure (only two bus routes in Guixi, no subway for both cities), it serves as a foundation for future enhancements and improvements.

I was born in Guixi and grew up in Guixi, so I have a deep connection to these cities. Due to my studies and work, I have fewer and fewer opportunities to return to Guixi. This project is a way for me to give back to my hometown and contribute to its development (even if only in a small way, I hope).

## Contact
If you have any questions or suggestions, please feel free to contact me by email: [hy20051123@gmail.com](mailto:hy20051123@gmail.com).