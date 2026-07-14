# MapNavigation

A navigation and path-planning system focused on Yingtan and Guixi cities in Jiangxi Province, China. This project combines OpenStreetMap data with Gaode Maps API to provide accurate routing and location search capabilities.
The interactive page is in Chinese.

## Features

### Core Functionality
- Interactive map display of Yingtan and Guixi cities using Leaflet and OpenStreetMap
- Shortest path routing between two points using directed A* search
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
- Leaflet for map visualization
- Gaode Maps API for location services

### Backend
- C++
- Crow framework for HTTP server
- JSON for data handling

### Dependencies
- crow
- json
- leaflet
- Leaflet

## Setup and Installation

### Prerequisites (Recommended)
- A C++17 compiler and CMake 3.16 or newer
- Node.js 20.19+ or 22.12+ and npm

### Backend Setup
1. Clone the repository
2. Configure and build the project:
   ```bash
   cmake -S backend -B backend/build
   cmake --build backend/build
   ```
3. Run the backend from the repository root:
   ```bash
   ./backend/build/MapNavigation
   ```
   Pass an OSM file as the first argument or set `MAP_NAVIGATION_OSM_PATH` to
   override the default `backend/data/mapofyt.osm`. Deployment can also set
   `MAP_NAVIGATION_ALLOWED_ORIGIN` and `MAP_NAVIGATION_BIND_ADDRESS`.

### Frontend Setup
1. Navigate to the frontend directory
2. Copy `.env.example` to `.env.local`, create a Gaode JavaScript API key, and
   set `VITE_AMAP_JS_KEY` and `VITE_API_BASE_URL`. Restrict the key to the exact development and
   production domains in the Gaode console. For production, set
   `VITE_AMAP_SERVICE_HOST` to a reverse proxy that stores the
   `securityJsCode` server-side. The plaintext `VITE_AMAP_SECURITY_CODE`
   fallback is intended for local development only. Values prefixed with
   `VITE_` are embedded in the browser bundle and are not server secrets.
3. Run the following command:
   ```bash
   npm ci
   npm run serve
   ```
4. Access the application at http://localhost:8080/

> Security: never commit real keys to this repository. If a key has already
> been committed, revoke or rotate it in the Gaode console; removing it from the
> latest source does not remove it from Git history.

## Technical Details

### Path Planning Algorithm
- Implements A* path finding with one-way-road support
- Projects requested coordinates onto the nearest road segment
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
