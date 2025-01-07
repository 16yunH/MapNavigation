<template>
  <div class="map-container">
    <div class="search-panel">
      <!-- 起点搜索 -->
      <div class="search-box">
        <input
            v-model="startSearch"
            @input="handleStartSearchInput"
            placeholder="输入起点..."
            class="search-input"
        />
        <div v-if="startSuggestions.length" class="suggestions-container">
          <div
              v-for="suggestion in startSuggestions"
              :key="suggestion.id"
              @click="selectStartLocation(suggestion)"
              class="suggestion-item"
          >
            {{ suggestion.name }}
          </div>
        </div>
      </div>

      <!-- 终点搜索 -->
      <div class="search-box">
        <input
            v-model="endSearch"
            @input="handleEndSearchInput"
            placeholder="输入终点..."
            class="search-input"
        />
        <div v-if="endSuggestions.length" class="suggestions-container">
          <div
              v-for="suggestion in endSuggestions"
              :key="suggestion.id"
              @click="selectEndLocation(suggestion)"
              class="suggestion-item"
          >
            {{ suggestion.name }}
          </div>
        </div>
      </div>
    </div>
    <!-- 控制面板 -->
    <div class="controls">
      <!-- 起点和终点信息显示 -->
      <div v-if="startPoint" class="coordinate-info">
        起点: {{ formatCoordinate(startPoint) }}
      </div>
      <div v-if="endPoint" class="coordinate-info">
        终点: {{ formatCoordinate(endPoint) }}
      </div>

      <!-- 操作按钮 -->
      <button
          v-if="startPoint && endPoint"
          @click="findShortestPath"
          class="action-button"
      >
        查找最短路径
      </button>
      <button
          @click="resetPoints"
          class="action-button"
      >
        重置选择点
      </button>

      <!-- 操作说明 -->
      <div class="instruction-panel" v-if="!startPoint || !endPoint">
        {{ !startPoint ? '请点击地图选择起点' : '请点击地图选择终点' }}
      </div>
    </div>

    <!-- 地图容器 -->
    <div class="map" ref="map"></div>

    <!-- 路径信息面板 -->
    <div v-if="routeInfo" class="route-info-panel">
      <h3>路径信息</h3>
      <div class="route-summary">
        <div>总距离: {{ (routeInfo.totalDistance/1000).toFixed(2) }}公里</div>
        <div>预计用时: {{ Math.round(routeInfo.totalDuration/60) }}分钟</div>
      </div>
      <div class="route-steps">
        <h4>导航步骤:</h4>
        <ol>
          <li v-for="(step, index) in routeInfo.steps" :key="index">
            {{ step.instruction }}
            <div class="step-details">
              {{ (step.distance).toFixed(0) }}米 |
              {{ Math.round(step.duration) }}秒
            </div>
          </li>
        </ol>
      </div>
    </div>
  </div>
</template>

<script>
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import axios from 'axios';

export default {
  name: 'MapNavigation',

  data() {
    return {
      map: null,             // Leaflet地图实例
      pathLayer: null,       // 路径图层
      startPoint: null,      // 起点坐标
      endPoint: null,        // 终点坐标
      startMarker: null,     // 起点标记
      endMarker: null,       // 终点标记
      routeInfo: null,       // 路径信息
      navigationMarkers: [], // 导航标记数组
      loading: false,         // 加载状态
      startSearch: '',
      endSearch: '',
      startSuggestions: [],
      endSuggestions: [],
      amapKey: 'b44c18bdc514de170db976d407902980'
    };
  },

  mounted() {
    this.initMap();
    this.resizeMap();
    // 监听窗口大小变化，确保地图正确显示
    window.addEventListener('resize', this.resizeMap);
  },

  beforeUnmount() {
    window.removeEventListener('resize', this.resizeMap);
  },

  methods: {
    // 在 methods 对象中添加以下方法：
    outOfChina(lat, lng) {
      // 检查坐标是否在中国境内
      if (lng < 72.004 || lng > 137.8347) {
        return true;
      }
      return lat < 0.8293 || lat > 55.8271;

    },

    transformLat(x, y) {
      // 纬度转换算法
      const PI = 3.14159265358979324;
      let ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * Math.sqrt(Math.abs(x));
      ret += (20.0 * Math.sin(6.0 * x * PI) + 20.0 * Math.sin(2.0 * x * PI)) * 2.0 / 3.0;
      ret += (20.0 * Math.sin(y * PI) + 40.0 * Math.sin(y / 3.0 * PI)) * 2.0 / 3.0;
      ret += (160.0 * Math.sin(y / 12.0 * PI) + 320 * Math.sin(y * PI / 30.0)) * 2.0 / 3.0;
      return ret;
    },

    transformLng(x, y) {
      // 经度转换算法
      const PI = 3.14159265358979324;
      let ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * Math.sqrt(Math.abs(x));
      ret += (20.0 * Math.sin(6.0 * x * PI) + 20.0 * Math.sin(2.0 * x * PI)) * 2.0 / 3.0;
      ret += (20.0 * Math.sin(x * PI) + 40.0 * Math.sin(x / 3.0 * PI)) * 2.0 / 3.0;
      ret += (150.0 * Math.sin(x / 12.0 * PI) + 300.0 * Math.sin(x / 30.0 * PI)) * 2.0 / 3.0;
      return ret;
    },

    gcj02ToWGS84(lat, lng) {
      // GCJ-02 坐标转 WGS-84 坐标
      if (this.outOfChina(lat, lng)) {
        return { lat, lng };
      }

      const PI = 3.14159265358979324;
      let dLat = this.transformLat(lng - 105.0, lat - 35.0);
      let dLng = this.transformLng(lng - 105.0, lat - 35.0);
      let radLat = lat / 180.0 * PI;
      let magic = Math.sin(radLat);
      magic = 1 - 0.00669342162296594323 * magic * magic;
      let sqrtMagic = Math.sqrt(magic);
      dLat = (dLat * 180.0) / ((6378245.0 * (1 - 0.00669342162296594323)) / (magic * sqrtMagic) * PI);
      dLng = (dLng * 180.0) / (6378245.0 / sqrtMagic * Math.cos(radLat) * PI);

      return {
        lat: lat - dLat,
        lng: lng - dLng
      };
    },
    // 初始化地图
    initMap() {
      this.map = L.map(this.$refs.map).setView([28.2825, 117.1286], 13);

      // 添加地图图层
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap contributors',
        maxZoom: 19
      }).addTo(this.map);

      // 加载道路网络
      this.loadRoadNetwork();

      // 添加地图点击事件
      this.map.on('click', this.handleMapClick);
    },

    // 加载道路网络数据
    async loadRoadNetwork() {
      try {
        const response = await axios.get('http://localhost:8081/roads');
        this.drawRoads(response.data.roads);
      } catch (error) {
        console.error('加载道路数据失败:', error);
      }
    },

    // 绘制道路网络
    drawRoads(roads) {
      roads.forEach(road => {
        if (road.coordinates.length >= 2) {
          const coordinates = road.coordinates.map(coord => [coord.lat, coord.lng]);
          L.polyline(coordinates, {
            color: '#ccc',
            weight: 2,
            opacity: 0.5
          }).addTo(this.map);
        }
      });
    },

    async handleStartSearchInput() {
      if (this.startSearch.length > 1) {
        await this.fetchSuggestions(this.startSearch, 'start');
      } else {
        this.startSuggestions = [];
      }
    },

    async handleEndSearchInput() {
      if (this.endSearch.length > 1) {
        await this.fetchSuggestions(this.endSearch, 'end');
      } else {
        this.endSuggestions = [];
      }
    },

    async fetchSuggestions(keyword, type) {
      try {
        const response = await axios.get(`https://restapi.amap.com/v3/assistant/inputtips`, {
          params: {
            keywords: keyword,
            key: this.amapKey,
            city: '鹰潭',
            location: '117.069202,28.260189',
            datatype: 'all',
          }
        });

        if (response.data.status === '1') {
          const suggestions = response.data.tips.map(tip => ({
            id: tip.id,
            name: tip.name,
            location: tip.location
          }));

          if (type === 'start') {
            this.startSuggestions = suggestions;
          } else {
            this.endSuggestions = suggestions;
          }
        }
      } catch (error) {
        console.error('获取位置建议失败:', error);
      }
    },

    async selectStartLocation(suggestion) {
      const [gcjLng, gcjLat] = suggestion.location.split(',').map(Number);
      const wgsCoords = this.gcj02ToWGS84(gcjLat, gcjLng);
      this.setStartPoint(wgsCoords);
      this.startSearch = suggestion.name;
      this.startSuggestions = [];
    },

    async selectEndLocation(suggestion) {
      const [gcjLng, gcjLat] = suggestion.location.split(',').map(Number);
      const wgsCoords = this.gcj02ToWGS84(gcjLat, gcjLng);
      this.setEndPoint(wgsCoords);
      this.endSearch = suggestion.name;
      this.endSuggestions = [];
    },

    // 处理地图点击事件
    handleMapClick(e) {
      const latlng = e.latlng;

      if (!this.startPoint) {
        this.setStartPoint(latlng);
      } else if (!this.endPoint) {
        this.setEndPoint(latlng);
      }
    },

    // 设置起点
    setStartPoint(latlng) {
      this.startPoint = latlng;
      if (this.startMarker) {
        this.map.removeLayer(this.startMarker);
      }
      this.startMarker = L.marker(latlng, {
        icon: L.divIcon({
          className: 'custom-marker',
          html: '<div class="marker-start">起点</div>'
        })
      }).addTo(this.map);
    },

    // 设置终点
    setEndPoint(latlng) {
      this.endPoint = latlng;
      if (this.endMarker) {
        this.map.removeLayer(this.endMarker);
      }
      this.endMarker = L.marker(latlng, {
        icon: L.divIcon({
          className: 'custom-marker',
          html: '<div class="marker-end">终点</div>'
        })
      }).addTo(this.map);
    },

    // 查找最短路径
    async findShortestPath() {
      if (!this.startPoint || !this.endPoint) {
        return;
      }

      this.loading = true;
      try {
        const response = await axios.post('http://localhost:8081/find_path', {
          startLat: this.startPoint.lat,
          startLon: this.startPoint.lng,
          endLat: this.endPoint.lat,
          endLon: this.endPoint.lng
        });

        if (response.data.success) {
          this.clearRoute();
          this.drawRoute(response.data);
          this.routeInfo = response.data;
        } else {
          alert('未找到有效路径');
        }
      } catch (error) {
        console.error('查找路径失败:', error);
        alert('路径查找失败，请稍后重试');
      } finally {
        this.loading = false;
      }
    },

    // 绘制路径
    drawRoute(routeData) {
      // 绘制路径线
      const coordinates = routeData.coordinates.map(coord => [coord.lat, coord.lng]);
      this.pathLayer = L.polyline(coordinates, {
        color: '#2196F3',
        weight: 5,
        opacity: 0.7
      }).addTo(this.map);

      // 添加导航点标记
      routeData.steps.forEach((step, index) => {
        if (step.instruction) {
          const marker = L.marker([step.lat, step.lng], {
            icon: L.divIcon({
              className: 'navigation-marker',
              html: `<div class="step-marker">${index + 1}</div>`
            })
          })
              .bindPopup(
                  `<div class="step-popup">
              <b>${step.instruction}</b><br>
              距离: ${(step.distance).toFixed(0)}米<br>
              预计时间: ${Math.round(step.duration)}秒
            </div>`
              )
              .addTo(this.map);

          this.navigationMarkers.push(marker);
        }
      });

      // 调整地图视图以显示整个路径
      this.map.fitBounds(this.pathLayer.getBounds(), {
        padding: [50, 50]
      });
    },

    // 清除路径
    clearRoute() {
      if (this.pathLayer) {
        this.map.removeLayer(this.pathLayer);
      }
      this.navigationMarkers.forEach(marker => {
        this.map.removeLayer(marker);
      });
      this.navigationMarkers = [];
      this.routeInfo = null;
    },

    // 重置所有点
    resetPoints() {
      if (this.startMarker) {
        this.map.removeLayer(this.startMarker);
      }
      if (this.endMarker) {
        this.map.removeLayer(this.endMarker);
      }
      this.clearRoute();
      this.startPoint = null;
      this.endPoint = null;
      this.startMarker = null;
      this.endMarker = null;
    },

    // 格式化坐标显示
    formatCoordinate(point) {
      return `纬度: ${point.lat.toFixed(6)}, 经度: ${point.lng.toFixed(6)}`;
    },

    // 调整地图大小
    resizeMap() {
      this.$nextTick(() => {
        this.map?.invalidateSize();
      });
    }
  }
};
</script>

<style>
.map-container {
  display: flex;
  flex-direction: column;
  height: 100vh;
  position: relative;
  margin: 0;
  padding: 0;
  overflow: hidden;
}

.map {
  flex: 1;
  z-index: 1;
}

.controls {
  position: absolute;
  top: 120px;
  left: 10px;
  z-index: 100;
}

.coordinate-info {
  margin: 5px 0;
  font-size: 14px;
  color: #333;
}

.action-button {
  margin: 5px;
  padding: 8px 16px;
  background-color: #2196F3;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  transition: background-color 0.3s;
}

.action-button:hover {
  background-color: #1976D2;
}

.marker-start {
  background-color: #4CAF50;
  color: white;
  padding: 6px 20px;
  border-radius: 10px;
  font-size: 14px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.2);
  display: flex;
  align-items: center;
  justify-content: center;
  white-space: nowrap;
}

.marker-end {
  background-color: #f44336;
  color: white;
  padding: 6px 20px;
  border-radius: 10px;
  font-size: 14px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.2);
  display: flex;
  align-items: center;
  justify-content: center;
  white-space: nowrap;
}

.step-marker {
  background-color: #2196F3;
  color: white;
  width: 24px;
  height: 24px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 12px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.2);
}

.route-info-panel {
  position: absolute;
  top: 10px;
  right: 10px;
  background-color: white;
  padding: 15px;
  border-radius: 8px;
  box-shadow: 0 2px 8px rgba(0,0,0,0.1);
  max-width: 300px;
  max-height: 70vh;
  overflow-y: auto;
  z-index: 2;
}

.route-summary {
  margin: 10px 0;
  padding: 10px;
  background-color: #f5f5f5;
  border-radius: 4px;
}

.route-steps {
  margin-top: 10px;
}

.route-steps ol {
  padding-left: 20px;
}

.route-steps li {
  margin-bottom: 10px;
}

.step-details {
  font-size: 12px;
  color: #666;
  margin-top: 4px;
}

.step-popup {
  padding: 5px;
  min-width: 200px;
}

.instruction-panel {
  margin: 10px 0;
  padding: 10px;
  background-color: #e3f2fd;
  border-radius: 4px;
  color: #1976D2;
  font-size: 14px;
}

@media (max-width: 768px) {
  .route-info-panel {
    max-width: 100%;
    top: auto;
    bottom: 0;
    right: 0;
    left: 0;
    max-height: 40vh;
    border-radius: 8px 8px 0 0;
  }
}
@media (max-width: 768px) {
  .search-panel {
    width: calc(100% - 20px);
  }
}
.search-panel {
  position: absolute;
  top: 10px;
  left: 10px;
  z-index: 1000;
  background: white;
  padding: 10px;
  border-radius: 4px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.2);
  width: 300px;
}

.search-box {
  position: relative;
  margin-bottom: 10px;
}

.search-input {
  width: 100%;
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 14px;
}

.suggestions-container {
  position: absolute;
  top: 100%;
  left: 0;
  right: 0;
  background: white;
  border: 1px solid #ddd;
  border-top: none;
  border-radius: 0 0 4px 4px;
  max-height: 200px;
  overflow-y: auto;
  z-index: 1001;
}

.suggestion-item {
  padding: 8px;
  cursor: pointer;
  border-bottom: 1px solid #eee;
}

.suggestion-item:hover {
  background-color: #f5f5f5;
}
</style>