<!-- App.vue - 应用的根组件 -->
<template>
  <div id="app">
    <!-- 主要内容区域：地图组件 -->
    <main class="main-content">
      <OsmMap />
    </main>

    <!-- 底部信息栏 -->
    <footer class="app-footer">
      <p>© 2024 贵溪市导航系统 - 基于 OpenStreetMap 数据</p>
    </footer>
  </div>
</template>

<script>
import OsmMap from './components/mapView.vue';

export default {
  name: 'App',
  components: {
    OsmMap,
  },
};
</script>

<style>
/* 全局重置样式 */
* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

/* 根元素样式 */
html, body {
  height: 100%;
  width: 100%;
  overflow: hidden; /* 防止滚动条出现 */
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto,
  'Helvetica Neue', Arial, sans-serif;
}

/* 应用容器样式 */
#app {
  display: flex;
  flex-direction: column;
  height: 100vh;
  width: 100%;
  position: relative; /* 允许子元素使用绝对定位 */
}

/* 主要内容区域样式 */
.main-content {
  flex: 1;
  position: absolute; /* 使用绝对定位 */
  top: 0;           /* 定位到顶部 */
  left: 0;          /* 定位到左侧 */
  right: 0;         /* 延伸到右边缘 */
  bottom: 30px;     /* 留出底部信息栏的空间 */
  overflow: hidden; /* 确保内容不会溢出 */
}

/* 底部信息栏样式 */
.app-footer {
  height: 30px;     /* 固定高度 */
  background-color: #f8f9fa;
  color: #6c757d;
  padding: 5px;     /* 减小内边距使界面更紧凑 */
  text-align: center;
  font-size: 0.875rem;
  border-top: 1px solid #dee2e6;
  position: absolute; /* 使用绝对定位 */
  bottom: 0;         /* 固定在底部 */
  width: 100%;       /* 占据全部宽度 */
}

/* 响应式设计调整 */
@media (max-width: 768px) {
  .app-footer {
    font-size: 0.75rem;
    padding: 3px;
  }
}
</style>