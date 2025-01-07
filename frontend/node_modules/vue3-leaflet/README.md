# vue3-leaflet

注意：此版本暂时为项目内部使用

#### 介绍

使用 leaflet 进行 vue 项目的开发

github 预览 demo 地址：https://wangli66.github.io/vue3-leaflet/

gittee 预览 demo 地址：http://wangli66.gitee.io/vue3-leaflet/（地址暂时无法访问）



前言说明：由于 vite 支持 esm 的写法，本组件依赖的 leaflet 是 common 写法；故需安装插件@originjs/vite-plugin-commonjs；

```js
npm install @originjs/vite-plugin-commonjs --save-dev
```

然后在 vite.config.js 的配置文件加入插件

```js
import { viteCommonjs } from "@originjs/vite-plugin-commonjs";
export default defineConfig({
	plugins: [viteCommonjs()],
	resolve: {
		alias: {
			"@": fileURLToPath(new URL("./src", import.meta.url)),
		},
	},
});
```

#### 安装使用

1、安装

```js
npm install vue3-leaflet leaflet
```

2、使用

```js
1、在main.ts中全局引用
import vue3Leaflet from 'vue3-leaflet';
app.use(vue3Leaflet);
2、仅在单个组件内容使用，按照一般的组件使用方法即可
```

#### 其他

-   在使用过程中有什么问题可以 issue 我
