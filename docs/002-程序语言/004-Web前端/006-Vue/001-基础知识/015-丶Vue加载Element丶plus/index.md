# 十六、Vue3 加载 Element-plus
## 16.1 什么是 Element-plus

Element Plus 是一个基于 Vue 3 的高质量 UI 组件库。它包含了丰富的组件和扩展功能，例如表格、表单、按钮、导航、通知等，让开发者能够快速构建高质量的 Web 应用。Element Plus 的设计理念是：提供开箱即用的 UI 组件和扩展功能，帮助开发者快速构建应用程序，同时提供详细的文档和教程，让开发者更好地掌握和使用 Element Plus。

- 官网: [Element Plus](https://element-plus.org/zh-CN/)

## 16.2 安装

```cmd
npm install element-plus --save

:: 按需引入配套插件
npm install -D unplugin-vue-components unplugin-auto-import
```

官方文档使用的是TS, 所以如果是复制运行的话, 请先安装TS:

```cmd
:: 实际上应该在创建环境的时候配置~
npm install -g typescript
```

## 16.3 引入

1. 全量导入 (不推荐, 文件会很大~)

```js
import { createApp } from 'vue'
import App from './App.vue'
import './registerServiceWorker'
import router from './router'
import store from './store'
import ElementPlus from 'element-plus' // 这个
import 'element-plus/dist/index.css'   // 还有这个

createApp(App).use(store).use(ElementPlus).use(router).mount('#app')
```

2. 按需导入 | (安装上面配套的插件后) 在`vue.config.js`:

```js
const { defineConfig } = require('@vue/cli-service')
const AutoImport = require('unplugin-auto-import/webpack')
const Components = require('unplugin-vue-components/webpack')
const { ElementPlusResolver } = require('unplugin-vue-components/resolvers')

module.exports = defineConfig({
  transpileDependencies: true,
  configurewebpack: {
    plugins: [
      AutoImport({
        resolvers: [ElementPlusResolver()]
      }),
      Components({
        resolvers: [ElementPlusResolver()]
      })
    ]
  }
})
```

~~不知道为什么我这样不行?!~~

## 16.4 使用

请查看官方文档!!!

随便一个示例:

```html
<template>
  <el-button v-loading.fullscreen.lock="fullscreenLoading" type="primary" @click="openFullScreen1">
    As a directive
  </el-button>
  <el-button type="primary" @click="openFullScreen2"> As a service </el-button>
</template>

<script>
import { ref } from 'vue';
import { ElLoading } from 'element-plus';

export default {
  setup() {
    const fullscreenLoading = ref(false);

    const openFullScreen1 = () => {
      fullscreenLoading.value = true
      setTimeout(() => {
        fullscreenLoading.value = false
      }, 2000)
    };

    const openFullScreen2 = () => {
      const loading = ElLoading.service({
        lock: true,
        text: 'Loading',
        background: 'rgba(0, 0, 0, 0.7)',
      })
      setTimeout(() => {
        loading.close()
      }, 1000)
    };

    return {
      fullscreenLoading,
      openFullScreen1,
      openFullScreen2,
    }
  }
}
</script>
```

## 16.5 使用图标

需要安装:

```cmd
npm install @element-plus/icons-vue
```

然后注册:

```ts
// main.ts
import * as ElementPlusIconsVue from '@element-plus/icons-vue'

const app = createApp(App)
for (const [key, component] of Object.entries(ElementPlusIconsVue)) {
  app.component(key, component)
}
```

然后使用即可: (可以去官方文档:[Icon 图标 | Element Plus](https://element-plus.org/zh-CN/component/icon.html)复制)

```html
<template>
  <div class="hello">
    <el-icon :size="50" color="#990099"><SwitchButton /></el-icon>
  </div>
</template>
```