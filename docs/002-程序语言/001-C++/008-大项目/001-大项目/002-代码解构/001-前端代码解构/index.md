# 前端代码解构
## 1. 环境配置 & 启动康康

- 环境配置: [【工具】用nvm管理nodejs版本切换，真香！](https://segmentfault.com/a/1190000044661290) | 需要使用 node.js 为 16.17.0

搞个nvm吧, 先把node.js删除! (按照上面的教程)

然后再配置VScode: [使用nvm管理多版本Node并在vsCode运行](https://blog.csdn.net/qq_39840761/article/details/94483525)(不用安装插件, 就配置环境变量)

### 1.1 nvm 常用命令

```sh
# 查看可用nodejs的稳定版本
nvm list available 

# 下载nodejs 20.9.0
nvm install 20.9.0

# 看已安装nodejs版本
nvm list

# 切换到14.19.1
nvm use 14.19.1

# 卸载版本20.9.0
nvm uninstall 20.9.0
```

### 1.2 npm

```sh
# 更新到指定版本
npm -g install npm@8.19.2

# 清理 npm 缓存数据
npm cache clean --force
```

### 1.3 启动项目

```sh
# 安装依赖(请先cd进入文件夹) 如果有问题就加 --legacy-peer-deps
npm install

# (就算上面报错了, 下面也试试看看能不能启动!)启动开发版本(有测试界面)
npm run dev
```

### 1.4 启动!

发现不知道密码, 直接改登录凭证进去~ 嘿: `.\mes-frontend\src\router\index.js`的`第66行`, 改为`if (!token) {`

然后就找个非登录链接, 就可以进去了 =-=

## 2. 学习
### 2.1 一种简洁的setup写法

如 `App.vue`:
```html
<script setup>
import zhCn from 'element-plus/dist/locale/zh-cn.mjs'
// element组件语言改为中文，中文国际化
</script>

<template>
    <el-config-provider :locale="zhCn">
        <router-view />
    </el-config-provider>
</template>
```

### 2.2 第三方使用

```json
{
    "name": "project-frontend",
    "type": "module",
    "version": "1.0.0",
    "packageManager": "pnpm@8.15.8",
    "engines": {
        "node": ">=18",
        "vscode": "^1.22.0",
        "pnpm": ">=8"
    },
    "scripts": {
        "preinstall": "npx only-allow pnpm && npm run corepack:pnpm",
        "corepack:pnpm": "corepack enable && corepack install",
        "git:fetch": "git fetch -p",
        "dev": "vite --open",
        "build": "vite build",
        "build:vue-tsc": "vue-tsc && vite build",
        "preview": "vite preview --port 4173",
        "lint:eslint": "eslint . --fix",
        "format:prettier": "prettier --write ."
    },
    "dependencies": {
        "@element-plus/icons-vue": "^2.0.9",
        "@element-plus/locale": "^0.0.5",
        "@vueuse/core": "^10.9.0",
        "@wangeditor/editor": "^5.1.23",
        "@wangeditor/editor-for-vue": "^5.1.12",
        "@zanmato/vue3-treeselect": "^0.2.0",
        "axios": "^0.27.2",
        "crypto-js": "^4.1.1",
        "element-plus": "^2.7.3",
        "fuse.js": "^7.0.0",
        "lodash-es": "^4.17.21",
        "pdfobject": "^2.2.12",
        "pinia": "^2.0.21",
        "qs": "^6.11.0",
        "v-charts": "^1.19.0",
        "vue": "^3.4.27",
        "vue-plugin-hiprint": "^0.0.48",
        "vue-router": "^4.3.2",
        "xlsx": "https://cdn.sheetjs.com/xlsx-0.19.2/xlsx-0.19.2.tgz"
    },
    "devDependencies": {
        "@antfu/eslint-config": "^2.18.1",
        "@eslint/eslintrc": "^3.1.0",
        "@eslint/js": "^9.3.0",
        "@rushstack/eslint-patch": "^1.10.3",
        "@types/lodash": "^4.17.4",
        "@types/lodash-es": "^4.17.12",
        "@types/node": "^20.12.13",
        "@types/qs": "^6.9.15",
        "@vitejs/plugin-vue": "^5.0.4",
        "@vue/eslint-config-prettier": "^7.0.0",
        "eslint": "^9.3.0",
        "eslint-config-flat-gitignore": "^0.1.5",
        "eslint-config-prettier": "^9.1.0",
        "eslint-plugin-format": "^0.1.1",
        "eslint-plugin-jsdoc": "^48.2.6",
        "eslint-plugin-prettier": "^5.1.3",
        "eslint-plugin-vue": "^9.3.0",
        "less": "^4.2.0",
        "prettier": "^3.2.5",
        "sass": "^1.54.9",
        "type-plus": "^7.6.2",
        "typescript": "^5.4.5",
        "unplugin-auto-import": "^0.17.6",
        "unplugin-vue-components": "^0.22.12",
        "vite": "^5.2.11",
        "vite-plugin-html": "^3.2.0",
        "vue-tsc": "^2.0.19"
    }
}
```

`main.js`:

```js
import { createApp } from "vue";
import { createPinia } from "pinia";

import App from "./App.vue";
import router from "./router";
import "element-plus/dist/index.css";
import ElementPlus from "element-plus";
import zhCn from "element-plus/es/locale/lang/zh-cn"; //配置 Element Plus 的国际化
import "./assets/main.css";
import * as ElementPlusIconsVue from "@element-plus/icons-vue";
// 添加了图标

// 定义特性标志
/* 解决__VUE_PROD_HYDRATION_MISMATCH_DETAILS__警告
/ 禁用对hydration不匹配的详细警告2。 */
// window.__VUE_PROD_DEVTOOLS__ = false;
// window.__VUE_PROD_HYDRATION_MISMATCH_DETAILS__ = false;

// 配置 Element Plus 的国际化
const app = createApp(App);

app.use(ElementPlus, {
    locale: zhCn,
});
for (const [key, component] of Object.entries(ElementPlusIconsVue)) {
    app.component(key, component);
}
//注册全局图标
app.use(createPinia());
app.use(router);
app.mount("#app");

// 安装HTTP中间件
import installHttp from "./plugins/http";
installHttp(router);

// 安装ElIcon
import installElIcon from "./plugins/el-icon";
installElIcon(app);
```


- 似乎没有使用到`vuex`(状态管理)?!