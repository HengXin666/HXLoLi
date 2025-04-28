# 九。Vue 组件生命周期
- 官方文档: [生命周期 | Vue.js](https://cn.vuejs.org/guide/essentials/lifecycle.html)

每个 Vue 组件实例在创建时都需要经历一系列的初始化步骤，比如设置好数据侦听，编译模板，挂载实例到 DOM，以及在数据改变时更新 DOM。在此过程中，它也会运行被称为生命周期钩子的函数，让开发者有机会在特定阶段运行自己的代码。

## 9.1 生命周期图示

| ##container## |
|:--:|
|![vue-20240624193345.png](./vue-20240624193345.png)|
|图片来源: Vue官方文档|

为了方便记忆, 我们可以将他们分类:
- 创建时: `beforeCreate`、`created`
- 渲染时: `beforeMount`、`mounted`
- 更新时: `beforeUpdate`、`updated`
- 卸载时: `beforeUnmount`、`unmounted`

## 9.2 使用示例

应用场景:
- 比如你可以在`组件渲染之后`再向服务器请求(此时展示一个加载中), 这样用户就不会在这里干等待全部请求后统一渲染. (类似于异步加载)

```html
<template>
    <div class="hello">
        <button @click="msg= msg ? '' : '改变'">点击更新</button>
        {{ msg }}
    </div>
</template>

<script>
export default {
    name: 'HelloWorld',
    data() {
        return {
            msg: ''
        }
    },
    beforeCreate() {
        console.log("beforeCreate: 组件创建之前");
    },
    created() {
        console.log("created: 组件创建之后");
    },
    beforeMount() {
        console.log("beforeMount: 组件渲染之前");
    },
    mounted() {
        console.log("mounted: 组件渲染之后");
    },
    beforeUpdate() {
        console.log("beforeUpdate: 组件更新之前");
    },
    updated() {
        console.log("updated: 组件更新之后");
    },
    beforeUnmount() {
        console.log("beforeUnmount: 组件销毁之前");
    },
    unmounted() {
        console.log("unmounted: 组件创建之后");
    }
}
</script>
```