# 五。Vue 列表渲染

- 官方文档: [列表渲染 | Vue.js](https://cn.vuejs.org/guide/essentials/list.html)

## 5.1 v-for

```html
<template>
  <div class="hello">
    <ul>
      <li v-for="i in arr">
        {{ i.name }}
      </li>

      <li v-for="(i, index) in arr">
        {{ index }} - {{ i.name }}
      </li>
    </ul>
  </div>
</template>

<script>
export default {
  name: 'HelloWorld',
  data() {
    return {
      arr: [{
        id: 1001,
        name: 'awa'
      }, {
        id: 1002,
        name: 'qwq'
      }]
    }
  }
}
</script>
```

(支持嵌套for)

## 5.2 通过 key 管理状态
Vue 默认按照“就地更新”的策略来更新通过`v-for`渲染的元素列表。当数据项的顺序改变时，Vue 不会随之移动 DOM 元素的顺序，而是就地更新每个元素，确保它们在原本指定的索引位置上渲染。

默认模式是高效的，但只适用于列表渲染输出的结果不依赖子组件状态或者临时 DOM 状态 (例如表单输入值) 的情况。

为了给 Vue 一个提示，以便它可以跟踪每个节点的标识，从而重用和重新排序现有的元素，你需要为每个元素对应的块提供一个唯一的`key`attribute:

```html
<div v-for="item in items" :key="item.id">
  <!-- 内容 -->
</div>
```
推荐在任何可行的时候为 v-for 提供一个 key attribute，除非所迭代的 DOM 内容非常简单 (例如：不包含组件或有状态的 DOM 元素)，或者你想有意采用默认行为来提高性能。

key 绑定的值期望是一个基础类型的值，例如字符串或 number 类型。不要用对象作为 v-for 的 key。关于 key attribute 的更多用途细节，请参阅 key API 文档。