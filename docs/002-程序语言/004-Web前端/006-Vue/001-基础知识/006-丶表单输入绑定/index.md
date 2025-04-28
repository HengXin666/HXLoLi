# 七、Vue 表单输入绑定

- 官方文档: [表单输入绑定 | Vue.js](https://cn.vuejs.org/guide/essentials/forms.html)

## 7.1 表单输入绑定
你可以用`v-model`指令在表单`<input>`、`<textarea>`及`<select>`元素上创建双向数据绑定。它会根据控件类型自动选取正确的方法来更新元素。尽管有些神奇，但v-model本质上不过是语法糖。它负责监听用户的输入事件来更新数据，并在某种极端场景下进行一些特殊处理。

```html
<template>
  <div class="hello">
    <input type="text" v-model="msg" >
    <p>{{ msg }}</p>
  </div>
</template>

<script>
export default {
  name: 'HelloWorld',
  data() {
    return {
      msg: "" // 绑定这个变量
    }
  }
}
</script>
```

## 7.2 修饰符

- `.lazy`: 默认情况下，`v-model`会在每次 input 事件后更新数据 (IME 拼字阶段的状态例外)。你可以添加 lazy 修饰符来改为在每次 change 事件后更新数据。

- `.number`: 如果你想让用户输入自动转换为数字，你可以在 v-model 后添加 .number 修饰符来管理输入。
    - 如果该值无法被 parseFloat() 处理，那么将返回原始值。
    - number 修饰符会在输入框有 type="number" 时自动启用。

- `.trim`: 如果你想要默认自动去除用户输入内容中**两端**的空格，你可以在 v-model 后添加 .trim 修饰符。

```html
<template>
  <div class="hello">
    <input type="text" v-model.lazy="msg1">
    <input type="text" v-model.trim="msg2">
    <p>{{ msg1 }}, {{ msg2 }}</p>
  </div>
</template>

<script>
export default {
  name: 'HelloWorld',
  data() {
    return {
      msg1: "",
      msg2: "",
    }
  }
}
</script>
```