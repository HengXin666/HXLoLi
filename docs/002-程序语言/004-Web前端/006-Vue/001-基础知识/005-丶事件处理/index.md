# 六。Vue 事件处理
## 6.1 监听事件
我们可以使用`v-on`指令 (简写为`@`) 来监听 DOM 事件，并在事件触发时执行对应的 JavaScript。用法: `v-on:click="handler"`或`@click="handler"`。

事件处理器 (handler) 的值可以是：

- 内联事件处理器: 事件被触发时执行的内联 JavaScript 语句 (与 onclick 类似)。

- 方法事件处理器: 一个指向组件上定义的方法的属性名或是路径。

## 6.2 内联事件处理器
内联事件处理器通常用于简单场景，例如:

```html
<template>
  <div class="hello">
    <button @click="count++">Add 1</button>
    <p>Count is: {{ count }}</p>
  </div>
</template>

<script>
export default {
  name: 'HelloWorld',
  data() {
    return {
      count: 0
    }
  }
}
</script>
```

为什么称作内联呢? 考虑一下代码:

```html
<template>
  <div class="hello">
    <button @click="{
      count++; 
      count++;
    }">Add 1</button>
    <p>Count is: {{ count }}</p>
  </div>
</template>
```

类似于匿名函数.. 不就相当于函数内联展开了吗~

## 6.3 方法事件处理器
随着事件处理器的逻辑变得愈发复杂，内联代码方式变得不够灵活。因此`v-on`也可以接受一个方法名或对某个方法的调用。

```html
<template>
  <div class="hello">
    <!-- `greet` 是下面定义过的方法名 -->
    <button @click="greet">Greet</button>
  </div>
</template>

<script>
export default {
  name: 'HelloWorld',
  data() {
    return {
      name: 'Vue.js'
    }
  },
  methods: {
    greet(event) {
      // 方法中的 `this` 指向当前活跃的组件实例 (就是data return 的那部分)
      alert(`Hello ${this.name}!`)
      // `event` 是 DOM 原生事件
      if (event) {
        alert(event.target.tagName)
      }
    }
  }
}
</script>
```

### 6.3.1 方法与内联事件判断
模板编译器会通过检查`v-on`的值是否是合法的 JavaScript 标识符或属性访问路径来断定是何种形式的事件处理器。举例来说，`foo`、`foo.bar`和`foo['bar']`会被视为方法事件处理器，而`foo()`和`count++`会被视为内联事件处理器。

## 6.4 在内联处理器中调用方法 (传参)
除了直接绑定方法名，你还可以在内联事件处理器中调用方法。这允许我们向方法传入自定义参数以代替原生事件:

```html
<template>
  <div class="hello">
    <button @click="say('hello')">Say hello</button>
    <button @click="say('bye')">Say bye</button>
  </div>
</template>

<script>
export default {
  name: 'HelloWorld',
  data() {
    return {
    }
  },
  methods: {
    say(message) {
      alert(message)
    }
  }
}
</script>
```
