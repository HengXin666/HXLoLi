# 十五、Vue3 新特性
Vue3是目前Vue的最新版本，自然也是新增了很多新特性
## 15.1 六大亮点
- **Performance**: 性能更比Vue2.0强。

- **Tree shaking support**: 可以将无用模块“剪辑”，仅打包需要的。

- **CompositionAPl**: **组合API**

- **Fragment,Teleport,Suspense**: “碎片"，Teleport即Protal传送门，“悬念"

- **BetterTypeScript support**: 更优秀的Ts支持

- **CustomRendererAPl**: 暴露了自定义渲染API

## 15.2 ref 和 reactive

- 这里学习一个JS的新语法: [Vue3解构赋值：让你的代码更简洁、优雅和高效](https://foreval.cn/archives/vue3-jie-gou-fu-zhi-)

- 详细学习:

    - [Vue3超详细的ref()用法，看这一篇就够了](https://blog.csdn.net/EchoLiner/article/details/130445600)

    - [Vue3响应式：ref vs reactive，5分钟消除使用困惑](https://segmentfault.com/a/1190000044769265)

示例:

```html
<template>
  <div class="about">
    <h1>This is an about page</h1>
    <p>cnt = {{ $store.state.cnt }}</p>
    <p>{{ msg }}</p>
    <p v-for="(i, index) in arr.list" :key="index">
      {{ i }}
    </p>
    <button @click="fun">消息</button>
  </div>
</template>

<script>
import { ref, reactive } from "vue"; // 需要导入

export default {
  setup() {
    const msg = ref("我是消息");

    const arr = reactive({
      list: ["awa", "qwq", "0.0"]
    });

    function fun() {
      msg.value = "新的消息~"; // 需要通过 .value 修改和访问 ref (在script中)
    };

    return { // 如果需要使用则需要return
      msg,
      arr,
      fun
    };
  }
}
</script>
```


## 15.3 setup() 中使用 props 和 context
在2.x中，组件的方法中可以通过this获取到当前组件的实例，并执行data变量的修改，方法的调用，组件的通信等等，但是在3.x中，setup(在beforeCreate和created时机就已调用，无法使用和2.x一样的this，但是可以通过接收setup(props,ctx)的方法，获取到当前组件的实例和props

示例:

```html
<!-- 父文件: 传递信息 -->
<HelloWorld msg="Welcome to Your Vue.js App"/>

<!-- HelloWorld.vue -->
<template>
  <div class="hello">
    <p>cnt = {{ $store.state.cnt }}</p>
    <p>{{ msg }}</p>
    <p v-for="(i, index) in arr.list" :key="index">
      {{ i }}
    </p>
    <button @click="fun">消息</button>
  </div>
</template>

<script>
import { ref, reactive } from "vue";

export default {
  props: { // 这个还是要写的
    msg: String
  },
  setup(props, ctx) {
    const msg = ref("我是消息");

    console.log(props.msg); // 传递消息

    const arr = reactive({
      list: ["awa", "qwq", "0.0"]
    });

    console.log(this); // 没有值
    console.log(ctx); // 在 setup中是没有this的, 因为它比this还要先创建
    
    function fun() {
      msg.value = "新的消息~"; // 需要通过 .value 修改和访问 ref (在script中)
      console.log(this); // 可以使用
      console.log(ctx); // 在 setup中是没有this的, 因为它比this还要先创建
    };

    return { // 如果需要使用则需要return
      msg,
      arr,
      fun
    };
  }
}
</script>
```

## 15.4 在setup中使用生命周期函数
- [组合式 API：生命周期钩子 | Vue.js](https://cn.vuejs.org/api/composition-api-lifecycle.html)

| Vue 2 生命周期钩子 | Vue 3 Composition API 等效项（在`setup`中使用） |
| :--: | :--: |
| beforeCreate | **Not needed** * (在 `setup` 调用前，组件实例尚未创建) |
| created | **Not needed** * (在 `setup` 调用时，组件实例尚未创建) |
| beforeMount | `onBeforeMount` |
| mounted | `onMounted` |
| beforeUpdate | `onBeforeUpdate` |
| updated | `onUpdated` |
| beforeUnmount | `onBeforeUnmount` |
| unmounted | `onUnmounted` |

*注意: 在 Vue 3 的 Composition API 中，`setup` 函数在 `beforeCreate` 和 `created` 生命周期钩子之前被调用，因此这两个钩子在 Composition API 中没有直接的等效项。在 `setup` 函数内部，你可以直接进行初始化操作，因为此时组件实例尚未创建，但你可以访问 `props` 和 `context`。*

示例:

```html
<script>
import { onMounted } from "vue";

export default {
  setup() { // 可以使用多次相同的生命周期函数执行不同逻辑, 顺序从上到下
    onMounted(() => {
      console.log("您好");
    });
    onMounted(() => {
      console.log("HX");
    });
  }
}
</script>
```

## 15.5 Provide / inject 嵌套组件数据传递
- `provide()`和`inject()`可以实现嵌套组件之间的数据传递。

- 这两个函数只能在`setup()`函数中使用。

- 父级组件中使用`provide()`函数向下传递数据。

- 子级组件中使用`inject()`获取上层传递过来的数据。

- 不限层级

示例: 关系: A -> B -> C

```html
<template>
    <dev>
        A
        <bVue/>
    </dev>
</template>

<script>
import bVue from "./b.vue";
import { provide } from "vue";

export default {
    components: {
        bVue
    },
    setup() {
        provide("toC_msg", "我是父组件啊~"); // 传递
    }
}
</script>
```

```html
<template>
    <dev>
        B
        <cVue/>
    </dev>
</template>

<script>
import cVue from "./c.vue";
export default {
    components: {
        cVue
    }
}
</script>
```

```html
<template>
    <dev>
        C {{ msg }}
    </dev>
</template>

<script>
import { inject } from "vue";

export default {
    setup() {
        const msg = inject("toC_msg"); // 接收
        return {
            msg
        }
    }
}
</script>
```