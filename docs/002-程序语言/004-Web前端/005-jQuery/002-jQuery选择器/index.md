# jQuery 语法
## jQuery 选择器
```js
$(CSS选择器|DOM元素);
```

<b style="color:red">注意: jQuery 选择器得到的结果是一个集合</b>

```js
// jQuery使用标签选择器
$("h1"); // 选择所有的h1标签

// jQuery使用类选择器
$(".box"); // 选择所有的使用了类样式box的标签

// jQuery使用ID选择器
$("#loginBtn"); // 选择使用了ID为loginBtn的标签,如果存在多个,则只选择第一个

// jQuery使用并集选择器
$("h1, div"); // 选择所有的h1、div标签

// jQuery使用后代选择器
$(".box input"); // 选择使用了类样式box的标签下的所有input标签

// jQuery使用子代选择器
$(".box > input"); // 选择使用了类样式box的标签下的下一级input标签

// jQuery使用属性选择器
$("[type='text']"); // 选择所有带有type='text'属性的标签

// jQuery使用过滤选择器
$("div:first"); // 选择所有div标签中的第一个div标签

// jQuery使用过滤选择器
$("div:last"); // 选择所有div标签中的最后一个div标签

// jQuery使用过滤选择器
$("div:even"); // 选择所有div标签中的下标是偶数的div标签

// jQuery使用过滤选择器
$("div:odd"); // 选择所有div标签中的下标是奇数的div标签

// jQuery使用过滤选择器
$("div:eq(0)"); // 选择所有div标签中的下标是0的div标签

// jQuery使用过滤选择器
$("div:gt(0)"); // 选择所有div标签中的下标大于0的div标签

// jQuery使用过滤选择器
$("div:lt(0)"); // 选择所有div标签中的下标小于0的div标签

// jQuery使用过滤选择器
$("input:focus"); // 选择所有获得焦点的input标签

$(document); // 选择整个文档
```

<div style="margin-top: 80px;">

---
</div>

## ready 方法
```js
$(选择器|DOM元素).ready(回调函数);
```
`ready`方法表示 `jQuery`选择器选择的标签准备好了之后需要执行的后续动作。通常用于表示文档准备好之后需要做的后续动作。

```js
$(document).ready(function() { // 表示文档准备好之后
    // 需要执行的jQuery脚本代码
});

// 省略的等价写法
$(function() { // 表示文档准备好之后
    // 需要执行的jQuery脚本代码
});
```