# JavaScript
## JavaScript 简介
JavaScript 是一种解释性脚本语言（不用编译），主要用于向 HTML 添加交互行为，语法与 Java 语言类似。

JavaScript 由 ECMAScript（简称 ES）、DOM（Document Object Model） 和 BOM（Broswer Object Model） 三大部分组成。

## JavaScript 基本结构

```html
<script type="text/javascript">
// JavaScript 代码
</script>
```

该结构可以在HTML中的任意位置书写，但必须保证 JavaScript 脚本中使用到的元素必须在 JavaScript 脚本执行前完成加载。

```html
<div id="a1"></div>

<script type="text/javascript">
// 此处不能访问到 a2
</script>

<div id="a2"></div>
```

## JavaScript 执行过程
用户从浏览器发出页面请求，服务器接收请求并进行处理，处理完成后会将页面返回至浏览器，浏览器开始解释执行该页面，如果页面中包含有 JavaScript 脚本，那么浏览器会再次向服务器发出 JavaScript脚本获取请求，服务器接收请求并进行处理，处理完成后会将 JavaScript 脚本返回至浏览器，浏览器开始解释执行JavaScript 脚本。

## JavaScript 引入方式
JavaScript 的引入方式与 CSS 样式引入方式是一致的，分为行内脚本、内部脚本和外部脚本。

### 行内脚本

```html
<input type="button" value="点击" onclick="alert('你点击了按钮');">
```

### 内部脚本

```html
<input type="button" value="点击" id="btn">
<script type="text/javascript">
    document.getElementById("btn").onclick=function(){
        alert('你点击了按钮');
    }
</script>
```

### 外部脚本

```js
// demo.js
document.getElementById("btn").onclick=function(){
    alert('你点击了按钮');
}
```

```html
<!-- demo.html -->
<input type="button" value="点击" id="btn">
<script type="text/javascript" src="demo.js"></script>
```