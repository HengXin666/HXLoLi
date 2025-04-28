# 改变CSS
## 改变 HTML 样式
如需改变 HTML 元素的样式，请使用这个语法：
```JS
document.getElementById(id).style.property = "新样式";
```
下面的例子会改变 `<p>` 元素的样式：
```html
<!DOCTYPE html>
<html>
    <head>
        <meta charset="utf-8">
          <!--这里示例均来自菜鸟教程-->
        <title>菜鸟教程(runoob.com)</title>
    </head>
    <body>
        <p id="p1">Hello World!</p>
        <p id="p2">Hello World!</p>
        <script>
            document.getElementById("p2").style.color="blue";
            document.getElementById("p2").style.fontFamily="Arial";
            document.getElementById("p2").style.fontSize="larger";
        </script>
        <p>以上段落通过脚本修改。</p>
    </body>
</html>
```

## 使用事件
HTML DOM 允许我们通过触发事件来执行代码。

比如以下事件：

- 元素被点击。
- 页面加载完成。
- 输入框被修改。
- ……

(更多请看下一章节)

本例改变了 `id="id1"` 的 HTML 元素的样式，当用户点击按钮时：

```html
<!DOCTYPE html>
<html>
    <body>
        <h1 id="id1">我的标题 1</h1>
            <button type="button"
              onclick="document.getElementById('id1').style.color='red'">
                点我!</button>
    </body>
</html>
```