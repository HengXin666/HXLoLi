# 改变HTML
## 改变 HTML 输出流
JavaScript 能够创建动态的 HTML 内容：

今天的日期是： <span><script>document.write(Date());</script></span> (中国标准时间)

在 JavaScript 中，`document.write()` 可用于直接向 HTML 输出流写内容。

```html
<!DOCTYPE html>
<html>
    <body>
      
        <script>
            document.write(Date());
        </script>
      
    </body>
</html>
```


> ##yellow##
> [警告] 绝对不要在文档(DOM)加载完成之后使用 document.write()。这会覆盖该文档。

## 改变 HTML 内容
修改 HTML 内容的最简单的方法是使用 `innerHTML` 属性。

如需改变 HTML 元素的内容，请使用这个语法：
```JS
document.getElementById(id).innerHTML = "新的 HTML内容";
```
本例改变了 id 为 `"p1"` 的 `<p>` 元素的内容：

```html
<html>
    <body>
        <p id="p1">Hello World!</p>
      
        <script>
            document.getElementById("p1").innerHTML="新文本!";
        </script>
    </body>
</html>
```

本例改变了 `<h1>` 元素的内容：

```html
<!DOCTYPE html>
<html>
    <body>
        <h1 id="header">Old Header</h1>
        <script>
            var element=document.getElementById("header");element.innerHTML="新标题";
        </script>
    </body>
</html>
```

## 改变 HTML 属性
如需改变 HTML 元素的属性，请使用这个语法：
```JS
document.getElementById(id).attribute = "新属性值";
```
本例改变了 `<img>` 元素的 src 属性：

```html
<!DOCTYPE html>
<html>
    <body>
        <img id="image" src="smiley.gif">
        <script>
            document.getElementById("image").src="landscape.jpg";
        </script>
    </body>
</html>
```