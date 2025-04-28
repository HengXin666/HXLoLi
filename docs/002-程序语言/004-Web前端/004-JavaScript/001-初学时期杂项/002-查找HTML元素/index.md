# 查找 HTML 元素
通常，通过 JavaScript，您需要操作 HTML 元素。

为了做到这件事情，您必须首先找到该元素。有三种方法来做这件事：

- 通过 id 找到 HTML 元素
- 通过标签名找到 HTML 元素
- 通过类名找到 HTML 元素

## 通过 id 查找 HTML 元素

~~(似乎这个是id的常用用法, 被查找(还有一种是超链接定位)(我目前认识的(css用class嘛~)))~~

本例查找 `id="intro"` 元素：
```JS
var x = document.getElementById("intro");
```

如果找到该元素，则该方法将以对象（在 x 中）的形式返回该元素。

如果未找到该元素，则 x 将包含 null。

## 通过标签名查找 HTML 元素

本例查找 `id="main"` 的元素，然后查找 `id="main"` 元素中的所有 `<p>` 元素：
```JS
var x = document.getElementById("main");
var y = x.getElementsByTagName("p");
```

## 通过类名找到 HTML 元素

本例通过 [getElementsByClassName](https://www.runoob.com/jsref/met-document-getelementsbyclassname.html) 函数来查找 `class="intro"` 的元素：

```JS
var x = document.getElementsByClassName("intro");
```