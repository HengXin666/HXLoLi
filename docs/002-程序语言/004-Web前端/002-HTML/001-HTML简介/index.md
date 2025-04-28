# HTML 简介
## 什么是HTML
HTML是`Hyper Text Markup Language`的简称，即超文本标记语言，是一种用于创建网页的标准标记语言。

HTML 运行在浏览器上，由浏览器来解析执行。

## HTML基本结构

```html
<!DOCTYPE html>
<html>
    <head>
        <meta charset="utf-8">
        <title>标题</title>
    </head>
    <body>
        <!--内容-->
    </body>
</html>
```

### 基本结构说明

- `<!DOCTYPE html>` 表示定义的文档类型为 HTML5 文档。
- `<html></html>` 表示整个 HTML 文档内容的定义只能在该标签对之间
- `<head></head>` 表示整个 HTML 文档的头部信息，比如文档的标题、文档使用的字符集编码、文档是否可以缩放等。
- `<meta charset="utf-8">` 表示定义文档的字符集编码为 "utf-8"，支持中文
- `<title></title>` 表示定义文档显示的标题
- `<body></body>` 表示 HTML 文档的主体内容部分应该定义在该标签内

<span style="color:red">标签一般都是成对出现，分别叫`开放标签`和`闭合标签`</span>

## HTML标签
### HTML 标签分类
HTML 标签分为两大类：**[块级标签](../002-块级标签/index.md)（block elements）** 和 **[行级标签](../003-行级标签/index.md)（inline-block elements）**.