# CSS 简介
## 什么是 CSS
`CSS`是`Cascading Style Sheet`的简写，表示层叠样式表，主要用于渲染HTML元素在网页中的展示效果。主要包括对元素高度、宽度、字体、颜色、背景图片、边距、定位、呈现方式等设定

# CSS 选择器

## 选择器
- CSS 选择分为基本选择器和层次选择器。
- <span style="color:red">CSS 基本选择分为`ID选择器`、`类选择器`和`标签选择器`三大类。
- CSS 选择器有优先级之分: <span style="color:red">ID选择器 > 类选择器 > 标签选择器


```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>demo_2</title>
     <!--css[2]内部样式表, 写在<head>里面, 作用域是整个文件-->
        <!--
            选择器{属性名称:属性值; ...属性键值对...}
            (有大括号就不用双引号了)
        -->

    <!--    选择器是写在 <style>里面写!
    /*标签选择器*/
    html标签名{
        声明1;
        声明2;
        ...
        声明n;
    }

    /*类选择器*/
    .类名{
        声明1;
        声明2;
        ...
        声明n;
    }

    // 主要配合js使用, 大量的请使用类选择器
    /*ID选择器*/
    #ID值{
        声明1;
        声明2;
        ...
        声明n;
    }
    -->

    <style>
        #demo{
            color: aqua;
        }
        
        .red{color: red;}

        .sora{background-color: blue;}

        /* 注意以下两个是不同意思 */
        .awa .qwq .eve { /* 子类 可以间隔多层 <awa><div><span><qwq><div><eve></..</..<... 依旧生效 */
            /* ... */
        }

        .awa > .qwq > .eve { /* awa子类必须是qwq, qwq子类必须是eve才能生效 */
            /* ... */
        }
    </style>
</head>
<!--设置背景(在body)
<body style="background-color: black;">-->
<body style="background-image: url(D:\图片/88438977_p0.png); size: 1920px;">
    <h1>没有设置啊你</h1>
    <h1 id="demo">设置了demo id选择器的属性</h1>
    <h1 class="red">类选择器设置了属性</h1>
    <h1 class="sora">title</h1>
</body>
</html>
```

## 样式引入

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>css demo</title>

    <!--css[2]内部样式表, 写在<head>里面, 作用域是整个文件-->
        <style>
            h1{color: crimson;}
            h2{
                background-color: blueviolet;
                color: brown;
            }
        </style>
        <!--
            选择器{属性名称:属性值; ...属性键值对...}
            (有大括号就不用双引号了)
        -->

</head>
<body>
    <!--css[1]内联样式表 作用在当前标签内-->
    <hr style="border: 3px dashed blue;">
    <!--因为在头部的css内部样式中设置了样式-->
    <h1>title</h1>  
    <h2>mini title</h2>

    <p style="background-color: yellow; color: rgb(33, 211, 60);">css样式字体</p>
    <!--使用方法:
        (""里面用';' 分开, 里面是类似py的字典 呈键值对形式)
        <元素名 style="属性名称:属性值; 属性名称:属性值">(</元素名>)

    -->
    <hr>

    <!--css[3]外部样式列表-->
    <link rel="stylesheet" href="demo.css">
    <h3>demo.css</h3>

    <!--
        样式表优先级
        内联    |       (少用)
        内部    |
        外部    V       (开发推荐)
              最低
    -->
</body>
</html>
```

## CSS 高级选择器
后代选择器

```css
div ul li {
  
}
```

子代选择器

```css
div > ul > li {
  
}
```

上面有说明, 这里就不再赘述