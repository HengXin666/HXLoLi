# CSS 样式
## 字体 font

```html
<!--字体-->

<!--
    font-style  字体风格
    font-weight 字体纤细
    font-size   字体大小
    font-family 字体类型

    字体的复合属性是有顺序的：风格 粗细 大小 类型
-->
<h1 style="font-style: italic;  
font-weight: 200;
font-size: medium;
font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;">
字体
</h1>

<!--
    在一行总设置所有字体属性
-->
<h1 style="font: italic bold 40px '萝莉体';">114514</h1>
```

示例:

<p style="font-style: italic;  
font-weight: 200;
font-size: medium;
font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;">字体</p>

<p style="font: italic bold 40px '萝莉体';">114514<p>

<div style="margin-top: 80px;">

---
</div>

## 文本 text
```html
<!--文本-->
<!--
    color           颜色
    text-align      元素对齐方式    center(居中)
    text-indent     文本首行的缩进(比如开头空几格)
    line-height     设置文本行高
    text-decoration 文本的装饰
-->
<h1 style="color: blueviolet; text-align:center; text-indent: 32px; line-height: 32px; text-decoration: double;">
文本
</h1>
```

示例:

<p style="color: blueviolet; text-align:center; text-indent: 32px; line-height: 32px; text-decoration: double;">文本</p>

<div style="margin-top: 80px;">

---
</div>


## 背景 background
```html
<!--背景-->
<!--
    background-color    背景颜色
    background-image    背景图片    background-image:url(图片地址)
    background-position 背景定位
    background-repeat   背景重复方式
-->
<h1 style="background-color: red; background-image: none; background-position: 205px 10px; background-repeat: no-repeat">
背景
</h1>

<!--
    background 一行背景属性
-->
<h1 style="background: yellow 100px 100px no-repeat;">
一行背景属性
</h1>
```

示例:

<p style="background-color: red; background-image: none; background-position: 205px 10px; background-repeat: no-repeat">背景</p>

<div style="margin-top: 80px;">

---
</div>

## 边框 border
```html
<!--边框-->
<!--
    border-color    边框颜色
    border-width    边框大小
    border-style    边框样式
-->
<div style="border-color: green; border-width: 8px; border-style: solid;">
    <h1>
        边框
    </h1>

    <h1 style="border-color: rebeccapurple; border-width: 2px; border-style: solid;">
        我怎么在边框里面?    
    </h1>

    <!--
        一行边框属性    好像不能用
    -->
    <div style="border: red 3px 3px solid;">
        这个是一行边框属性
    </div>
</div>
```

示例:

<div style="border-color: green; border-width: 8px; border-style: solid;">
    <p>
        边框
    </p>
    <p style="border-color: rebeccapurple; border-width: 2px; border-style: solid;">
        我怎么在边框里面?    
    </p>
    <div style="border: red 3px 3px solid;">
        <p>
            这个是一行边框属性-好像不能用
        </p>
    </div>
</div>

<div style="margin-top: 80px;">

---
</div>

## 边距
### 外边距 margin
```html
<!--        外边距
    边距有4个方向：上、下、左、右
    外边距： margin
    margin-top: 2px;
    margin-bottom: 2px;
    margin-left: 2px;
    margin-right: 2px;

    这个是总的
    margin: 2px;
-->
<div style="border-color: green; border-width: 8px; border-style: solid; margin: 20px;">
    外边距
</div>
```

示例:

<div style="border-color: green; border-width: 8px; border-style: solid; margin: 20px;">外边距</div>

<div style="margin-top: 80px;">

---
</div>

### 内边距 padding
```html
<!--        内边距
    内边距：padding
    padding-top: 2px;
    padding-bottom: 2px;
    padding-left: 2px;
    padding-right: 2px;

    这个是总的
    padding: 2px;
-->

<div style="border-color: green; border-width: 8px; border-style: solid; padding: 20px;">
    内边距
</div>
```

示例:

<div style="border-color: green; border-width: 8px; border-style: solid; padding: 20px;">内边距</div>

<div style="margin-top: 80px;">

---
</div>

## 浮动 float
元素浮动有两个方向：left 和 right

float属性指定一个盒子（元素）是否应该浮动。

**注意**: 绝对定位的元素忽略float属性！

```html
<p style="float: left">内容left--------------------------------</p>
<p style="float: right">--------------------------------内容right</p>
```

示例: (尝试改变窗口大小, 他们是呈现在一行的)

<p style="float: left">内容left--------------------------------</p>
<p style="float: right">--------------------------------内容right</p>

<br><br>

<div style="margin-top: 80px;">

---
</div>

## 清除浮动 clear
清除浮动有三种选择：left、right 和 both。

浮动的元素与其他元素不在同一个层级，清除浮动后，浮动的元素就与其他元素在同一个层级了

```html
<div style="background: red">
    <div style="width:200px; height:100px; background:black; float:left;">1</div>
    <div style="width:200px; height:100px; background:orange; float:right;">2</div>
    <div style="width:200px; height:100px; background:yellowgreen; clear:both;">3</div>
</div>
```

示例: (尝试改变窗口大小)

<div style="background: red">
<div style="width:200px; height:100px; background:black; float:left;">1</div>
<div style="width:200px; height:100px; background:orange; float:right;">2</div>
<div style="width:200px; height:100px; background:yellowgreen; clear:both;">3</div>
</div>

<div style="margin-top: 80px;">

---
</div>

## 定位 position
元素定位分为无定位、绝对定位、相对定位和固定定位四种。元素定位是根据参照物来进行定位，定位时根据元素与参照物上下左右四个方向中任意相邻的两个方向的距离来进行定位，定位方式不同，参照物也不一样。元素定位默认为无定位。<span style="color:red">绝对定位和固定定位的元素必须设置宽度和高度</span>

| 属性值 | 说明 |
| --- | --- |
| static | 默认值，没有定位 |
| relative | 相对自身进行定位 |
| absolute | 绝对含有定位的最近的父容器进行定位 |
| fixed | 相对于浏览器窗口进行固定定位 |

```html
<!DOCTYPE html>
<html>
<body>

<h2>定位属性示例</h2>

<div style="position: static; border: 1px solid red; width: 200px;">
  <p>这是一个静态定位的元素（position: static）。</p>
</div>

<div style="position: relative; top: 20px; left: 20px; border: 1px solid blue; width: 200px;">
  <p>这是一个相对定位的元素（position: relative）。</p>
</div>

<div style="position: absolute; top: 80px; right: 0; border: 1px solid green; width: 200px;">
  <p>这是一个绝对定位的元素（position: absolute）。</p>
</div>

<div style="position: fixed; bottom: 0; right: 0; border: 1px solid yellow; width: 200px;">
  <p>这是一个固定定位的元素（position: fixed）。</p>
</div>

</body>
</html>
```

示例:

<div style="position: static; border: 1px solid red; width: 240px;">
  <p>这是一个静态定位的元素（position: static）。</p>
</div>

<div style="position: relative; top: 30px; left: 20px; border: 1px solid blue; width: 240px;">
  <p>这是一个相对定位的元素（position: relative）。</p>
</div>

剩下2个略...

<div style="margin-top: 80px;">

---
</div>

## 列表样式 list
```html
<!--
    list-style-type     设置列表每一项前面的修饰类型
    list-style-image    设置列表每一项前面图片
    list-style-position 设置列表每一项前面的位置(缩进)

    list-style          一行列表属性
-->
<ul style="list-style-type:hiragana; list-style-image: none; list-style-position: inside;">
    <li>我</li>
    <li>是</li>
    <li>誰</li>
</ul>
```

示例:

<ul style="list-style-type:hiragana; list-style-image: none; list-style-position: inside;">
    <li>我</li>
    <li>是</li>
    <li>誰</li>
</ul>

<div style="margin-top: 80px;">

---
</div>

## 伪类样式 :hove
常用的伪类样式是鼠标悬浮的伪类样式 `:hove`

```html
<style>
.xxx_name:hover {
    background: red;
    }
</style>

<h1 class="xxx_name">
  饿哦
</h1>
```

示例:

<style>
.xxx_name:hover {
    background: red;
    }
</style>

<p class="xxx_name">
  饿哦
</p>

---

### 其他伪类样式

| 伪类名称 | 含义 | 示例 |
| --- | --- | --- |
| a:link | 未单击访问时超链接样式 | `a:link{color:black;}` |
| a:visited | 单击访问后超链接样式 | `a:visited {color:pink;}` |
| a:hover | 鼠标悬浮其上的超链接样式 | `a:hover{color:red;}` |
| a:active | 鼠标单击未释放的超链接样式 | `a:active {color:orange;}` |

当超链接同时拥有上面的伪类样式时，其书写顺序有要求: `a:link->a:visited->a:hover->a:active`

```html
<!-- 建议使用tm的 类选择器, 下面代码是ai写的 -->

<a href="#" style="color:black; text-decoration:none;">未单击访问时的超链接样式</a><br>

<a href="#" style="color:pink; text-decoration:none;">单击访问后的超链接样式</a><br>

<a href="#" onmouseover="this.style.color='red'" onmouseout="this.style.color='black'" style="color:black; text-decoration:none;">鼠标悬浮其上的超链接样式</a><br>

<a href="#" onmousedown="this.style.color='orange'" onmouseup="this.style.color='black'" style="color:black; text-decoration:none;">鼠标单击未释放的超链接样式</a><br>

<!-- 例如: -->
<style>
.button /* 补充了一点点细节 */
{
    display: inline-block;
    border-radius: 4px;
    background-image: linear-gradient(rgba(235, 213, 29), rgba(169, 8, 161));
    border: none;
    color: rgb(11, 178, 2);
    text-align: center;
    font-size: 28px;
    padding: 20px;
    width: 200px;
    transition: all 0.5s;
    cursor: pointer;
    margin: 5px;
    opacity: 0.8;
}
  
.button a:after 
{
    content: '>>';
    position: absolute;
    opacity: 0;
    top: 5px;
    right: -20px;
    transition: 0.5s;
}

.button:hover a 
{
    padding-right: 35px;
}

.button:hover a:after 
{
    opacity: 1;
    right: 0;
}

a:hover
{
    color: rgb(255, 255, 255);
}
</style>
```

示例:

<style>
.button 
{
    display: inline-block;
    border-radius: 4px;
    background-image: linear-gradient(rgba(235, 213, 29), rgba(169, 8, 161));
    border: none;
    color: rgb(11, 178, 2);
    text-align: center;
    font-size: 28px;
    padding: 20px;
    width: 200px;
    transition: all 0.5s;
    cursor: pointer;
    margin: 5px;
    opacity: 0.8;
}
  
.button a:after 
{
    content: '>>';
    position: absolute;
    opacity: 0;
    top: 5px;
    right: -20px;
    transition: 0.5s;
}

.button:hover a 
{
    padding-right: 35px;
}

.button:hover a:after 
{
    opacity: 1;
    right: 0;
}

a:hover
{
    color: rgb(255, 255, 255);
}
</style>

<a href="#" class="button"> awa </a>