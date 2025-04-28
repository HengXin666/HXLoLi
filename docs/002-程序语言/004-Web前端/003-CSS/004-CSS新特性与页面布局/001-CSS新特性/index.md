# CSS3 新特性
## 边框 border-radius
```html
<p style="border-radius:30px; background-color:red;">圆角边框</p>
```

示例:

<d style="border-radius:30px; background-color:red;">_圆角边框\_</d>

<d style="border-radius: 180px; background-color:red;"> $圆角边框$ </d>

## 盒子阴影 box-shadow

```css
box-shadow:阴影类型 水平阴影位置 垂直阴影位置 阴影模糊距离 阴影大小 阴影颜色;
```

```html
<p style="box-shadow: inset 2px 2px 2px 2px red;">盒子阴影</p>

<span style="box-shadow: 2px 2px 2px 2px red;">盒子阴影</span>
```

示例:

<span style="box-shadow: inset 2px 2px 2px 2px red;">盒子阴影</span>

<span style="box-shadow: 2px 2px 2px 2px red;">盒子阴影</span>

## 渐变 Gradients
### 线性渐变 `Linear Gradients`
- 颜色沿着一条直线过渡: 从左到右、从右到左、从上到下等

  ```css
  linear-gradient(渐变方向, 颜色1, yanse2, ..., 颜色n)
  ```


  ```css
  .block1 {
      /* 从上到下的线性渐变： */
      background: linear-gradient(red, blue);
  }
  
  .block2 {
      /* 从左到右的线性渐变：*/
      background: linear-gradient(to right,red, blue);
  }
  
  .block3 {
      /* 从左上角到右下角的线性渐变：*/
      background: linear-gradient(to bottom right, red , blue);
  }
  ```

示例:

<div style="height: 100px; background: linear-gradient(red, blue)">从上到下的线性渐变</div><br>

<div style="height: 100px; background: linear-gradient(to right,red, blue)">从左到右的线性渐变</div><br>

<div style="height: 100px; background: linear-gradient(to bottom right, red , blue)">从左上角到右下角的线性渐变</div>

### 径向渐变 `Radial Gradients`
- 圆形或椭圆形渐变，颜色不再沿着一条直线变化，而是从一个起点朝所有
方向混合

    ```css
    radial-gradient(center, shape size, start-color, ..., last-color);
    ```

    ```css
    .block1 {
        /* 颜色结点均匀分布的径向渐变：*/
        background: radial-gradient(red, green, blue);
    }
    
    .block2 {
        /* 颜色结点不均匀分布的径向渐变： */
        background: radial-gradient(red 5%, green 15%, blue 60%);
    }
    
    .block3 {
        /* 形状为圆形的径向渐变：*/
        width: 600px;height: 400px;
        background: radial-gradient(circle, red, yellow, green);
    }
    ```

示例:
<div style="height: 100px; background: radial-gradient(red, green, blue)">颜色结点均匀分布的径向渐变</div><br>

<div style="height: 100px; background: radial-gradient(red 5%, green 15%, blue 60%)">颜色结点不均匀分布的径向渐变</div><br>

<div style="height: 100px; background: radial-gradient(circle, red, yellow, green)">形状为圆形的径向渐变</div>

## 文本效果
### 向文本添加阴影 text-shadow

| 值 | 说明 |
| --- | --- |
| h-shadow | 必需，水平阴影的位置，允许负值 |
| v-shadow | 必需，垂直阴影的位置，允许负值 |
| blur | 可选，模糊距离 |
| color | 可选，阴影的颜色 |


### 当文本溢出包含元素时发生的事情 text-overflow
- 超出部分显示省略号
    - `white-space:nowrap` 文本不会换行，在同一行继续
    - `overflow:hidden` 溢出隐藏
    - `text-overflow:ellipsis` 用省略号来代表被修剪的文本


```css
/*文本阴影与盒子阴影的区别在于：文本阴影无内外之分，且文本阴影没有阴影大小的设置*/
text-shadow: 2px 2px 2px red;

/*文本溢出时不换行*/
white-space: nowrap;

/*元素溢出部分隐藏掉*/
overflow: hidden;

/*文本溢出部分使用省略号显示*/
text-overflow: ellipsis;

/* 需要叠加使用 */
<p style="width: 50px; white-space: nowrap">文本溢出时不换行</p>

<p style="width: 50px; white-space: nowrap; overflow: hidden">元素溢出部分隐藏掉</p>

<p style="width: 50px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis">文本溢出部分使用省略号显示</p>
```

示例:

<p style="text-shadow: 2px 2px 2px red">文本阴影与盒子阴影的区别在于：文本阴影无内外之分，且文本阴影没有阴影大小的设置</p>

<p style="width: 50px; white-space: nowrap">文本溢出时不换行</p>

<p style="width: 50px; white-space: nowrap; overflow: hidden">元素溢出部分隐藏掉</p>

<p style="width: 50px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis">文本溢出部分使用省略号显示</p>

## 字体 @font-face

```css
@font-face {
    font-family: 必需。规定字体的名称
    src: 必需。定义字体文件的 URL
    font-weight: 可选。定义字体的粗细。默认是 "normal"
    font-style: 可选。定义字体的样式。默认是 "normal"
}
```

- 一般是用于那种字体表情, 它可以通过`color`属性进行变色, 而不是图片表情

## 变形
- CSS3 变形是一些效果的集合。如平移、旋转、缩放、倾斜效果；每个效果都可以称为变形（transform），它们可以分别操控元素发生平移、旋转、缩放、倾斜等变化

  ```css
  /*transform-function是一个变形函数，可以是一个，也可以是多个，中间以空格分开*/
  transform:[transform-function];
  ```

### 平移 translate(X, Y)
- translate(x, y)：平移函数，基于X、Y坐标重新定位元素的位置
- translateX(x)：表示只设置X轴的位移
- translateY(y)：表示只设置Y轴的位移

示例
```css
transform: translate(20px, 30px);
transform: translateX(20px);
transform: translateY(20px);
```
> > <p>正常情况</p>

> > <p style="transform: translate(20px, 30px)">translate(20px, 30px)</p>

> > <p style="transform: translateX(20px)">translateX(20px)</p>

> > <p style="transform: translateY(20px)">translateY(20px)</p>

### 2D 缩放 scale(X, Y)
- scale(x, y)：缩放函数，可以使任意元素对象尺寸发生变化。当该函数只接收一个值时，表示同时设置X与Y的值
- scaleX(x)：表示只设置X轴的缩放
- scaleY(y)：表示只设置Y轴的缩放

示例
```css
transform: scale(0.5, 1.5);
transform: scaleX(0.5);
transform: scaleY(0.5);
```

> > <p>正常情况</p>

> > <p style="transform: scale(0.5, 1.5)">transform: scale(0.5, 1.5)</p>

> > <p style="transform: scaleX(0.5)">scaleX(0.5)</p>

> > <p style="transform: scaleY(0.5)">scaleY(0.5)</p>

### 旋转 rotate(deg)
- rotate(degree)：旋转函数，取值是一个度数值。参数degree单位使用deg表示，参数degree取正值时元素相对原来中心顺时针旋转

- 可以理解`deg`为'`°`'， (> 360°就是转动 > 1圈)

示例
```css
transform: rotate(10deg);
```

> > <p>正常情况</p>

> > <p style="transform: rotate(10deg)">transform: rotate(10deg)</p>

### 倾斜 skew(X, Y)
- skew(x, y)：倾斜函数，取值是一个度数值。
- skewX(x)：表示只设置X轴的倾斜
- skewY(y)：表示只设置Y轴的倾斜

示例

```css
transform: skew(20deg, 60deg);
transform: skewX(45deg);
transform: skewY(45deg);
```

> > <p>正常情况</p>

> > <p style="transform: skew(20deg, 10deg)">skew(20deg, 10deg)</p>

> > <p style="transform: skewX(50deg)">transform: skewX(50deg)</p>

> > <p style="transform: skewY(10deg)">transform: skewY(10deg)</p>

## 过渡 transition

- transition呈现的是一种过渡，是一种动画转换的过程，如渐现、渐弱、动画快慢等

- CSS3 transition的过渡功能通过一些 CSS 的简单动作触发样式平滑过渡

```css
transition:[transition-property transition-duration transition-timing-function transition-delay ]
```
- transition-property：
    - 过渡或动态模拟的 CSS 属性，为了方便，一般都指定all，表示所有属性

- transition-duration：
    - 完成过渡所需要的时间，即从设置旧属性到换新属性所花费的时间，单位为秒（s）

- transition-timing-function：指定过渡函数
    - linear：规定以相同速度开始至结束的过渡效果
    - ease：规定慢速开始，然后变快，然后慢速结束的过渡效果（默认值）
    - ease-in：规定以慢速开始的过渡效果
    - ease-out：规定以慢速结束的过渡效果
    - ease-in-out：规定以慢速开始和结束的过渡效果

- transition-delay：
    - 过渡开始出现的延迟时间。
    - 正值表示元素过渡效果不会立即触发，当过了设置的时间值后才会被触发；
    - 负值表示元素过渡效果会从该时间点开始显示，之前的动作被截断；
    - 0是默认值，元素过渡效果立即执行

- 过渡效果的出发时机
    - 伪类触发： `:hover` `:active` `:focus` `:checked`
    - 媒体查询：通过`@media`属性判断设备的尺寸，方向等
    - JavaScript触发：用JavaScript脚本触发

示例

```css
.tran {
    width: 200px;
    height: 200px;
    background-color: red;
    /*宽度发生变化时就会触发过渡效果*/
    transition: width .5s ease 0s;
}

.tran:hover {
    width: 50px;
}
```

<style>
.tran {
    width: 200px;
    height: 200px;
    transform: skew(0deg, 0deg) rotate(0deg);
    background-color: red;
    /*宽度发生变化时就会触发过渡效果*/
    transition: .75s ease 0s;
}

.tran:hover {
    transform: skew(10deg, 180deg) rotate(360deg);
    /* width: 50px; */
}
</style>

<div class="tran">
  <p>Text</p>
</div>

## 媒体查询 @media screen

```css
@media mediatype and|not|only (media feature) {
    CSS-Code;
}
```

- mediatype : 表示媒体类型
    - all：用于所有设备
    - screen：用于电脑屏幕，平板电脑，智能手机等。

- media feature ：表示媒体功能
    - max-width：定义输出设备中的页面最大可见区域宽度。
    - min-width：定义输出设备中的页面最小可见区域宽度。

示例

```css
.box {
    background-color: red;
    height: 50px;
}

@media screen and (min-width: 700px) {
    .box {
        width: 200px;
    }
}

@media screen and (min-width: 900px) {
    .box {
        width: 300px;
    }
}

@media screen and (min-width: 1200px) {
    .box {
        width: 400px;
    }
}
```

<style>
.box {
    background-color: red;
    height: 50px;
}

@media screen and (min-width: 700px) {
    .box {
        width: 200px;
    }
}

@media screen and (min-width: 900px) {
    .box {
        width: 300px;
    }
}

@media screen and (min-width: 1200px) {
    .box {
        width: 400px;
    }
}
</style>

<div class="box">请尝试改变浏览器大小</div>