# QML - Item与Rectangle
## 1. z (z-index)

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("02: Hello World")

    Rectangle { // A
        x: 50
        y: 50
        width: 50
        height: 50
        color: "#990099"
    }

    Rectangle { // B
        x: 75
        y: 75
        width: 50
        height: 50
        color: "#03f"
    }
}
```

对于上面这种, 后来的 `B` 会把先来的 `A` 盖住的情况, 我们可以使用类似于`css`的`z-index`的属性,

但在`qml`中, 它叫做: `z`

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("02: Hello World")

    Rectangle {
        x: 50
        y: 50
        z: 2
        width: 50
        height: 50
        color: "#990099"
    }

    Rectangle {
        x: 75
        y: 75
        z: 1
        width: 50
        height: 50
        color: "#03f"
    }
}
```

这样就不会被覆盖了!

## 2. 锚点 (anchors)
### 2.1 元素 (Item)

> 像是下面的 parent, 就是一个 Item

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("02: Hello World")

    Rectangle {
        anchors.fill: parent // 描点: 设置当前控件的大小填充满整个`parent`(父控件)
        color: "red"
    }
}
```

### 2.2 示例: 对齐跟随元素后边

有时候, 如果我们希望一个控件在另一个控件的后边, 可能会这样写:

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("02: Hello World")

    Rectangle {
        id: rect1
        x: 50
        y: 200
        width: 50
        height: 50
        color: "#3c3c3c"
    }

    Rectangle {
        id: rect2
        x: rect1.x + 75 // 写的是绝对位置
        y: 200
        width: 50
        height: 50
        color: "#3c3c3c"
    }
}
```

但还有一种更方便、灵活的写法是, 就是使用锚点:

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("02: Hello World")

    // 基于锚点的写法
    Rectangle {
        id: rect3
        x: 50
        y: 300
        width: 50
        height: 50
        color: "#3c3c3c"
    }

    Rectangle {
        id: rect4
        anchors.left: rect3.right // 左边对齐 rect3 的右边
        anchors.top: rect3.top    // 与 rect3 的上边对齐 (不然y就是0了)
        anchors.leftMargin: 20    // 外边距是 20
        width: 50
        height: 50
        color: "#3c3c3c"
    }
}
```

### 2.3 示例: 居中

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("02: Hello World")

    // 居中
    Rectangle {
        width: 50
        height: 50
        color: "#666abc"
        anchors.horizontalCenter: parent.horizontalCenter
    }
}
```

### 2.4 常用属性

🔹 常见锚点属性列表:

| 属性名 | 说明 |
| :-: | - |
| `anchors.left` | 元素的左边缘锚定到某个目标元素的左边缘(或其他边缘)|
| `anchors.right` | 元素的右边缘锚定到某个目标元素的右边缘(或其他边缘)|
| `anchors.top` | 元素的上边缘锚定到某个目标元素的上边缘(或其他边缘)|
| `anchors.bottom` | 元素的下边缘锚定到某个目标元素的下边缘(或其他边缘)|
| `anchors.horizontalCenter` | 元素的水平中心线锚定到目标元素的水平中心线|
| `anchors.verticalCenter` | 元素的垂直中心线锚定到目标元素的垂直中心线|
| `anchors.centerIn` | 将当前元素的中心锚定在指定目标的中心(是 horizontalCenter 和 verticalCenter 的组合)|
| `anchors.fill` | 使当前元素填充指定目标的全部空间，相当于将四个边(left, right, top, bottom)都锚定到目标|

🔹 辅助属性:

| 属性名 | 说明 |
| :-: | - |
| `anchors.margins` | 设置统一的外边距，作用于所有方向的锚点|
| `anchors.leftMargin` | 左侧外边距|
| `anchors.rightMargin` | 右侧外边距|
| `anchors.topMargin` | 顶部外边距|
| `anchors.bottomMargin` | 底部外边距|

## 3. 旋转 (rotation)

```qml
Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("02: Hello World")

    // 旋转
    Rectangle {
        width: 50
        height: 50
        color: "#666abc"
        anchors.centerIn: parent
        rotation: 30 // 旋转30度
    }
}
```

## 4. 放缩 (scale)

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("02: Hello World")

    // 放缩
    Rectangle {
        width: 50
        height: 50
        color: "#666abc"
        anchors.centerIn: parent
        scale: 2     // 放缩 2 倍
    }
}
```

## 5. Rectangle
### 5.1 抗锯齿 (antialiasing)

> qt中, antialiasing 默认是 **关闭** 的, 一般只有对对象进行旋转、缩放、倾斜等 `非整数` 像素绘制操作时, 才会有明显的锯齿, 平常是看不出来的.
>
> 并不是所有元素都有 antialiasing 属性, 只有使用 Canvas 或 Shape、Path、PaintedItem 这些绘制类的元素才有这个选项.

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("02: Hello World")

    // 旋转 + 抗锯齿
    Rectangle {
        width: 50
        height: 50
        color: "#666abc"
        anchors.centerIn: parent
        rotation: 45 // 旋转
        antialiasing: true // 抗锯齿开
    }
}
```

### 5.2 边框宽度 (border.width)

> [!NOTE]
> 这个宽度是向内的拓展的, 不会演变为 `外边距`. (其更像是 `内边距`)

```qml
Rectangle {
    width: 50
    height: 50
    color: "#666abc"
    border.width: 10 // 边框宽度
}
```

### 5.3 圆角 (radius)

```qml
Rectangle {
    width: 50
    height: 50
    color: "#666abc"
    anchors.centerIn: parent
    radius: 10 // 圆角
}
```

### 5.4 渐变 (gradient)

```qml
Rectangle {
    width: 50
    height: 50
    gradient: Gradient {
        GradientStop { position: 0.0; color: "lights teelblue" }
        GradientStop { position: 1.0; color: "blue" }
    }
}
```

### 5.5 实现一个仅有上下边框的矩形

- 在 `ui/MyRectangle.qml` 中写(注意, 组件名应该为大写):

```qml
import QtQuick

Rectangle {
    id: borderRect
    property int _topMargin: 10
    property int _bottomMargin: 10

    color: "#000000"
    Rectangle {
        id: innerRect
        color: "#990099"
        anchors.fill: parent
        anchors.topMargin: borderRect._topMargin
        anchors.bottomMargin: borderRect._bottomMargin
        anchors.leftMargin: 0
        anchors.rightMargin: 0
    }
}
```

表示一个矩形套另一个矩形, 其中内部的矩形充当矩形主体, 外部的矩形作为 `边框`.

使用:

```qml
import QtQuick
import "ui" // 导入 (使用 import "路径")
            // 或者配置: 注册为 QML 模块
            // 这个太复杂了, 暂时先忽略
Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("02: Hello World")

    MyRectangle {
        width: 50
        height: 50
        x: 400
        y: 400
    }
}
```

> [!TIP]
> 此处也展示了一种, 自定义qml控件的方法!