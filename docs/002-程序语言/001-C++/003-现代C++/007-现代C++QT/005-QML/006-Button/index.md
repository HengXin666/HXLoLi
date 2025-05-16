# Button
## 1. 常用信号槽

实际上和 [MouseArea](../005-MouseArea/index.md) 的是一样的.

```qml
import QtQuick
import QtQuick.Controls

Window {
    /* ... */
    Button {
        width: 50; height: 50;
        onClicked: {}
        onPressed: {}
        onReleased: {}
        onPressAndHold: {}
        onDoubleClicked: {}
    }
}
```

## 2. 用不上的属性: flat & highlighted

> 很少能用上的属性

```qml
Button {
    width: 50; height: 50;
    flat: true // 使按钮为背景色 (只有点击的时候才会显现)
    highlighted: true // 始终显示为被摁压的样子 (高亮)
}
```

## 3. 切换型按钮
### 3.1 使用 checkable 属性实现

> [!TIP]
> 提醒, 6.2版本后 `checked` 属性的修改不再影响 `checkable` 属性.

总之, 如果你希望你的按钮像是开关一样, 点击是开、点击是关, 那么就可以设置:

```qml
Button {
    checkable: true
}
```

### 3.2 互斥的切换型按钮 (autoExclusive)

有时候, 我们可能希望, 对于多个 `切换型按钮`, 我们可以选中一个, 然后取消选择原本那个. 这可以使用 `autoExclusive` 属性实现.

> [!TIP]
> `autoExclusive: true` 不是针对整个界面, 而是在同一个“父容器中”启用互斥行为的. 也就是说, 它只在兄弟控件之间起作用.

下面这三个按钮, 就只有一个会处于被选中状态:

```qml
Button {
    width: 50; height: 50;
    autoExclusive: true
    checkable: true
}

Button {
    width: 50; height: 50;
    x: 60
    autoExclusive: true
    checkable: true
}

Button {
    width: 50; height: 50;
    x: 120
    autoExclusive: true
    checkable: true
}
```

## 4. 长按会多次触发

现在长按按钮, 会不断的触发 `onClicked`/`onPressed`/`onReleased` 事件

```qml
Button {
    // 长按多次触发
    autoRepeat: true
    autoRepeatDelay: 3000       // 控制长按到触发所需的时间 (ms)
    autoRepeatInterval: 1000    // 控制触发的时间间隔 (ms)

    onClicked: console.log("onClicked")
    onPressed: console.log("onPressed")
    onReleased: console.log("onReleased")
}
```

## 5. onDownChanged

> 对比: `onPressed` **只会** 在按下的时候触发, `onDownChanged` 你按下鼠标移开按钮区域也会触发.

```qml
Button {
    width: 50; height: 50;
    x: 60
    autoExclusive: true
    checkable: true

    // Pressed只会在按下的时候触发, Down你鼠标移开也会触发
    onDownChanged: console.log("down:", down, "pressed:", pressed)
    onPressed: console.log("!onPressed")
}
```

## 6. 不常用的外观
### 6.1 图标 (icon)

```qml
Button {
    width: 50; height: 50;

    // 这个好像不能加载 png/jpg
    // icon.source: "qrc:/img/misaka.png"
    // icon.color: "red"

    indicator: Image {
        anchors.fill: parent
        source: "qrc:/img/misaka.png"
    }
}
```

### 6.2 文本

```qml
Button {
    width: 50; height: 50;
    text: "文本"
}
```

## 7. 更自定义的外观 (background)

之前的 `6. 不常用的外观`, 它所自定义的内容有限, 所以我们一般不使用他们, 而是使用 `background`.

`background` 可以放置一个 `item`, 以使用 `item` 作为按钮的样式.

```qml
Button {
    id: btn3
    width: 50; height: 50;

    background: Rectangle {
        anchors.fill: btn3

        // 注: 此处应该使用状态机等, 这里仅供示例
        color: btn3.pressed ? "#990099" : "#2b2b2b"
        border.color: btn3.pressed ? "#2b2b2b" : "#990099"
    }
}
```

> [!TIP]
> 如果发现, 某些时候无法显示为自定义的 `background: Rectangle` 的话,
>
> 那么需要 切换到支持自定义的 非原生样式, 如 `import QtQuick.Controls.Basic`