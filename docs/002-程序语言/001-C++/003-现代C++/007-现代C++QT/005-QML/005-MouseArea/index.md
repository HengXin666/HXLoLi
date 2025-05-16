# MouseArea

官方文档: [MouseArea QML Type](https://doc.qt.io/qt-6/zh/qml-qtquick-mousearea.html)

MouseArea 继承自 Item, 其是一种不可见的项目, 通常与可见项目结合使用. 以便为该项目提供鼠标处理功能.

## 1. 按键槽
### 1.1 onPressed

鼠标按下, 会触发这个 `onPressed` 槽.

### 1.2 onReleased

鼠标抬起, 会触发这个 `onReleased` 槽.

### 1.3 onClicked

onClicked = onPressed + onReleased

当鼠标按下并抬起, 才会触发 `onClicked` 槽. (注意, 需要抬起才会真的触发)

> [!TIP]
> 那你可能感觉它等价于 `onReleased`, 实则不然:
>
> | 槽函数 | 触发条件 |
> | :-: | :-: |
> | `onPressed` | 鼠标 **按下时立即触发** |
> | `onReleased` | 鼠标在此区域按下后, 无论抬起时是否还在区域内, 都会触发 |
> | `onClicked` | **按下 + 释放都在同一个 MouseArea 内**, 才会触发 |

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("05: Hello World")

    MouseArea {
        id: mouseArea
        width: 200
        height: 200
        Rectangle { // 需要套一个 Item 才可以显示
            anchors.fill: parent
            color: "#990099"
        }

        onClicked: {
            console.log("onClicked");
        }
        onPressed: {
            console.log("onPressed");
        }
        onReleased: {
            console.log("onReleased");
        }
    }
}
```

### 1.4 onDoubleClicked (双击)

```qml
MouseArea {
    onDoubleClicked: console.log("双击")
}
```

## 2. 设置监听的按键事件 (acceptedButtons)

你可能发现, 上面的代码, 仅有鼠标左键有效, 而右键、中间都没有什么效果.

这时候就需要设置 `acceptedButtons` 属性 (默认是 `Qt.LeftButton`).

```qml
MouseArea {
    // 这是一个枚举; 设置监听左键和右键
    acceptedButtons: Qt.LeftButton | Qt.RightButton
}
```

但是这样光触发了, 但是我们不知道用户到底是按了左键还是右键, 这怎么办呢?


## 3. 区分按键的信号 (pressedButtons & 枚举)

`pressedButtons` 该属性显示当前按下的鼠标按钮.

它包含以下按位组合
- Qt.LeftButton
- Qt.RightButton
- Qt.MiddleButton

> [!TIP]
> 特别的, 只有 `onPressed` 可以接收到按键, 在 `onReleased` 与 `onClicked` 时候, 按键已经弹起, `pressedButtons` 已经被清空.

```qml
MouseArea {
    id: mouseArea
    width: 200
    height: 200
    Rectangle {
        anchors.fill: parent
        color: "#990099"
    }

    acceptedButtons: Qt.LeftButton | Qt.RightButton
    onClicked: {
        console.log("onClicked -> ", pressedButtons);
    }
    onPressed: {
        console.log("onPressed -> ", pressedButtons);
        if (pressedButtons & Qt.LeftButton) {
            console.log("onPressed 左键");
        } else if (pressedButtons & Qt.RightButton) {
            console.log("onPressed 右键");
        }
    }
    onReleased: {
        console.log("onReleased -> ", pressedButtons);
    }
}
```

## 4. 判断鼠标是否处于 MouseArea 内
### 4.1 containsMouse

该属性表示鼠标 **当前** 是否位于 `MouseArea` 内.

> [!WARNING]
> 特别的,  如果 `hoverEnabled` 是 `false`, 那么 **只有** 在 `MouseArea` 内按下鼠标, 才会为 `true`.
>
> 但如果在 `onPressed` 处理程序中设置 `mouse.accepted = false`, `containsMouse` 将保持 `false`, 因为按下被拒绝.

> 如果改变了, 即是否处于悬浮状态, 那么也会触发 `onHoveredChanged`.

### 4.2 containsPress

等价于 `pressed && containsMouse`, 即 在 `MouseArea` 内按下鼠标, 才会为 `true`.

```qml
MouseArea {
    width: 200
    height: 200
    Rectangle {
        anchors.fill: parent
        color: "#990099"
    }

    hoverEnabled: true // 该属性用于确定是否处理悬停事件 (默认是 false)
    acceptedButtons: Qt.LeftButton | Qt.RightButton
    onClicked: {
        console.log("onClicked");
    }
    onContainsMouseChanged: {
        console.log("onContainsMouseChanged", containsMouse);
    }
    onContainsPressChanged: {
        console.log("onContainsPressChanged", containsPress);
    }
}
```

## 5. 改变光标样式 (cursorShape)

```qml
MouseArea {
    width: 200
    height: 200
    // 十字形光标
    cursorShape: Qt.CrossCursor
}
```

## 6. 拖动 (drag)

drag 提供了一种使项目可拖动的便捷方法:

- `drag.target` 指定要拖动的项目的 id
- `drag.active` 指定目标项目当前是否正在被拖动 *(该属性为系统设置, 为只读属性, 不能被用户赋值)*
- `drag.axis` 指定是否可以水平(`Drag.XAxis`)、垂直(`Drag.YAxis`)、任意(`Drag.XAndYAxis`)方向拖动
- `drag.minimum` 和 `drag.maximum` 限制目标项可以沿相应轴拖动的距离

下面示例展示一个可以 x 方向拖动的 Rectangle, 并且透明度会随着拖动下降.

```qml
Window {
    id: container; /* ... */

    Rectangle {
        id: rect
        width: 50; height: 50
        color: "red"
        opacity: (container.width - rect.x) / container.width

        MouseArea {
            anchors.fill: parent
            drag.target: rect
            drag.axis: Drag.XAxis
            drag.minimumX: 0
            drag.maximumX: container.width - rect.width
        }
    }
}
```

### 6.1 drag.filterChildren (属性继承)

如果设置 `drag.filterChildren: true`, 那么子控件也可以被拖动, 而不会阻止拖动信号.

```qml
Rectangle {
    width: 480
    height: 320

    Rectangle {
        x: 30; y: 30
        width: 300; height: 240
        color: "lightsteelblue"

        MouseArea {
            anchors.fill: parent
            drag.target: parent;
            drag.axis: Drag.XAxis
            drag.minimumX: 30
            drag.maximumX: 150
            drag.filterChildren: true   // 子控件是否继承父控件的属性
                                        // 默认是 false
            Rectangle {
                color: "yellow"
                x: 50; y : 50
                width: 100; height: 100
                MouseArea {
                    anchors.fill: parent
                    onClicked: console.log("Clicked")
                }
            }
        }
    }
}
```

## 7. 鼠标位置

`mouseX` 与 `mouseY` 是当前鼠标位置, `onMouseXChanged` 与 `onMouseYChanged` 会在 `鼠标在 MouseArea 内 && x/y 坐标改变` 时候触发. (需要 `hoverEnabled: true`, 否则只在按键事件才触发)

```qml
MouseArea {
    width: 100
    height: 100
    Rectangle {
        anchors.fill: parent
        color: "#990099"
    }
    hoverEnabled: true // 该属性用于确定是否处理悬停事件 (默认是 false)

    onMouseXChanged: {
        console.log("x:", mouseX);
    }
    onMouseYChanged: {
        console.log("y:", mouseY);
    }
}
```

## 8. 长按鼠标 (onPressAndHold)

没什么好说的, 长按鼠标 `800ms` 就会触发.

```qml
MouseArea {
    width: 50
    height: 50
    Rectangle {
        anchors.fill: parent
        color: "#990099"
    }

    onPressAndHold: {
        console.log("长按鼠标 (800ms)");
    }
}
```

## 9. 信号传递 (propagateComposedEvents)

该属性表示组成的鼠标事件是否会自动传播到与该 `MouseArea` 重叠但在视觉堆叠顺序中较低的其他 `MouseAreas`. 默认情况下, 该属性为 `false`.

特别的, **QML 的鼠标事件默认是只会被第一个接受的元素处理, 且不会向上传递的**.

故如果需要事件传递, 需要 `propagateComposedEvents: true` && `mouse.accepted = false`

```qml
Rectangle {
    color: "yellow"
    width: 100; height: 100

    MouseArea {
        anchors.fill: parent
        onClicked: console.log("clicked yellow")
    }

    Rectangle {
        color: "blue"
        width: 50; height: 50

        MouseArea {
            anchors.fill: parent
            propagateComposedEvents: true
            onClicked: (mouse) => {
                console.log("clicked blue")
                mouse.accepted = false // 这个不能少
            }
        }
    }
}
```