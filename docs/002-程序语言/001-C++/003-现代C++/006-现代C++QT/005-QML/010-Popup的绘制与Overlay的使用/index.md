# Popup的绘制与Overlay的使用

- 官方文档: [Popup](https://doc.qt.io/qt-6/zh/qml-qtquick-controls-popup.html)

`Popup` 是弹出式用户界面控件.

## 1. 简单的使用

Popup 默认是隐藏的, 我们一般需要调用其 open() 方法让其显示.

默认是非模态的, 可以使用 `modal: true` 设置其为模态.

```qml
import QtQuick
import QtQuick.Controls

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("10: Hello World")

    Button {
        text: "Open"
        onClicked: {
            console.log("弹出窗口");
            popup.open(); // 显示窗口的方法: 使用 open() 方法
                          //                或者 手动设置其 visible 属性
        }
    }

    Popup { // 类似于 Rectangle
        id: popup
        x: 100
        y: 100
        width: 200
        height: 300
        modal: true // 模态: true (默认是非模态)
        focus: true // 焦点
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent

        Component.onCompleted: {
            console.log("可见:", visible); // 默认 visible = false
        }
    }
}
```

## 2. 一些特别的特性
### 2.1 父控件隐藏, 不关我事

在一般的情况下, 普通的控件的父控件如果是隐藏的, 那么其子控件也会被隐藏, 即便子控件设置了 `visible: true`.

```qml
Rectangle {
    width: 100; height: 100;
    color: "#990099"
    visible: false
    Rectangle {
        width: 50; height: 50;
        color: "#2b2b2b"
        visible: true
    }
}
```

但是, Popup 是个例外. 它 **不会** 受父控件隐藏而影响!

### 2.2 z-index 高人一等

特别的的, Popup 默认是处于所有控件之上的, 因此 普通控件的 z-index 不可能把 Popup 给覆盖.

但是 Popup 会因为 z-index 把另一个 Popup 给覆盖.

### 2.3 绘制范围限制

普通控件 受限于父控件的边界 (如 clip: true), 而 Popup 不受父控件边界限制, 可悬浮任意位置!

## 3. 关闭策略

```qml
Popup {
    // 默认值为 Popup.CloseOnEscape | Popup.CloseOnPressOutside
    // 即 点击窗口外 || 按ESC 都会关闭窗口
    closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
/*
                常量                                说明
    Popup.NoAutoClose                   弹出窗口只有在收到手动关闭指示时才会关闭
    Popup.CloseOnPressOutside           当鼠标在弹出窗口外按下时, 弹出窗口将关闭
    Popup.CloseOnPressOutsideParent     当鼠标被按到父级之外时, 弹出窗口将关闭
    Popup.CloseOnReleaseOutside         在弹出窗口外释放鼠标时, 弹出窗口将关闭
    Popup.CloseOnReleaseOutsideParent   当鼠标从父窗口外释放时, 弹出窗口将关闭
    Popup.CloseOnEscape                 在弹出窗口有活动焦点时按下 Escap 键, 弹出窗口将关闭
*/
}
```

> [!TIP]
> 注意: 有一个已知的限制, 即`Popup.CloseOnReleaseOutside`和`Popup.CloseOnReleaseOutsideParent`策略仅适用于`modal`弹出窗口

## 4. 调暗背景 (dim)

```qml
Popup {
    dim: true // 控制是否调暗背景
}
```

> 其颜色值在 `Overlay.modeless` 中定义.

## 5. 开关窗口的动画

```qml
Popup {
    // 窗口弹出时候的动画
    enter: Transition {
        NumberAnimation { property: "opacity"; from: 0.0; to: 1.0; duration: 1000 }
    }
    // 窗口关闭时候的动画
    exit: Transition {
        NumberAnimation { property: "opacity"; from: 1.0; to: 0.0; duration: 1000 }
    }
}
```

## 6. 自定义背景颜色与内部元素内容

```qml
Popup { // 类似于 Rectangle
    id: popup
    width: 200
    height: 300
    modal: true // 模态: true (默认是非模态)
    dim: true   // 控制是否调暗背景

    // 窗口弹出时候的动画
    enter: Transition {
        NumberAnimation { property: "opacity"; from: 0.0; to: 1.0; duration: 1000 }
    }
    // 窗口关闭时候的动画
    exit: Transition {
        NumberAnimation { property: "opacity"; from: 1.0; to: 0.0; duration: 1000 }
    }

    // 自定义模态背景色
    Overlay.modal: Rectangle {
        anchors.fill: parent
        color: "#64990099"
    }

    // 自定义内部内容
    contentItem: Rectangle {
        anchors.fill: parent
        Text {
            text: "Msg HX: 114514"
        }
        Button {
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 10
            text: "确定"
            onClicked: console.log("确定")
        }
    }
}
```

## 7. 如何实现模态时候可以点击外部

暂时不清楚这样做的应用场景在哪里...

```qml
// 此处仅阐述思路

Popup { // 主弹出窗口
    Popup { // 需要点击的内容
        x: ?; y: ?;
        width: ?; height: ?;
        // 设置其大小不和主窗口重叠, 但是却覆盖到可以点击的位置
        // 这样我们就可以点击到了
    }
}
/*
|------------------------------------------| (大窗口)
|                                          |
|  ================================== xxx  | == 这里是 子Popup
|  ================================== xxx  | == 子在主上, 故可点击, 并且不和`主`重叠,
|                                          |    故也不阻碍`主`.
|                                          |
|          =========================       | == 这里是 主Popup
|          =========================       |
|          =========================       |
|          =========================       |
|          =========================       |
*/
```