# focus相关与FocusScope

本节内容主要就是说明: `focus` 与 `activeFocus` 的不同.

一个窗口中, 可以有很多元素 `focus: true` (有焦点);

但是谁真正使用了这个焦点, 就应该看是谁 `activeFocus` 为 `true`.

## 1. 虚假的焦点 (focus)

```qml
import QtQuick
import Qt5Compat.GraphicalEffects
import QtQuick.Controls

pragma ComponentBehavior: Bound

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("14: Hello World")

    // 明确创建焦点范围
    FocusScope {
        width: 100; height: 50;
        focus: true
        Button {
            id: btn1
            anchors.fill: parent
            focus: true
            focusPolicy: Qt.NoFocus
            background: Rectangle {
                anchors.fill: parent
                border.color: btn1.focus ? "#990099" : "black"
            }
        }
    }

    FocusScope {
        width: 100; height: 50;
        y: 200
        focus: true
        Button {
            id: btn2
            anchors.fill: parent
            focus: true
            focusPolicy: Qt.NoFocus // 设置如何获取焦点: 默认是 Qt.NoFocus (无法获取)
                                    // 常用有: Qt.StrongFocus (鼠标和Tab可获取) 等
            background: Rectangle {
                anchors.fill: parent
                border.color: btn2.focus ? "#990099" : "black"
            }
        }
    }
}
```

运行发现, 两个控件都是 `#990099` 色的边框 => 说明他们的 `focus: ture`, 这是怎么回事?

怎么会都获取到焦点呢?

## 2. 真正的焦点 (activeFocus)

`activeFocus` 属性才显示真正是誰在操作这个控件

```qml
FocusScope {
    width: 100; height: 50;
    focus: true
    Button {
        id: btn1
        anchors.fill: parent
        focus: true
        focusPolicy: Qt.NoFocus
        background: Rectangle {
            anchors.fill: parent
            border.color: btn1.activeFocus ? "#990099" : "black"
        }
        onClicked: {
            forceActiveFocus();
        }
    }
}

FocusScope {
    width: 100; height: 50;
    y: 200
    focus: true
    Button {
        id: btn2
        anchors.fill: parent
        focus: true
        focusPolicy: Qt.NoFocus
        background: Rectangle {
            anchors.fill: parent
            border.color: btn2.activeFocus ? "#990099" : "black"
        }
        onClicked: {
            forceActiveFocus(); // 设置当前控件有焦点
        }
    }
}
```