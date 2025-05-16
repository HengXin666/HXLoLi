# states与transitions-动画效果制作

## 1. 状态 (states)
### 1.1 基本写法

如下代码, 使用 `states` 预先定义多个状态(`State`)

然后使用 `state` 使用一个状态, 作为初始状态.

定义多个状态的好处是, 可以把不同状态下各个属性值给封装为「状态」, 而 **不是** 到处`xxx.属性1 = orz; xxx.属性2 = orz`.

```qml [z1-动态绑定写法]
import QtQuick

// 注: Qt 官方从 Qt 6.5 起加强的静态分析器规则, 下面代码会有 qmllint 警告
Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("03: Hello World")

    Rectangle {
        id: root
        anchors.centerIn: parent
        width: 100
        height: 100

        states: [
            State {
                name: "red_color"
                PropertyChanges { target: root; color: "red"; width: 200; }
            },
            State {
                name: "blue_color"
                PropertyChanges { target: root; color: "blue"; height: 200; }
            }
        ]
        state: "red_color"

        MouseArea {
            anchors.fill: parent
            onPressed: {
                root.state = "blue_color"
            }
            onReleased: {
                root.state = "red_color"
            }
        }
    }
}
```

```qml [z1-静态绑定写法(性能好)]
import QtQuick

// 备注: 这种写法是 `静态键路径绑定`(statically-parsed bindings), 可以提升性能
Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("03: Hello World")

    Rectangle {
        id: root
        anchors.centerIn: parent
        width: 100
        height: 100

        states: [
            State {
                name: "red_color"
                PropertyChanges { root.color: "red"; root.width: 200; }
            },
            State {
                name: "blue_color"
                PropertyChanges { root.color: "blue"; root.height: 200; }
            }
        ]
        state: "red_color"

        MouseArea {
            anchors.fill: parent
            onPressed: {
                root.state = "blue_color"
            }
            onReleased: {
                root.state = "red_color"
            }
        }
    }
}
```

## 2. 动画
### 2.1 PropertyAnimation & NumberAnimation

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("03: Hello World")

    // 动画示例
    Rectangle {
        id: flashingblob
        width: 75
        height: 75
        color: "blue"
        opacity: 1.0

        MouseArea {
            anchors.fill: parent
            onClicked: {
                animateColor.start();
                animateOpacity.start();
                animateWidth.start();
            }
        }

        // 定义一个属性动画
        PropertyAnimation {
            id: animateColor        // 动画名称(id)
            target: flashingblob    // 作用对象(id)
            properties: "color"     // 需要改变的属性名称
            to: "green"             // 从当前值改为的目标值
            duration: 1000          // 持续时间: ms
        }
        // 定义一个数值动画
        NumberAnimation {
            id: animateOpacity      // 动画名称(id)
            target: flashingblob    // 作用对象(id)
            properties: "opacity"   // 需要改变的属性名称
            from: 0.1               // 初始值
            to: 1.0                 // 目标值
            duration: 2000          // 持续时间: ms
        }
        // 可定义多个, 通过 id 调用
        NumberAnimation {
            id: animateWidth
            target: flashingblob
            properties: "width"
            from: 100
            to: 175
            duration: 1000
        }
    }
}
```

### 2.2 立即生效的动画

> 上面的动画, 是我们需要一个动作触发, 才会生效的.
>
> 而有时候, 我们可能会希望其默认就触发一些动画, 比如在刚刚启动页面时候, 如果有一种控件缓缓从四周出现的感觉, 以作为一种快速加载动画, 有时候会比较好看和酷炫.
>
> 想实现这种效果, 这时候, 只需要在之前的 `PropertyAnimation & NumberAnimation` 加上 `on 属性名` 即可

> [!TIP]
> 注意, 如果值被 `on 属性1` 了, 就 **不能** 再写 `属性1: xxx` 了

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("03: Hello World")

    // 立刻生效的动画
    Rectangle {
        width: 75
        height: 75
        color: "#990099"

        PropertyAnimation on x {
            from: 75
            to: 175
            duration: 1000
        }
        PropertyAnimation on y {
            from: 75
            to: 175
            duration: 1000
        }
        NumberAnimation on opacity {
            from: 0.1
            to: 1.0
            duration: 2000
        }
    }
}
```

### 2.3 ColorAnimation (颜色动画)

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("03: Hello World")

    // 颜色动画
    Rectangle {
        width: 75
        height: 75
        x: 300
        y: 300
        ColorAnimation on color {
            from: "#990099"
            to: "#2b2b2b"
            duration: 1000
        }
    }
}
```

### 2.4 SequentialAnimation (动画队列)

`SequentialAnimation` 是动画队列, 会顺序播放动画:

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("03: Hello World")

    // 动画队列
    Rectangle {
        width: 75
        height: 75
        x: 500
        SequentialAnimation on color {
            ColorAnimation { to: "yellow"; duration: 1000 }
            ColorAnimation { to: "#990099"; duration: 1000 }
            ColorAnimation { to: "blue"; duration: 1000 }
        }
        SequentialAnimation on y {
            NumberAnimation { from: 100; to: 300; duration: 2000 }
            NumberAnimation { to: 100; duration: 2000 }
        }
    }
}
```

### 2.5 动画过渡 (transitions)

只需要在 `states` 的基础上, 编写 `transitions`, 即可自动生成过渡动画.

相当于 `states` 定义属性数值, `transitions` 设定动画时长.

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("03: Hello World")

    // 动画过渡
    Rectangle {
        id: rect2
        width: 75
        height: 75
        x: 600
        state: "state_01"

        states: [
            State {
                name: "state_01"
                PropertyChanges {
                    rect2.color: "#990099"
                }
            },
            State {
                name: "state_02"
                PropertyChanges {
                    rect2.color: "#2b2b2b"
                }
            }
        ]

        transitions: [
            Transition {
                from: "state_01"
                to: "state_02"
                ColorAnimation {
                    target: rect2
                    duration: 1000
                }
            },
            Transition {
                from: "state_02"
                to: "state_01"
                ColorAnimation {
                    target: rect2
                    duration: 1000
                }
            }
        ]

        MouseArea {
            anchors.fill: parent
            onReleased: rect2.state = "state_01"
            onPressed: rect2.state = "state_02"
        }
    }
}
```

- 小技巧: 其实可以只写一个 `Transition` 不指定 `from/to`, 比如:

> 唯一的坏处是, 这样所有动画的持续时间都是一样的

```qml
transitions: [
    Transition {
        ColorAnimation {
            target: rect2
            duration: 1000
        }
    }
]
```

> [!TIP]
> 你可以在一个 Transition 中放入多个动画, 例如颜色 + 位置一起动画:
> 
> ```qml
> transitions: Transition {
>     ColorAnimation { target: rect2; duration: 1000 }
>     NumberAnimation { target: rect2; property: "x"; duration: 1000 }
> }
> ```

### 2.6 值改变而触发的动画 (Behavior)

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("03: Hello World")

    // 值改变而触发的动画
    Rectangle {
        id: rect3
        width: 75
        height: 75
        x: 600
        y: 200
        color: "#990099"
        MouseArea {
            anchors.fill: parent
            onPressed: {
                rect3.x -= 30;
                rect3.y += 20;
            }
        }

        Behavior on y {
            NumberAnimation {
                id: bouncebehavior
                easing {
                    type: Easing.OutElastic
                    amplitude: 1.0
                    period: 0.5
                }
            }
        }
    }
}
```

### 2.7 依次的动画

> 实际上就是 `SequentialAnimation`, 只是让它不是都作用在同一个元素.

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("03: Hello World")

    // 依次的动画
    Rectangle {
        id: rect4
        width: 150
        height: 100
        x: 600
        y: 300
        color: "#8b8b8b"

        Column { // <-- 此处还没有学到, 可以忽略, 之后会讲
            anchors.centerIn: parent
            Text {
                id: text_01
                text: qsTr("这个是文本1")
                opacity: 0
            }
            Text {
                id: text_02
                text: qsTr("这个是文本2")
                opacity: 0
            }
            Text {
                id: text_03
                text: qsTr("这个是文本3")
                opacity: 0
            }
        }

        MouseArea {
            anchors.fill: parent
            onPressed: playBanner.start()
        }

        SequentialAnimation {
            id: playBanner
            running: false // 是否自动触发
            NumberAnimation { target: text_01; property: "opacity"; from: 0; to: 1; duration: 1000 }
            NumberAnimation { target: text_02; property: "opacity"; from: 0; to: 1; duration: 1000 }
            NumberAnimation { target: text_03; property: "opacity"; from: 0; to: 1; duration: 1000 }
        }
    }
}
```