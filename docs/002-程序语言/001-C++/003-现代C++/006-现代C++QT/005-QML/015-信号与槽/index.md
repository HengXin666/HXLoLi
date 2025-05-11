# 信号与槽
## 1. qml中自定义信号与槽绑定

```qml
import QtQuick
import QtQuick.Controls

pragma ComponentBehavior: Bound

Window {
    id: root
    width: 640
    height: 480
    visible: true
    title: qsTr("15: Hello World")

    signal hxMySignal(string str, int sum); // 自定义信号

    function func(ss, ii) { // 函数
        console.log("[func]:", ss, ii);
    }

    // 自动生成的槽 (默认和`hxMySignal`信号连接了)
    onHxMySignal: {
        console.log("[HX]: siganl");
    }

    Button {
        text: "自定义槽"
        onClicked: {
            root.hxMySignal("abc", 123); // 发送信号
        }
    }

    // 绑定槽
    Component.onCompleted: {
        hxMySignal.connect(func); // 给信号绑定槽
    }
}
```

## 2. Connections 优雅绑定槽

> [!TIP]
> 如果项目结构复杂, 建议使用`Connections`来绑定信号, 这样结构更清晰, 而不是堆在 `Component.onCompleted` 或者 漫无目的的遍布 `onXxx`; 而你写 `Connections` 可以清楚的知道这个就是自定义槽, 并且还指明了 `target` (监听对象).

```qml
Window {
    id: root
    signal hxMySignal(string str, int sum); // 自定义信号

    Button {
        text: "自定义槽"
        onClicked: {
            root.hxMySignal("abc", 123); // 发送信号
        }
    }

    Connections {
        target: root
        function onHxMySignal(str, val) {
            console.log("[Connections]:", str, val);
        }
    }
}
```

## 3. 连接自定义组件的自定义信号

自定义组件:

```qml
import QtQuick
import QtQuick.Controls

Rectangle {
    id: root
    width: 100; height: 100;
    signal hxButtonClicked(string msg);
    Button {
        id: btn
        signal hxButtonNaKa(string msg); // 这个外面收不到, 你喊什么都妹用!
        text: "按钮1"
        onClicked: {
            root.hxButtonClicked("偷偷告诉你, 这个是自定义信号: 按钮1被人按了!");
            hxButtonNaKa("别听它乱说, 我才是按钮!");
        }
    }
}
```

外部使用:

```qml
Loader {
    id: loader
    sourceComponent: MySignalRect {
        Connections {
            target: loader.item
            ignoreUnknownSignals: true  // 连接到不存在的信号会产生运行时错误
                                        // 设置为 true 表示忽略这些错误警告
            function onHxButtonClicked(msg: string) {
                console.log("里面说:", msg);
                console.log("我回复道:", "徒弟我来救你啦");
            }
        }
        // Connections {
        //     target: loader.item.btn // 届かない
        //     function onHxButtonNaKa(msg: string) {
        //         console.log("里面说:", msg);
        //         console.log("我回复道:", "我徒弟呢!?");
        //     }
        // }
    }
}
```

就像是 `onClicked`, 你也可以把它当做是内部发送了一个 `click` 信号, 然后你自定义了一个叫 `onClicked` 的槽, 以捕获它.

这里的写法其核心实际上也是这样.

故此, 你也可以这样:

```qml
Loader {
    id: loader
    sourceComponent: MySignalRect {
        onHxButtonClicked: (msg) => {
            console.log("(真没办法) 里面说:", msg);
        }
    }
}
```