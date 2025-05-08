# Repeater

用法一: 通过索引

```qml
import QtQuick
import QtQuick.Controls

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("11: Hello World")

    Repeater {
        model: 3
        delegate: Button {
            width: 50; height: 50;
            y: index * 60
        }
    }
}
```

用法二:

```qml
Repeater {
    model: ["文本1", "文本2", "我是文本三"]
    delegate: Button {
        width: 50; height: 50;
        x: 100
        y: index * 60
        text: modelData // 通过 modelData 范问到 arr[index]
    }
}
```

类似于 vue 的 v-for