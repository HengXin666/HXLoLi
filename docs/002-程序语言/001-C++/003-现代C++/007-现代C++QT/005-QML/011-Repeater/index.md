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

    // 用法一
    Repeater {
        model: 3 // 数字, 表示元素个数
        delegate: Button { // 委托(item), 表示每一项是如何绘制的
            required property int index // 显式注入 (默认注入的索引值)
            width: 50; height: 50;
            y: index * 60
        }
    }

    // 用法二
    Repeater {
        model: ["文本1", "文本2", "我是文本三"] // list<T> 可以自定义数据
        delegate: Button {
            required property int index         // 默认注入的索引值
            required property string modelData  // 对应索引的元素 (也是默认注入的)
            width: 50; height: 50;
            x: 100
            y: index * 60
            text: modelData
        }
    }
}
```

类似于 vue 的 v-for