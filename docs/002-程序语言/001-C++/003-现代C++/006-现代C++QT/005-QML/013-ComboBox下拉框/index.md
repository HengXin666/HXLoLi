# ComboBox 下拉框

## 1. 基础
### 1.1 基础代码

```qml
import QtQuick
import QtQuick.Controls

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("13: Hello World")

    ComboBox {
        model: ["选项1", "选项2", "选项3"]
        Component.onCompleted: {
            console.log("cnt:", count); // 项的个数
        }
    }
}
```

### 1.2 支持用户输入

```qml
ComboBox {
    editable: true // 支持用户输入
    model: ListModel {
        id: model
        ListElement {
            text: "选项1"
        }
        ListElement {
            text: "选项2"
        }
        ListElement {
            text: "选项3"
        }
    }
    Component.onCompleted: {
        console.log("cnt:", count);
    }
    onAccepted: { // 选中后, 如果不存在, 那么添加进List
        if (find(editText) === -1) {
            model.append({text: editText});
        }
    }
}
```

### 1.3 自定义绘制文本

![自定义绘制文本 ##w500##r10##](PixPin_2025-05-09_15-56-56.png)

```qml
ComboBox {
    textRole: "hxText" // 作为文本的变量名称 (ListElement 中)
    valueRole: "hxVal" // 作为值的变量名称
    // 自定义显示文本
    displayText: `${currentText} -> ${currentValue}`
    model: ListModel {
        id: hxListModel
        ListElement {
            hxText: "文本1"
            hxVal: 123
        }
        ListElement {
            hxText: "文本2"
            hxVal: 456
        }
        ListElement {
            hxText: "文本3"
            hxVal: 789
        }
    }
    onCurrentTextChanged: { // 当前选择的项对应的文本
        console.log("txt:", currentText);
    }
    onCurrentValueChanged: { // 当前选择的项对应的值
        console.log("val:", currentValue);
    }
}
```

### 1.4 正则表达式限制输入内容

```qml
ComboBox {
    editable: true
    // validator: IntValidator { // 匹配数字, 不怎么好用
    //     top: 100
    //     bottom: 0
    // }
    validator: RegularExpressionValidator { // 仅可输入正则表达式: (16进制数)
        regularExpression: /0x[0-9A-F]+/
    }
    onAcceptableInputChanged: { // 匹配状态改变时候触发
        console.log("匹配状态:", acceptableInput);
    }
}
```

## 2. 自定义绘制
