# Text的使用

- 官方文档: [Text QML Type](https://doc.qt.io/qt-6/zh/qml-qtquick-text.html)

## 1. 显示与宽高

Text可以直接使用 `\n` 表示换行.

对它设置 `width: 0; height: 0;` 是无用的, 因为它文本的 `w/h` 是自动计算的:

即 `contentWidth`/`contentHeight`

```qml
Text {
    id: txt
    width: 0; height: 0; // 无用
    text: qsTr("text    \n 换行 \n <h1>我是标题啊</h1> \n ## 副标题")

    Component.onCompleted: {
        console.log("width", contentWidth);
        console.log("height", contentHeight);
    }
}
```

## 2. 文本省略

示例:

```qml
Rectangle {
    x: 50; y: 50;
    width: 50; height: 50;
    border.color: "#000000"

    Text {
        id: txt
        elide: Text.ElideRight // 文本过长, 右边省略
        anchors.fill: parent   // 需要指定大小 (也就是 `w/h`)
        text: qsTr("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")

        Component.onCompleted: {
            console.log("width", contentWidth);
            console.log("height", contentHeight);
        }
    }
}
```

## 3. 字体样式

```qml
Text {
    text: qsTr("zzzzz\nLoli")

    Component.onCompleted: {
        console.log("行数:", lineCount);
        console.log("每一行字体间隔的高度", lineHeight)
    }

    font.family: "萝莉体"
    font.italic: true       // 斜体
    font.letterSpacing: 10  // 字母之间的横向距离
    font.pixelSize: 16      // 字体大小: px (像素)
    // font.pointSize: 16      // 字体大小: 磅
    font.underline: true    // 下划线
}
```

## 4. 文本格式 (textFormat)

|常量|说明|
|:-:|:-:|
|Text.AutoText|(默认) 通过 `Qt::mightBeRichText()` 启发式检测到|
|Text.PlainText|所有样式标记都被视为纯文本|
|Text.StyledText|HTML 3.2 中经过优化的基本富文本格式|
|Text.RichText|HTML 4 的子集|
|Text.MarkdownText|CommonMark加上用于表格和任务列表的GitHub扩展 (自 5.14 起)|

```qml
Column {
    Text {
        font.pointSize: 24
        text: "<b>Hello</b> <i>World!</i>"
    }
    Text {
        font.pointSize: 24
        textFormat: Text.RichText
        text: "<b>Hello</b> <i>World!</i>"
    }
    Text {
        font.pointSize: 24
        textFormat: Text.PlainText
        text: "<b>Hello</b> <i>World!</i>"
    }
    Text {
        font.pointSize: 24
        textFormat: Text.MarkdownText
        text: "**Hello** *World!*"
    }
}
```

## 5. 自动换行

```qml
Text {
    id: txt
    anchors.fill: parent
    text: qsTr("zzzzz\nLoli")
    wrapMode: Text.WordWrap // 单词换行, 它换行不会拆分单词
                            // 默认属性是不换行
}
```

## 6. 超链接支持

超链接需要富文本(htlm/Markdown), 才可以在 `Text` 中显示.

```qml
Text {
    font.pixelSize: 16
    textFormat: Text.MarkdownText
    text: "See the [Qt Project website](http://qt-project.org) or [HXLoLi](https://hengxin666.github.io/HXLoLi/docs/)"
    onLinkActivated: (link) => {
        console.log("点击了链接:", link);
    }
    // 进出链接会触发
    onLinkHovered: (link) => {
        console.log("选停在链接上:", link);
    }
    // 表示当前鼠标悬停处的链接 (hoveredLink), 改变时候, 就会触发 (实际上, 和上面的没有什么区别)
    // 大概率是因为这个是属性, 自动生成的 onXxxChanged 导致的, 可能会出现一些是一样的.
    // 虽然他们本质语义是不同的: 一个是`信号`, 一个是`属性变更通知`
    onHoveredLinkChanged: {
        console.log("onHoveredLinkChanged", hoveredLink);
    }

    // 如果希望悬浮在url上, 有光标样式改变, 可以像下面这样
    // 不过坏处是, 点击的话, 需要自定义编写, 因为鼠标信号被 MouseArea 捕获了, 不会透过...
    MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: parent.hoveredLink !== "" ? Qt.PointingHandCursor : Qt.ArrowCursor
        onClicked: {
            const link = parent.hoveredLink;
            if (link !== "") {
                console.log("[onClicked] 点击了链接:", link);
            }
        }
    }
}
```