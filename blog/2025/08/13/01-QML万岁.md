---
authors: Heng_Xin
title: QML 支持 Wayland 比 QWidget 好!
date: 2025-08-13 21:34:56
tags:
    - C++
---

## 本人正式加入 QML 圣教!!!

最近重拾之前写的音乐播放器项目. 因为一开始是使用 QWidget 开发的. 可是我尊贵的 Arch Linux 使用的是 KDE (Wayland) 桌面系统...

<!-- truncate -->

结果就是 窗口的 move 不行, 顶置不行, 可穿透窗口 / 透明窗口 / 无边窗口, 都难搞...

网上全是 CSDN 的赤💩教程, 根本就不行. 外网也只看到了外国老哥在无奈...

GPT / DS 直接上大炮, 写得显然不跨平台还超复杂...

QWidget 无边窗口被放弃了...

---

> [!TIP]
> ## 本人正式加入 QML 圣教!!!
>
> 要是 QML 改叫 **QTML** 就更好了 (类似于 js -> ts, 此次的 T 是 `type`)

最近看番, 歌好听... Linux 没有酷狗... 不习惯其他音乐程序... 正好可以重构我的 ASS Music 播放器~

并且学了 QML 也要找个东西练手.

谁知道, 让我发现了!! QML 居然默认有跨平台的 移动、缩放, 这让无边窗口非常简单. 并且鼠标穿透窗口也非常简单!

### 1. 通用无边窗口

```qml [c01-通用无边窗口]
// https://github.com/HengXin666/HX-Music/blob/main/resources/qml/window/BorderlessWindow.qml
pragma ComponentBehavior: Bound
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window

Window {
    id: root
    flags: Qt.FramelessWindowHint
    color: "transparent"

    property int bw: 3                  // 边框厚度
    property bool allowInnerMove: true  // 中间区域是否允许拖动
    property bool showBorder: true      // 是否显示边框 (不影响resize交互)
    property color borderColor: "red"   // 边框颜色
    property Component delegate: Item { // 正文内容
        Text {
            text: "请务必实现 delegate 自绘内部内容"
        }
    }
    property Component titleBar: Rectangle { // 自绘标题栏, 可为null, 内部需要自定义 height
        height: 30
        color: "#3f3f3f"
        MouseArea {
            anchors.fill: parent
            onDoubleClicked: root.toggleMaximized()
            acceptedButtons: Qt.LeftButton
            // 不要拖动, 移动逻辑交给外面的 DragHandler
        }
        RowLayout {
            anchors.fill: parent
            Label {
                text: root.title
                elide: Label.ElideRight
                horizontalAlignment: Qt.AlignHCenter
                verticalAlignment: Qt.AlignVCenter
                Layout.fillWidth: true
            }
            // 最小化按钮
            Item {
                Layout.preferredWidth: 30
                Layout.preferredHeight: 30
                Image {
                    anchors.centerIn: parent
                    width: 16
                    height: 16
                    source: "qrc:/icons/dropdown.svg"
                }
                MouseArea {
                    anchors.fill: parent
                    onClicked: root.showMinimized()
                }
            }

            // 最大化/还原按钮
            Item {
                Layout.preferredWidth: 30
                Layout.preferredHeight: 30
                Image {
                    anchors.centerIn: parent
                    width: 16
                    height: 16
                    source: root.visibility === Window.Maximized
                            ? "qrc:/icons/restore.svg"   // 还原图标
                            : "qrc:/icons/up.svg"  // 最大化图标
                }
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        if (root.visibility === Window.Maximized)
                            root.showNormal();
                        else
                            root.showMaximized();
                    }
                }
            }

            // 关闭按钮
            Item {
                Layout.preferredWidth: 30
                Layout.preferredHeight: 30
                Image {
                    anchors.centerIn: parent
                    width: 16
                    height: 16
                    source: "qrc:/icons/close.svg"
                }
                MouseArea {
                    anchors.fill: parent
                    onClicked: root.close()
                }
            }
        }
    }

    // 暴露的引用
    property alias delegateRef: mainUI.item
    property alias titleBarRef: titleBarLoader.item

    function toggleMaximized() {
        if ((root.visibility & Window.Maximized) === Window.Maximized) {
            root.showNormal();
        } else {
            root.showMaximized();
        }
    }

    // 内部状态数据
    QtObject {
        id: self
        property int bw: 0 // 边框厚度 [const]

        Component.onCompleted: {
            if (root.showBorder) {
                // 如果显示边框, 那么全屏时候也需要显示边框, 因此不应该屏蔽边框
                return;
            }
            bw = root.bw;
        }
    }

    // 全屏时候是否需要边距
    onVisibilityChanged: (val) => {
        if (self.bw === 0) {
            return;
        }
        root.bw = (val & Window.Maximized) === Window.Maximized 
            ? 0 
            : self.bw;
    }

    // 鼠标区域仅用于设置正确的光标形状
    MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: {
            const mousePoint = Qt.point(mouseX, mouseY);
            const b = root.bw + 10; // 角尺寸
            if (mousePoint.x < b && mousePoint.y < root.bw)              // 左上
                return Qt.SizeFDiagCursor;
            if (mousePoint.x >= width - b && mousePoint.y >= height - b) // 右下
                return Qt.SizeFDiagCursor;
            if (mousePoint.x >= width - b && mousePoint.y < root.bw)     // 右上
                return Qt.SizeBDiagCursor;
            if (mousePoint.x < b && mousePoint.y >= height - b)          // 左下
                return Qt.SizeBDiagCursor;
            if (mousePoint.x < b || mousePoint.x >= width - b)           // 水平
                return Qt.SizeHorCursor;
            if (mousePoint.y < root.bw || mousePoint.y >= height - b)    // 竖直
                return Qt.SizeVerCursor;
        }
        acceptedButtons: Qt.NoButton // 不处理实际事件
    }

    // 缩放
    DragHandler {
        id: resizeHandler
        target: null
        // 覆盖整个窗口
        onActiveChanged: if (active) {
            const p = resizeHandler.centroid.position;
            const b = root.bw + 10; // 角尺寸
            let e = 0;
            const leftEdge = p.x < b;
            const rightEdge = p.x >= root.width - b;
            const topEdge = p.y < root.bw;
            const bottomEdge = p.y >= root.height - b;

            if (leftEdge) {
                e |= Qt.LeftEdge;
            }
            if (rightEdge) {
                e |= Qt.RightEdge;
            }
            if (topEdge) {
                e |= Qt.TopEdge;
            }
            if (bottomEdge) {
                e |= Qt.BottomEdge;
            }

            if (e !== 0) {
                root.startSystemResize(e);
            } else if (root.allowInnerMove) {
                root.startSystemMove();
            }
        }
    }

    // 绘制边框 (仅视觉)
    Rectangle {
        anchors.fill: parent
        color: "transparent"
        border.color: root.showBorder ? root.borderColor : "transparent"
        border.width: root.showBorder ? root.bw : 0
    }

    // 标题栏 (如果有的话)
    Loader {
        id: titleBarLoader
        width: parent.width - 2 * root.bw
        x: root.bw
        y: root.bw
        sourceComponent: root.titleBar
    }

    // 内容区
    Loader {
        id: mainUI
        x: root.bw
        y: titleBarLoader.item ? titleBarLoader.y + titleBarLoader.height : root.bw
        width: root.width - 2 * root.bw
        height: root.height - (titleBarLoader.item ? titleBarLoader.y + titleBarLoader.height + root.bw : 2 * root.bw)
        sourceComponent: root.delegate
    }
}
```

简直简单到爆炸!

- `root.startSystemMove();` 就可以拖动

- `root.startSystemResize(e);` 就可以调整大小 (听说 macOS 不支持(?))

他们都是跨平台的! 不论 win / linux, x11 / Wayland 都可以.

直接爽飞了! 个人正式加入 QML 圣教! QWidget 你算老几啊!

### 2. 鼠标穿透

```qml [c2-鼠标穿透(整个窗口)]
Window {
    property bool locked: false // 是否开启整个窗口的鼠标穿透

    flags: Qt.FramelessWindowHint
         | Qt.WindowStaysOnTopHint
         | Qt.Tool
         | (locked ? Qt.WindowDoesNotAcceptFocus | Qt.WindowTransparentForInput : Qt.NoItemFlags)
}
```

```qml [c2-鼠标穿透(部分窗口)-QML]
// QML 部分
Window {
    property bool locked: false // 是否开启部分窗口的鼠标穿透

    flags: Qt.FramelessWindowHint
         | Qt.WindowStaysOnTopHint

    function lock() {
        locked = true;
        Qt.callLater(() => {
            // 计算不需要穿透的位置
            const posInRoot = delegateRef.lockButtonRef.mapToItem(delegateRef.delegateRoot, 0, 0);
            WindowMaskUtil.addControlRect(
                posInRoot.x, posInRoot.y,
                delegateRef.lockButtonRef.width,
                delegateRef.lockButtonRef.height
            );
            WindowMaskUtil.setMask(root);
        });
    }
    function unlock() {
        locked = false;
        Qt.callLater(() => {
            WindowMaskUtil.clear(root);
        });
    }
}
```

```cpp [c2-鼠标穿透(部分窗口)-C++]
// C++ 侧
#include <QObject>
#include <QWindow>
#include <QPolygon>
#include <QRegion>

#include <QDebug>

namespace HX {

class WindowMaskUtil : public QObject {
    Q_OBJECT
public:
    explicit WindowMaskUtil(QObject* parent = nullptr)
        : QObject(parent)
    {}

    Q_INVOKABLE void clear(QWindow* window) {
        _regions = {};
        if (window) {
            window->setMask(_regions);
        }
    }

    Q_INVOKABLE void addControlRect(int x, int y, int width, int height) {
        _regions = QRegion(
            QRect(x, y, width, height)
        );
    }

    Q_INVOKABLE void setMask(QWindow* window) {
        if (!window || _regions.isEmpty()) {
            return;
        }
        // 设置窗口掩码, 支持不规则的
        window->setMask(_regions);
    }
private:
    QRegion _regions;
};

} // namespace HX
```

不过我期望一个鼠标可以穿透, 但是仍然可以接受到鼠标事件的api... 似乎不行, 很难搞. (Wayland)