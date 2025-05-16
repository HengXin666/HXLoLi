# Window介绍
## 一、环境配置

> [!TIP]
> 我使用的是 VsCode 在 win 上写 qt

### 1.1 CMake

```cmake
set(projectName "HX-01-QML")

project(${projectName}
    VERSION 0.0.1
    DESCRIPTION ${projectName}
    HOMEPAGE_URL "https://github.com/HengXin666/HXTest"
    LANGUAGES CXX
)

# QT Start
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB_RECURSE src_files CONFIGURE_DEPENDS 
    src/*.cpp
    include/*.h
    include/*.hpp
)

file(GLOB_RECURSE qrc_files CONFIGURE_DEPENDS 
    resources/*.qrc
)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Qml Quick)

qt_add_executable(${projectName}
    ${src_files}
    ${qrc_files}
)

target_compile_features(${projectName} PUBLIC cxx_std_20)

target_link_libraries(${projectName}
    PRIVATE Qt::Core
    PRIVATE Qt::Gui
    PRIVATE Qt::Widgets
    PRIVATE Qt::Qml
    PRIVATE Qt::Quick
)

# 添加 QML 文件所在目录作为资源路径 (推荐方式)
qt_add_qml_module(${projectName}
    URI ${projectName}
    VERSION 1.0
    QML_FILES
        qml/Main.qml
)

if (WIN32)
    # 解决路径问题, 确保 windeployqt.exe 存在
    set(QT_BIN_DIR "${QT_COMPILER_PATH}/bin")
    if(NOT EXISTS "${QT_BIN_DIR}/windeployqt.exe")
        message(FATAL_ERROR "Error: windeployqt.exe not found in ${QT_BIN_DIR}")
    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Debug")
        add_custom_command(TARGET ${projectName} POST_BUILD
            COMMAND "${QT_BIN_DIR}/windeployqt.exe" --debug "$<TARGET_FILE:${projectName}>"
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/Debug"
        )
    elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
        file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Release")
        add_custom_command(TARGET ${projectName} POST_BUILD
            COMMAND "${QT_BIN_DIR}/windeployqt.exe" --release "$<TARGET_FILE:${projectName}>"
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/Release"
        )
    endif()
endif()

set_target_properties(${projectName} PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE OFF
    WIN32_EXECUTABLE OFF # 这里需要为 OFF 才可以让vscode在控制台中输出...
)

include(GNUInstallDirs)
install(TARGETS ${projectName}
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
```

### 1.2 main.cpp

```cpp
#include <QGuiApplication>
#include <QQmlApplicationEngine>

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection
    );
    engine.loadFromModule("HX-01-QML", "Main");

    return app.exec();
}
```

### 1.3 文件结构

```shell
.
| - include
|
| - src
|    | - main.cpp
|
| - qml
|    | - Main.qml
|
| - CMakeLists.txt
```

## 二、初识QML

> [!NOTE]
> 官方文档: [QT-6: QML 参考资料](https://doc.qt.io/qt-6/zh/qmlreference.html)

### 2.1 qml默认内容

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")
}
```

> 这长的像是 js...

### 2.2 Window 常用属性讲解

以下是 Window 的常用属性, 其他属性请查阅官方文档.

```qml
import QtQuick

Window { // root 控件, 父窗口是主界面
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    // 相对于 父控件 的位置
    x: 50
    y: 50

    // 控制窗口大小
    minimumWidth: 400
    minimumHeight: 300
    maximumWidth: 720
    maximumHeight: 560

    // 设置窗口透明度: 0.00 ~ 1.00
    opacity: 0.5
}
```

### 2.3 信号与槽
#### 2.3.1 自定义信号与槽

```qml
Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    // 自定义信号
    // hxSig信号名, `()`是参数列表
    signal hxSig()

    // on + 大写开头的信号 => 就是对应的槽
    onHxSig: {
        
    }
}
```

#### 2.3.2 默认的槽函数

> 这种 `onXxx`的就是触发某信号时候调用的槽函数

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    // 宽度变化触发该槽
    onWidthChanged: {
        console.log("宽度变化:", width)
    }
}
```

### 2.4 自定义属性

```qml
import QtQuick

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    // 自定义属性
    property int hxVal: 0

    // 自定义属性, 也会自动生成对应槽, 并且自动连接
    onHxValChanged: {
        console.log("hxVal 被修改了", hxVal);
    }
}
```

### 2.5 焦点 (focus)

> [!TIP]
> 特别的, 有时候如果我们发现键盘事件没用, 那大概率是像下面这种, `focus` 跑到别的地方去了.

```qml
import QtQuick
import QtQuick.Controls

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")
    color: "#990099" // 颜色

    // 焦点
    Button {
        id: btn1                // 唯一标识
        objectName: "左边的按钮" // 名称 (给人看的)
        focus: true             // 设置默认获取焦点
        
        x: 50
        y: 50
        width: 50
        height: 50

        background: Rectangle { // 设置矩形边框样式
            border.width: 6
            // 根据是否有焦点, 响应式更新
            border.color: btn1.focus ? "blue" : "black";
        }

        onCheckedChanged: {
            console.log("点击了按钮")
        }
        Keys.onRightPressed: {
            btn2.focus = true;
        }
    }

    Button {
        id: btn2
        objectName: "右边的按钮"
        focus: true
        
        x: 150
        y: 50
        width: 50
        height: 50

        background: Rectangle {
            border.width: 6
            border.color: btn2.focus ? "blue" : "black";
        }

        onCheckedChanged: {
            console.log("点击了按钮")
        }
        Keys.onLeftPressed: {
            btn1.focus = true;
        }
    }

    // 焦点改变信号
    onActiveFocusItemChanged: {
        console.log("当前焦点所在元素", activeFocusItem, activeFocusItem.objectName);
    }
}
```