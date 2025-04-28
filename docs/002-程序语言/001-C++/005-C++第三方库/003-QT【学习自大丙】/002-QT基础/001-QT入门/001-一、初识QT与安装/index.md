# 一、初识QT 与 安装
## 1.1 环境搭建
- [Qt6安装](https://subingwen.cn/qt/qt6-install/)
- [Qt入门](https://subingwen.cn/qt/qt-primer/) (配置环境变量, QTIDE基本使用)

## 1.2 第一个QT程序
- main.cpp
```C++
#include "mainwindow.h"		// 生成的窗口类头文件
#include <QApplication>		// 应用程序类头文件

int main(int argc, char *argv[]) {
    // 创建应用程序对象, 在一个Qt项目中实例对象有且仅有一个
    // 类的作用: 检测触发的事件, 进行事件循环并处理
    QApplication a(argc, argv);
    // 创建窗口类对象
    MainWindow w;
    // 显示窗口
    w.show();
    // 应用程序对象开始事件循环, 保证应用程序不退出
    return a.exec();
}
```

- mainwindow.ui

	在Qt中每一个窗口都对应一个可编辑的可视化界面（*.ui）, 这个界面对应的是一个xml格式的文件, 一般情况下不需要在xml格式下对这个文件进行编辑, 关于这个文件结构了解即可。

```xml
<!-- 双击这个文件看到的是一个窗口界面, 如果使用文本编辑器打开看到的是一个XML格式的文件 -->
<!-- 看不懂这种格式没关系, 我们不需要在这种模式下操作这个文件。 -->
<!-- 这里只是给大家介绍这个文件的本质 -->
<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>MainWindow</class>
 <widget class="QMainWindow" name="MainWindow">
  <property name="geometry">
   <rect>
    <x>0</x>
    <y>0</y>
    <width>800</width>
    <height>600</height>
   </rect>
  </property>
  <property name="windowTitle">
   <string>MainWindow</string>
  </property>
  <widget class="QWidget" name="centralwidget"/>
  <widget class="QMenuBar" name="menubar"/>
  <widget class="QStatusBar" name="statusbar"/>
 </widget>
 <resources/>
 <connections/>
</ui>
```

- mainwindow.h

```C++
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>		// Qt标准窗口类头文件

QT_BEGIN_NAMESPACE
// mainwindow.ui 文件中也有一个类叫 MainWindow, 将这个类放到命名空间 Ui 中
namespace Ui { class MainWindow; }	
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT	// 这个宏是为了能够使用Qt中的信号槽机制

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;		// 定义指针指向窗口的 UI 对象
};
#endif // MAINWINDOW_H
```

- mainwindow.cpp

```C++
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow) { // 基于mainwindow.ui创建一个实例对象
    // 将 mainwindow.ui 的实例对象和 当前类的对象进行关联
    // 这样同名的连个类对象就产生了关联, 合二为一了
    ui->setupUi(this);
}

MainWindow::~MainWindow() {
    delete ui;
}
```
