# 二、log输出
## 2.1 在调试窗口中输入日志
在Qt中进行log输出, 一般不使用c中的printf, 也不是使用C++中的cout, Qt框架提供了专门用于日志输出的类, 头文件名为`QDebug`, 使用方法如下:

```C++
#include <QDebug>

// 包含了QDebug头文件, 直接通过全局函数 qDebug() 就可以进行日志输出了
qDebug() << "Date:" << QDate::currentDate();
qDebug() << "Types:" << QString("String") << QChar('x') << QRect(0, 10, 50, 40);
qDebug() << "Custom coordinate type:" << coordinate;

// 和全局函数 qDebug() 类似的日志函数还有: qWarning(), qInfo(), qCritical()
int number = 666;
float i = 11.11;
qWarning() << "Number:" << number << "Other value:" << i;
qInfo() << "Number:" << number << "Other value:" << i;
qCritical() << "Number:" << number << "Other value:" << i;

qDebug() << "我是要成为海贼王的男人!!!";
qDebug() << "我是隔壁的二柱子...";
qDebug() << "我是鸣人, 我擅长嘴遁!!!";
```

## 2.2 在终端窗口中输出日志
使用上面的方法只能在项目调试过程中进行日志输出, 如果不是通过IDE进行程序调试, 而是直接执行可执行程序在这种情况下是没有日志输出窗口的, 因此也就看不到任何的日志输出。

默认情况下日志信息是不会打印到终端窗口的, 如果想要实现这样的效果, 必须在项目文件中添加相关的属性信息

打开项目文件(qmake)（*.pro）找到配置项 config, 添加 console 控制台属性:

```C++
CONFIG += c++17 console
```

属性信息添加完毕, **重新编译项目** 日志信息就可以打印到终端窗口了