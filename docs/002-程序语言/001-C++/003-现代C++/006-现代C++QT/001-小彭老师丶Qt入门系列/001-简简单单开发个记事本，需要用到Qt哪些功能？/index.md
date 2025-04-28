# 【Qt入门系列】简简单单开发个记事本, 需要用到Qt哪些功能？

- B站: [【Qt入门系列】简简单单开发个记事本, 需要用到Qt哪些功能？](https://www.bilibili.com/video/BV1PfqdYaEnU/)

## 1. Qt的反射机制
### 1.1 Qt反射机制初步介绍
Qt 的反射机制不是依赖 C++ 本身实现, 而是通过`Q_OBJECT`等宏标记需要支持反射的类、信号、槽等, 在编译时由 Qt 的 MOC 工具解析这些标记, 生成一份额外的代码文件(如元数据表、虚函数重载等)。这些生成的代码与用户代码一起编译, 从而实现了反射功能, 比如动态调用、信号槽绑定等。

同理, Qt 的 UI 系统也类似。Qt 的`.ui`文件是用 XML 描述界面的, 这些文件在编译时通过 **UIC(User Interface Compiler)** 工具转化为对应的 C++ 代码(通常是一个类的定义和初始化代码)。这个生成的代码文件与用户自定义逻辑结合, 最终实现 UI 的功能。

咱们可以看看`Q_OBJECT`的宏定义:
```C++
/* qmake ignore Q_OBJECT */
#define Q_OBJECT \
public: \
    QT_WARNING_PUSH \
    Q_OBJECT_NO_OVERRIDE_WARNING \
    // QMetaObject 是Q元对象
    static const QMetaObject staticMetaObject; \
    virtual const QMetaObject *metaObject() const; \
    virtual void *qt_metacast(const char *); \
    virtual int qt_metacall(QMetaObject::Call, int, void **); \
    QT_TR_FUNCTIONS \
private: \
    Q_OBJECT_NO_ATTRIBUTES_WARNING \
    Q_DECL_HIDDEN_STATIC_METACALL static void qt_static_metacall(QObject *, QMetaObject::Call, int, void **); \
    QT_WARNING_POP \
    QT_DEFINE_TAG_STRUCT(QPrivateSignal); \
    QT_ANNOTATE_CLASS(qt_qobject, "")
```

### 1.2 从实例看反射

头文件:

```C++
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT // Qt反射宏

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
	// 写了一个槽函数
    void on_action_4_triggered(bool checked);

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
```

cpp文件

```C++
#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow) // 注入ui, 里面有一个`action_4`变量
{
    ui->setupUi(this);
}

MainWindow::~MainWindow() {
    delete ui;
}

// 槽函数, 通过 Qt 的元对象系统（反射机制）, 会自动绑定到 action_4 的槽, 监听其 triggered(bool) 信号
void MainWindow::on_action_4_triggered(bool checked) {
    qDebug() << checked;
}
```

cmake编译

```C++
[1/5 0.6/sec] Automatic MOC and UIC for target textApp // 目标textApp的自动MOC和UIC
[2/5 0.4/sec] Building CXX object CMakeFiles/textApp.dir/textApp_autogen/mocs_compilation.cpp.obj
[3/5 0.6/sec] Building CXX object CMakeFiles/textApp.dir/main.cpp.obj
[4/5 0.8/sec] Building CXX object CMakeFiles/textApp.dir/mainwindow.cpp.obj
[5/5 1.0/sec] Linking CXX executable textApp.exe
```

会发现, 其生成了:

- .\build\Desktop_Qt_6_8_0_llvm_mingw_64_bit-Debug\textApp_autogen\EWIEGA46WW\moc_mainwindow.cpp

```C++
/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.8.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../mainwindow.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.8.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSMainWindowENDCLASS_t {};
constexpr auto qt_meta_stringdata_CLASSMainWindowENDCLASS = QtMocHelpers::stringData(
    "MainWindow",
    "on_action_4_triggered",
    "",
    "checked"
);
#else  // !QT_MOC_HAS_STRINGDATA
#error "qtmochelpers.h not found or too old."
#endif // !QT_MOC_HAS_STRINGDATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSMainWindowENDCLASS[] = {

 // content:
      12,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   20,    2, 0x08,    1 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,

       0        // eod
};

Q_CONSTINIT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_CLASSMainWindowENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSMainWindowENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSMainWindowENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<MainWindow, std::true_type>,
        // method 'on_action_4_triggered'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>
    >,
    nullptr
} };

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_action_4_triggered((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSMainWindowENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 1;
    }
    return _id;
}
QT_WARNING_POP
```

可以发现, 它导入了`"../../../../mainwindow.h"`, 并且实现了以下函数

```C++
void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a);

const QMetaObject *MainWindow::metaObject() const;

void *MainWindow::qt_metacast(const char *_clname);

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a);
```

这里就对应了`Q_OBJECT`宏的:
```C++
public: \
    virtual const QMetaObject *metaObject() const; \
    virtual void *qt_metacast(const char *); \
    virtual int qt_metacall(QMetaObject::Call, int, void **); \
private: \
    Q_DECL_HIDDEN_STATIC_METACALL static void qt_static_metacall(QObject *, QMetaObject::Call, int, void **);
```

<details>
<summary>具体意义: (By GPT)</summary>

```C++
public: \
    // 返回指向当前类的 QMetaObject 的指针，用于描述该类的元信息（如信号、槽、属性等）
    virtual const QMetaObject *metaObject() const; \
    
    // 根据类名字符串动态转换为对应的类类型，如果无法匹配返回 nullptr
    virtual void *qt_metacast(const char *); \
    
    // 根据调用类型（QMetaObject::Call）和方法 ID 调用相应的槽或信号，支持动态调用机制
    virtual int qt_metacall(QMetaObject::Call, int, void **); \
    
private: \
    // 静态的元调用函数，用于处理信号与槽的绑定和执行逻辑
    // 第一个参数是目标对象，
    // 第二个参数是调用类型（如 InvokeMetaMethod），
    // 第三个参数是方法 ID，(_id 是槽函数的索引，由 MOC 自动生成，确保槽函数可以通过 ID 唯一标识)
    // 第四个参数是参数数组
    Q_DECL_HIDDEN_STATIC_METACALL static void qt_static_metacall(QObject *, QMetaObject::Call, int, void **);
```
</details>

同时也对`static const QMetaObject staticMetaObject;`进行了初始化:

```C++
Q_CONSTINIT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_CLASSMainWindowENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSMainWindowENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSMainWindowENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<MainWindow, std::true_type>,
        // method 'on_action_4_triggered'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>
    >,
    nullptr
} };
```

<details>
<summary>具体意义: (By GPT)</summary>

```C++
Q_CONSTINIT const QMetaObject MainWindow::staticMetaObject = { {
    // 用于链接父类的 QMetaObject，表明 MainWindow 的元对象继承自 QMainWindow 的元对象
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),

    // 存储类名、方法名等字符串的偏移和大小，供元对象系统使用
    qt_meta_stringdata_CLASSMainWindowENDCLASS.offsetsAndSizes,

    // 该类的元数据表，包含信号、槽、属性等信息的描述（见 moc_mainwindow.cpp 中的 qt_meta_data_CLASSMainWindowENDCLASS 数组）
    qt_meta_data_CLASSMainWindowENDCLASS,

    // 静态元调用函数，用于根据信号或槽的 ID 调用对应的方法
    qt_static_metacall,

    // 动态属性回调函数，通常为 nullptr，除非启用了动态属性支持
    nullptr,

    // 元类型数组，定义信号与槽参数的类型信息，用于动态调用时的类型匹配
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSMainWindowENDCLASS_t,
        // 描述类的类型信息，这里指定 MainWindow 是完整类型（std::true_type）
        QtPrivate::TypeAndForceComplete<MainWindow, std::true_type>,

        // 槽函数 'on_action_4_triggered' 的返回值类型（void）
        QtPrivate::TypeAndForceComplete<void, std::false_type>,

        // 槽函数 'on_action_4_triggered' 的参数类型（bool）
        QtPrivate::TypeAndForceComplete<bool, std::false_type>
    >,

    // 备用指针，用于额外扩展功能，目前为空
    nullptr
} };
```

</details>

### 1.3 总结

質問: 所以就实现了自动的槽绑定？(实际上还是静态生成的, 只是MOC帮忙编译了一次？)之后我们程序可以直接通过字符串来调用吗？还是可以 for of 这个类的所有槽？

<details>
<summary>答えを見る: (By GPT)</summary>

### 自动槽绑定的实现机制

实际上，Qt 的槽绑定是 **静态生成 + 动态调用** 的组合，它主要依赖以下几点：

1. **MOC 静态生成代码**  
   - MOC 根据 `Q_OBJECT` 宏解析槽函数，并在编译前生成与槽相关的元信息（如函数名、参数类型等），存储在 `QMetaObject` 结构中。
   - `qt_static_metacall` 是静态生成的代码，用于在运行时调用具体的槽函数。

2. **信号与槽动态关联**  
   - 在运行时，通过 `connect` 函数将信号与槽进行绑定，利用元信息（如槽名和参数）来完成动态调用。

---

### 程序能否通过字符串直接调用槽函数？

是的，通过 `QMetaObject` 和反射机制，可以通过字符串调用槽函数。这需要以下步骤：

1. **获取元对象**  
   调用 `QObject::metaObject()` 获取当前类的 `QMetaObject`，该对象包含所有槽函数的元信息。

2. **调用槽函数**  
   使用 `QMetaObject::invokeMethod` 函数，通过字符串指定方法名进行调用：
   ```cpp
   QMetaObject::invokeMethod(object, "on_action_4_triggered", Q_ARG(bool, true));
   ```
   - `object` 是目标对象指针。
   - `"on_action_4_triggered"` 是槽函数的名字。
   - `Q_ARG` 是 Qt 提供的宏，用于封装参数类型和值。

---

### 能否遍历类的所有槽？

不能直接遍历**仅槽函数**，但可以通过元对象遍历所有的成员函数，包括槽函数、信号等。具体方法：

1. **遍历元信息中的方法**  
   使用 `QMetaObject` 的 `methodCount` 和 `method()`：
   ```cpp
   const QMetaObject *metaObj = object->metaObject();
   for (int i = 0; i < metaObj->methodCount(); ++i) {
       QMetaMethod method = metaObj->method(i);
       qDebug() << method.methodSignature();
   }
   ```

2. **过滤槽函数**  
   检查 `QMetaMethod::methodType()` 是否为 `QMetaMethod::Slot`：
   ```cpp
   if (method.methodType() == QMetaMethod::Slot) {
       qDebug() << "Slot:" << method.methodSignature();
   }
   ```

---

### 静态与动态的结合点

- **静态生成**：MOC 编译阶段生成槽函数的元信息和调用代码（如 `qt_static_metacall`）。
- **动态调用**：运行时可以通过字符串名称调用槽函数，或通过 `QMetaObject` 进行元数据遍历。

因此，从程序员的角度来看，**槽绑定是自动完成的**，而其实现实际上是在编译期和运行时共同作用的结果。这种设计既提供了开发便捷性，又保持了 C++ 的高性能特性。

</details>

## 2. Qt 宏规范

哇! 好高级! 这个是 Qt++, 不是C++啦! 爱啦爱啦! 是高大上的**关键字**!

```C++
private slots:
	// ...自定义槽

signals:
	// ...信号
```

实际上只是: (宏)

```C++
#ifdef signals
#  define signals public __attribute__((annotate("qt_signal")))
#endif

#ifdef slots
#  define slots __attribute__((annotate("qt_slot")))
#endif

#ifdef Q_SIGNALS
#  define Q_SIGNALS public __attribute__((annotate("qt_signal")))
#endif

#ifdef Q_SLOTS
#  define Q_SLOTS __attribute__((annotate("qt_slot")))
#endif

// 下面是声明单个槽/信号 使用的宏 (很少用到)
#ifdef Q_SIGNAL
#  define Q_SIGNAL __attribute__((annotate("qt_signal")))
#endif

#ifdef Q_SLOT
#  define Q_SLOT __attribute__((annotate("qt_slot")))
#endif

// 这些宏的功能是为 Qt 的信号和槽机制提供标记，主要用于告诉 MOC（Meta-Object Compiler） 哪些成员函数是信号或者槽。
```

小彭老师如是说到, 更加推荐`Q_SIGNALS`、`Q_SLOTS`的用法!

因为当你导入`<Python.h>`库, slots 关键字, 就会重名啦! 那就报错啦!

因此, 还是使用更加现代的`Q_SIGNALS`、`Q_SLOTS`吧!

## 3. Qt 的跨平台配置文件

Qt中提供了`<QSettings>`非常方便的, 它不是普通的ini或者json或者xml, 而是他们的抽象!

在win系统, `<QSettings>`可以直接把配置写到 **注册表** 中, 而在Linux中, 则可以把配置文件保存到`~/.config`中(以`.ini`的文件格式)

使用也非常简单, 只需要`{"机构名", "项目名"}`即可使用

(剩下的自己琢磨~)