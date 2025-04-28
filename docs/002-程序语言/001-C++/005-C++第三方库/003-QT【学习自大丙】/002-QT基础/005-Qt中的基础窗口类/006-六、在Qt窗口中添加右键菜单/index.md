# 六、在Qt窗口中添加右键菜单
如果想要在某一窗口中显示右键菜单, 其处理方式大体上有两种, 这两种方式分别为基于鼠标事件实现和基于窗口的菜单策略实现。其中第二种方式中又有三种不同的实现方式, 因此如果想要在窗口中显示一个右键菜单一共四种实现方式, 

## 6.1 基于鼠标事件实现
### 6.1.1 实现思路
> 使用这种方式实现右键菜单的显示需要使用`事件处理器函数`, 在Qt中这类函数都是回调函数, 并且在自定义窗口类中我们还可以自定义事件处理器函数的行为（因为子类继承了父类的这个方法并且这类函数是虚函数）。
实现步骤如下

1. 在当前窗口类中重写鼠标操作相关的的事件处理器函数，有两个可以选择
 
```C++
// 以下两个事件二选一即可, 只是事件函数被调用的时机不同罢了
// 这个时机对右键菜单的显示没有任何影响
[virtual protected] void QWidget::mousePressEvent(QMouseEvent *event);
[virtual protected] void QWidget::mouseReleaseEvent(QMouseEvent *event);
```
2. 在数据表事件处理器函数内部判断是否按下了鼠标右键

3. 如果按下了鼠标右键创建菜单对象(也可以提前先创建处理), 并将其显示出来

```C++
// 关于QMenu类型的菜单显示需要调用的 API
// 参数 p 就是右键菜单需要显示的位置, 这个坐标需要使用屏幕坐标
// 该位置坐标一般通过调用 QCursor::pos() 直接就可以得到了
QAction *QMenu::exec(const QPoint &p, QAction *action = nullptr);
```

### 6.1.2 代码实现
在头文件中添加要重写的鼠标事件处理器函数声明, 这里使用的是 mousePressEvent()

```C++
// mainwindow.h
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    // 鼠标按下, 该函数被Qt框架调用, 需要重写该函数
    void mousePressEvent(QMouseEvent *event);

private:
    Ui::MainWindow *ui;
};
```

在源文件中重写从父类继承的虚函数mousePressEvent()

```C++
// mainwindow.cpp
void MainWindow::mousePressEvent(QMouseEvent *event) {
    // 判断用户按下的是哪一个鼠标键
    if(event->button() == Qt::RightButton) {
        // 弹出一个菜单, 菜单项是 QAction 类型
        QMenu menu;
        QAction* act = menu.addAction("C++");
        connect(act, &QAction::triggered, this, [=]() {
            QMessageBox::information(this, "title", "您选择的是C++...");
        });
        menu.addAction("Java");
        menu.addAction("Python");
        menu.exec(QCursor::pos()); // 右键菜单被模态显示出来了
    }
}
```

## 6.2 基于窗口的菜单策略实现
> 这种方式是使用 Qt 中`QWidget`类中的右键菜单函数`QWidget::setContextMenuPolicy(Qt::ContextMenuPolicy policy)`来实现, 因为这个函数的参数可以指定不同的值, 因此不同参数对应的具体的实现方式也不同。
>
> 这个函数的函数原型如下

```C++
// 函数原型:
void QWidget::setContextMenuPolicy(Qt::ContextMenuPolicy policy);
/* 参数:     
  - Qt::NoContextMenu         --> 不能实现右键菜单
  - Qt::PreventContextMenu   --> 不能实现右键菜单
  - Qt::DefaultContextMenu   --> 基于事件处理器函数 QWidget::contextMenuEvent() 实现
  - Qt::ActionsContextMenu   --> 添加到当前窗口中所有 QAction 都会作为右键菜单项显示出来
  - Qt::CustomContextMenu    --> 基于 QWidget::customContextMenuRequested() 信号实现 */
```

### 6.2.1 Qt::DefaultContextMenu
> 使用这个策略实现右键菜单, 需要借助窗口类从父类继承的虚函数`QWidget::contextMenuEvent()`并重写它来实现。

要做的第一步是在窗口类的头文件中添加这个函数的声明

```C++
// mainwindow.h
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    // 如果窗口设置了 Qt::DefaultContextMenu 策略, 
    // 点击鼠标右键该函数被Qt框架调用
    void contextMenuEvent(QContextMenuEvent *event);

private:
    Ui::MainWindow *ui;
};
```

第二步在这个窗口类的构造函数设置右键菜单策略

```C++
// mainwindow.cpp
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 给窗口设置策略: Qt::DefaultContextMenu
    // 在窗口中按下鼠标右键, 这个事件处理器函数被qt框架调用 QWidget::contextMenuEvent()
    setContextMenuPolicy(Qt::DefaultContextMenu);
}
```

第三步在这个窗口类的源文件中重写事件处理器函数`contextMenuEvent()`

```C++
// mainwindow.cpp
void MainWindow::contextMenuEvent(QContextMenuEvent *event) {
    // 弹出一个菜单, 菜单项是 QAction 类型
    QMenu menu;
    QAction* act = menu.addAction("C++");
    connect(act, &QAction::triggered, this, [=]() {
        QMessageBox::information(this, "title", "您选择的是C++...");
    });
    menu.addAction("Java");
    menu.addAction("Python");
    menu.exec(QCursor::pos());    // 右键菜单被模态显示出来了
}
```

### 6.2.2 Qt::ActionsContextMenu
> 使用这个策略实现右键菜单, 是最简单的一种, 我们只需要创建一些`QAction`类型的对象并且将他们添加到当前的窗口中, 当我们在窗口中点击鼠标右键这些`QAction`类型的菜单项就可以显示出来了。
>
> 虽然这种方法比较简单，但是它有一定的局限性，就是在一个窗口中不能根据不同的需求制作不同的右键菜单，这种方式只能得到一个唯一的右键菜单。
>
> 相关的处理代码如下

```C++
// mainwindow.cpp
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 只要将某个QAction添加给对应的窗口, 这个action就是这个窗口右键菜单中的一个菜单项了
    // 在窗口中点击鼠标右键, 就可以显示这个菜单
    setContextMenuPolicy(Qt::ActionsContextMenu);
    // 给当前窗口添加QAction对象
    QAction* act1  = new QAction("C++");
    QAction* act2 = new QAction("Java");
    QAction* act3  = new QAction("Python");
    this->addAction(act1);
    this->addAction(act2);
    this->addAction(act3);
    connect(act1, &QAction::triggered, this, [=]() {
         QMessageBox::information(this, "title", "您选择的是C++...");
    });
}
```

### 6.2.3 Qt::CustomContextMenu
> 使用这个策略实现右键菜单, 当点击鼠标右键，窗口会产生一个`QWidget::customContextMenuRequested()`信号，注意仅仅只是发射信号，意味着要自己写显示右键菜单的槽函数（slot），这个信号是QWidget唯一与右键菜单有关的信号。
>
> 我们先来看一下这个信号的函数原型:
 
```C++
// 注意: 信号中的参数pos为当前窗口的坐标，并非屏幕坐标，右键菜单显示需要使用屏幕坐标
[signal] void QWidget::customContextMenuRequested(const QPoint &pos)
```

代码实现也比较简单, 如下所示:

```C++
// mainwindow.cpp
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 策略 Qt::CustomContextMenu
    // 当在窗口中点击鼠标右键, 窗口会发出一个信号: QWidget::customContextMenuRequested()
    // 对应发射出的这个信号, 需要添加一个槽函数, 用来显示右键菜单
    this->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, &MainWindow::customContextMenuRequested, this, [=](const QPoint &pos) {
        // 参数 pos 是鼠标按下的位置, 但是不能直接使用, 这个坐标不是屏幕坐标, 是当前窗口的坐标
        // 如果要使用这个坐标需要将其转换为屏幕坐标
        QMenu menu;
        QAction* act = menu.addAction("C++");
        connect(act, &QAction::triggered, this, [=]() {
            QMessageBox::information(this, "title", "您选择的是C++...");
        });
        menu.addAction("Java");
        menu.addAction("Python");
        // menu.exec(QCursor::pos());
        // 将窗口坐标转换为屏幕坐标
        QPoint newpt = this->mapToGlobal(pos);
        menu.exec(newpt);
    });
}
```

在上边的程序中, 我们通过窗口发射的信号得到了一个坐标类型的参数, 大家一定要 **注意这个坐标是当前窗口的窗口坐标, 不是屏幕坐标, 显示右键菜单需要使用屏幕坐标**。

对应这个坐标的处理可以有两种方式:

1. 弃用，选择使用`QCursor::pos()`得到光标在屏幕的坐标位置

2. 坐标转换, 将窗口坐标转换为屏幕坐标, 这里用到了一个函数`mapToGlobal`

```C++
// 参数是当前窗口坐标, 返回值为屏幕坐标
QPoint QWidget::mapToGlobal(const QPoint &pos) const;
```

不管使用以上哪种方式显示右键菜单, 显示出来之后的效果是一样的

## 6.3 附: 自定义右键菜单项显示图标函数

最后如果想要让自己的右键菜单项显示图标, 可以调用这个函数:

```C++
// 只显示文本字符串
QAction *QMenu::addAction(const QString &text);
// 可以显示图标 + 文本字符串
QAction *QMenu::addAction(const QIcon &icon, const QString &text);
```
