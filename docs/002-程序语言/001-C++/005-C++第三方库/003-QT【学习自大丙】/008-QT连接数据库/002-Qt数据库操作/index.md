# Qt数据库操作
## 1. 概述
Qt框架中对数据库操作提供了很好的支持，我们可以通过Qt提供的类非常方便地和本地或者远程数据库进行连接。众所周知，数据库是 C-S（client-server）结构的，我们要连接的数据库属于服务器端，通过Qt编写的应用程序属于客户端。

如果想用通过Qt访问数据库，首先我们需要在项目中添加数据库模块，模块名为sql。找到项目文件`xxx.pro`，把模块名加进去:

```qmake
QT += core gui sql
```
这个sql模块其实对应的就是一个动态库，动态库中有数据库相关类的二进制源码，在使用动态库中的数据库类的时候，动态库被加载到内存，根据程序猿的需求来提供相应的服务。

首先来介绍一下Qt为我们提供的常用的数据库类:

- **QSqlDatabase**: 通过这个类添加/删除/复制/关闭数据库实例
- **QSqlQuery**: 数据库查询类
- **QSqlRecord**: 数据库记录（通常是数据库中表或视图中的一行）的功能和特征。
- **QSqlField**: 数据库表或视图中单个列的特征，例如数据类型和列名。
- **QSqlQueryModel**: 执行SQL语句和遍历结果集的高级接口。它构建在底层QSqlQuery之上，可以用来为视图类(如QTableView)提供数据。
- **QSqlError**: 数据操作失败可以通过这个类获取相关的错误信息。

在Qt中操作数据库是非常简单的，但是有一个前提条件就是需要使用者能够自己通过SQL语句实现数据库表数据的`添、删、查、改`。在Qt中不论我们连接的何种类型的关系型数据库，在我们使用的时候其操作流程是一致的:

1. 创建数据库实例并初始化
2. 连接数据库
3. 对数据库进行一系列的添、删、查、改操作（编写并执行SQL语句）
4. 关闭数据库

## 2. 类的使用
### 2.1 QSqlDatabase
QSqlDatabase类提供了一个连接访问数据库的接口。一个QSqlDatabase的实例就表示一个连接。该连接通过受支持的数据库驱动程序之一提供对数据库的访问，这些驱动程序派生自QSqlDriver。

下面介绍一下这个中常用的一些 API 函数:

#### 静态函数
1. 得到可以使用的数据库驱动名字的集合

```C++
[static] QStringList QSqlDatabase::drivers();
```

直接调用该函数就可以在输出窗口看到支持的数据库驱动的名字了

```C++
("QSQLITE", "QMYSQL", "QMYSQL3", "QOCI", "QOCI8", "QODBC", "QODBC3", "QPSQL", "QPSQL7")
```

> 在高版本的Qt中，有些数据库的驱动/插件（本质是动态库）需要自己编译，比如：MySQL、Oracle。如果没有对应的驱动/插件是无法进行数据库连接的。

- QMYSQL 是MySQL数据库对应的驱动名
- QOCI 是Oracle数据库对应的驱动名

2. 添加一个数据库实例

```C++
[static] QSqlDatabase QSqlDatabase::addDatabase(
    const QString &type, 
    const QString &connectionName = QLatin1String( defaultConnection )
);
```

该函数的有两个参数分别是:
- type: 指定要连接什么样的数据库，就是数据库驱动对应的驱动名
- connectionName: 数据库连接名，默认叫: `defaultConnection`，我们可以在应用程序中添加多个数据库连接（也就是多个实例），每个连接都对应一个唯一的名字。

函数的返回值就是得到的数据库实例对象。

3. 通过数据库连接名得到数据库实例对象

```C++
[static] QSqlDatabase QSqlDatabase::database(
    const QString &connectionName = QLatin1String( defaultConnection ), 
    bool open = true
);
```

该函数的有两个参数分别是:
- connectionName: 通过`addDatabase()`函数的第二个参数指定的连接名
- open: 实例的状态
    - true: 得到的实例设置为打开状态
    - false: 得到的实例设置为关闭状态

4. 判断连接名对应的连接是否已经存在了，给函数参数指定数据库连接名函数就可以返回连接对应的状态了: true（打开） 或者 false（关闭）。

```C++
[static] bool QSqlDatabase::contains(
    const QString &connectionName = QLatin1String( defaultConnection )
);
```

#### 公共成员函数
1. 给数据库实例设置连接服务器相关的数据信息

- 设置数据库名

```C++
void QSqlDatabase::setDatabaseName(const QString &name);
```

- 设置数据库服务器主机名（一般指定服务器IP地址即可）

```C++
void QSqlDatabase::setHostName(const QString &host);
```

- 设置数据库服务器绑定的端口

```C++
void QSqlDatabase::setPort(int port);
```

> [!TIP]
> 如果数据库服务器绑定的是默认端口，可以省略设置端口的操作。

- 设置连接的数据库中某个用户的用户名

```C++
void QSqlDatabase::setUserName(const QString &name);
```

- 设置连接的数据库中某个用户对应的密码

```C++
void QSqlDatabase::setPassword(const QString &password);
```

2. 连接数据库（必须要先设置连接信息，然后再连接数据库）

```C++
// 数据库连接成功返回true，连接失败返回false。
bool QSqlDatabase::open();
```

3. 判断数据库是否打开了，返回值和open()函数相同。

```C++
bool QSqlDatabase::isOpen() const;
```

4. 关闭数据库连接

```C++
void QSqlDatabase::close();
```

5. 返回有关数据库上发生的最后一个错误的信息。

```C++
QSqlError QSqlDatabase::lastError() const;
QString QSqlError::text() const;
```

> 该函数返回的是一个`QSqlError`对象，调用`QSqlError`类提供的`text()`方法就可以得到最后一个错误对应的信息描述了

6. 事务操作

创建一个事务

```C++
bool QSqlDatabase::transaction();
```

提交事务

```C++
bool QSqlDatabase::commit();
```

事务回滚

```C++
bool QSqlDatabase::rollback();
```

### 2.2 QSqlQuery
QSqlQuery封装了从QSqlDatabase上执行的SQL查询中创建、导航和检索数据所涉及的功能。既可以执行SELECT、INSERT、UPDATE、DELETE等DML(数据操作语言)语句，也可以执行CREATE TABLE等DDL(数据定义语言)语句。

#### 常用成员函数
1. 构造函数

```C++
QSqlQuery::QSqlQuery(
    const QString &query = QString(), 
    QSqlDatabase db = QSqlDatabase()
);
```
参数说明（这两个参数都有默认值，因此可以根据实际需求进行指定）:

- query：要执行的SQL语句
- db：数据库实例对象，<span style="color:red">如果没有指定db，或者是无效的，则使用应用程序的默认数据库。</span>

2. 执行一个SQL语句

```C++
bool QSqlQuery::exec();                     // ①
bool QSqlQuery::exec(const QString &query); // ②
```

- ①：没有参数，执行的SQL语句是在构造函数中指定的。
- ②：有参数，参数对应的字符串就是要执行的SQL语句。

3. 检索查询得到的结果集中的下一条记录(如果可用)，并将查询定位到检索到的记录上。

```C++
bool QSqlQuery::next();
```
该函数检索结果集中的每一条记录的规则如下:

- 如果当前位于结果集第一个记录之前，例如，在执行查询之后，将尝试检索第一个记录。

- 如果当前位于结果集最后一条记录之后，结果集已经检索完毕并返回`false`。

- 如果结果位于结果集中间的某个位置，则尝试检索下一个记录。

函数调用之后，如果检索到有效记录返回`true`，否则返回`false`。

4. 获取当前记录中字段的值

```C++
QVariant QSqlQuery::value(int index) const;  // ①
// 通过字段名直接取出字段值
QVariant QSqlQuery::value(const QString &name) const; // ②
```
- ①：通过字段的索引得到当前字段的值，编号从0开始。
- ②：通过字段的名字得到当前字段的值。

由于数据库表中的字段可以有多种数据类型，因此将这多种类型通过`QVariant`类([四、QVariant](../../002-QT基础/002-Qt中的基础数据类型/004-四、QVariant/index.md))进行了包装从而实现了整齐划一。我们可以通过调用`QVariant`类的API函数得到其内部实际类型的数据。

掌握了`QSqlQuery`类中介绍的以上4个函数之后，程序中对数据库进行添、删、查、改都是没有问题的。关于这个类中的其他函数可以自己查询帮助文档仔细阅读一下。

## 3. 示例代码

```C++
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QLibraryInfo>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 1. 添加一个数据库实例
    QStringList ls = QSqlDatabase::drivers();
    qDebug() << ls;

    QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL");
    // 2. 得到实例对象之后需要初始化连接信息
    //    -- 服务器的IP, 端口, 数据库名, 用户名, 密码
    db.setHostName("127.0.0.1");
    db.setPort(3306); // 如果使用的是默认端口, 可以不调用该函数
    db.setDatabaseName("mysql");
    db.setUserName("root");
    db.setPassword("root");

    // 3. 连接数据库
    bool bl = db.open();
    if (!bl) {
        qDebug() << db.lastError().text();
        return;
    }
    else {
        qDebug() << "数据库连接成功了...";
    }

    QSqlQuery query;
    // 4. 插入数据
    QString sql = "INSERT INTO `hx_demo`.`stu` (`id`, `name`, `sex`, `birthday`, `className`) VALUES (NULL, '张三', 1, '1992-01-17', '计科2班')";

    // 开启事务
    db.transaction();
    bool flag = query.exec(sql);
    if (flag) {
        db.commit(); // 提交事务
    }
    else {
        db.rollback(); // 回溯
    }

    // 5. 查询数据表 - stu
    query.exec("select `name`, `className` from `hx_demo`.`stu`");

    // 遍历结果集
    while(query.next()) {
        // 从当前记录中取出各个字段的值 (两种获取方式)
        qDebug() << query.value(0).toString()
                 << query.value(1).toString()

                 << query.value("name").toString()
                 << query.value("className").toString();
    }

    db.close(); // 关闭数据库连接
}
```
