# QT6.0
- [Qt6通过CMake连接MySQL](https://www.bilibili.com/read/cv16264207/)

- ~~视频: [Qt6配置MySQL驱动，连接MySQL数据库，mingw6.5.3，MySQL8.0](https://www.bilibili.com/video/BV12J4m1H7q4) (评论区可以下载编译好的..)~~

- 只建议: [使用QT6.7.2连接到MySQL数据库](https://blog.csdn.net/qq_45824193/article/details/142385131) (必需要手动编译! 版本得对上!)

## CMake

```CMake
find_package(Qt6 REQUIRED COMPONENTS Sql)

# MySqlDemo 是项目名称
target_link_libraries(MySqlDemo PRIVATE Qt6::Sql)
```

## 测试

```C++
#include "QSqlDatabase"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    qDebug() << QSqlDatabase::drivers(); // 输出: QList("QSQLITE", "QMIMER", "QMARIADB", "QMYSQL", "QODBC", "QPSQL")
}
```

大功告成!