## 附. 配置CMake

```CMake
find_package(Qt6 COMPONENTS Network REQUIRED) 
target_link_libraries(你的工程名称 PRIVATE Qt6::Network)
```

> [!TIP]
> 注意, `target_link_libraries`需要放在下面一些(找到`target_link_libraries`的位置)


注: QMake
```QMake
QT += network
```

# 基于TCP的Qt网络通信
在标准C++没有提供专门用于套接字通信的类，所以只能使用操作系统提供的基于C的API函数，基于这些C的API函数我们也可以封装自己的C++类([对常用操作进行类封装](../../../../002-tmp丶C++丶memo/004-C++网络编程/001-实践版本/006-对常用操作进行类封装/index.md))。但是Qt就不一样了，它是C++的一个框架并且里边提供了用于套接字通信的类（TCP、UDP）这样就使得我们的操作变得更加简单了（当然，在Qt中使用标准C的API进行套接字通信也是完全没有问题的）。下面，给大家讲一下如果使用相关类的进行TCP通信。

使用Qt提供的类进行基于TCP的套接字通信需要用到两个类:

- <span style="color:red">`QTcpServer`: 服务器类，用于监听客户端连接以及和客户端建立连接。</span>

- <span style="color:red">`QTcpSocket`: 通信的套接字类，客户端、服务器端都需要使用。</span>

这两个套接字通信类都属于网络模块`network`。

## 1. QTcpServer
`QTcpServer`类用于监听客户端连接以及和客户端建立连接，在使用之前先介绍一下这个类提供的一些常用API函数:

### 1.1 公共成员函数
构造函数

```C++
QTcpServer::QTcpServer(QObject *parent = Q_NULLPTR);
```

给监听的套接字设置监听

```C++
bool QTcpServer::listen(const QHostAddress &address = QHostAddress::Any, quint16 port = 0);
// 判断当前对象是否在监听, 是返回true，没有监听返回false
bool QTcpServer::isListening() const;
// 如果当前对象正在监听返回监听的服务器地址信息, 否则返回 QHostAddress::Null
QHostAddress QTcpServer::serverAddress() const;
// 如果服务器正在侦听连接，则返回服务器的端口; 否则返回0
quint16 QTcpServer::serverPort() const
```

- 参数：
    - **address**：通过类`QHostAddress`可以封装`IPv4`、`IPv6`格式的IP地址，`QHostAddress::Any`表示自动绑定
    - **port**：如果指定为0表示随机绑定一个可用端口。
- 返回值：绑定成功返回true，失败返回false

得到和客户端建立连接之后用于通信的`QTcpSocket`套接字对象，它是`QTcpServer`的一个子对象，当`QTcpServer`对象析构的时候会自动析构这个子对象，当然也可自己手动析构，建议用完之后自己手动析构这个通信的`QTcpSocket`对象。

```C++
QTcpSocket *QTcpServer::nextPendingConnection();
```

阻塞等待客户端发起的连接请求，<span style="color:red">不推荐在单线程程序中使用，建议使用非阻塞方式处理新连接，即使用信号`newConnection()`。</span>

```C++
bool QTcpServer::waitForNewConnection(int msec = 0, bool *timedOut = Q_NULLPTR);
```

- 参数：
    - `msec`：指定阻塞的最大时长，单位为毫秒（ms）
    - `timeout`：传出参数，如果操作超时`timeout`为true，没有超时`timeout`为false

### 1.2 信号
当接受新连接导致错误时，将发射如下信号。`socketError`参数描述了发生的错误相关的信息。

```C++
[signal] void QTcpServer::acceptError(QAbstractSocket::SocketError socketError);
```

每次有新连接可用时都会发出`newConnection()`信号。

```C++
[signal] void QTcpServer::newConnection();
```

## 2. QTcpSocket
`QTcpSocket`是一个套接字通信类，不管是客户端还是服务器端都需要使用。在Qt中发送和接收数据也属于IO操作（网络IO），先来看一下这个类的继承关系:

| ##container## |
|:--:|
|![Clip_2024-09-29_22-18-45.png ##w700##](./Clip_2024-09-29_22-18-45.png)|

### 2.1 公共成员函数
构造函数

```C++
QTcpSocket::QTcpSocket(QObject *parent = Q_NULLPTR);
```

**连接服务器**，需要指定服务器端绑定的IP和端口信息。

```C++
[virtual] void QAbstractSocket::connectToHost(const QString &hostName, quint16 port, OpenMode openMode = ReadWrite, NetworkLayerProtocol protocol = AnyIPProtocol);

[virtual] void QAbstractSocket::connectToHost(const QHostAddress &address, quint16 port, OpenMode openMode = ReadWrite);
```

<span style="color:red">在Qt中不管调用读操作函数接收数据，还是调用写函数发送数据，操作的对象都是本地的由Qt框架维护的一块内存。因此，调用了发送函数数据不一定会马上被发送到网络中，调用了接收函数也不是直接从网络中接收数据，关于底层的相关操作是不需要使用者来维护的。</span>

接收数据

```C++
// 指定可接收的最大字节数 maxSize 的数据到指针 data 指向的内存中
qint64 QIODevice::read(char *data, qint64 maxSize);
// 指定可接收的最大字节数 maxSize，返回接收的字符串
QByteArray QIODevice::read(qint64 maxSize);
// 将当前可用操作数据全部读出，通过返回值返回读出的字符串
QByteArray QIODevice::readAll();
```

发送数据

```C++
// 发送指针 data 指向的内存中的 maxSize 个字节的数据
qint64 QIODevice::write(const char *data, qint64 maxSize);
// 发送指针 data 指向的内存中的数据，字符串以 \0 作为结束标记
qint64 QIODevice::write(const char *data);
// 发送参数指定的字符串
qint64 QIODevice::write(const QByteArray &byteArray);
```

### 2.2 信号
在使用`QTcpSocket`进行套接字通信的过程中，如果该类对象发射出`readyRead()`信号，说明对端发送的数据达到了，之后就可以调用`read`函数接收数据了。

```C++
[signal] void QIODevice::readyRead();
```

调用`connectToHost()`函数并成功建立连接之后发出`connected()`信号。

```C++
[signal] void QAbstractSocket::connected();
```

在套接字断开连接时发出`disconnected()`信号。

```C++
[signal] void QAbstractSocket::disconnected();
```

## 3. 通信流程
使用Qt提供的类进行套接字通信比使用标准C API进行网络通信要简单（因为在内部进行了封装([网络协议与进程间通信](../../../../002-tmp丶C++丶memo/004-C++网络编程/001-实践版本/002-网络协议与进程间通信/index.md))） Qt中的套接字通信流程如下:

### 3.1 服务器端
#### 3.1.1 通信流程

1. 创建套接字服务器`QTcpServer`对象
2. 通过`QTcpServer`对象设置监听，即：`QTcpServer::listen()`
3. 基于`QTcpServer::newConnection()`信号检测是否有新的客户端连接
4. 如果有新的客户端连接调用`QTcpSocket *QTcpServer::nextPendingConnection()`得到通信的套接字对象
5. 使用通信的套接字对象`QTcpSocket`和客户端进行通信

#### 3.1.2 代码片段
服务器端 & 客户端的窗口界面如下图所示:

| ##container## |
|:--:|
|![Clip_2024-09-29_23-39-44.png ##w600##](./Clip_2024-09-29_23-39-44.png)|

- 头文件

```C++
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_startServer_clicked();
    void on_sendMsg_clicked();

private:
    Ui::MainWindow *ui;
    QTcpServer* m_server;
    QTcpSocket* m_tcp;
};
```

- 源文件
 
```C++
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("TCP - 服务器");
    // 创建 QTcpServer 对象
    m_server = new QTcpServer(this);
    // 检测是否有新的客户端连接
    connect(m_server, &QTcpServer::newConnection, this, [=]() {
        m_tcp = m_server->nextPendingConnection();
        ui->record->append("成功和客户端建立了新的连接...");
        m_status->setPixmap(QPixmap(":/connect.png").scaled(20, 20));
        // 检测是否有客户端数据
        connect(m_tcp, &QTcpSocket::readyRead, this, [=]() {
            // 接收数据
            QString recvMsg = m_tcp->readAll();
            ui->record->append("客户端Say: " + recvMsg);
        });
        // 客户端断开了连接
        connect(m_tcp, &QTcpSocket::disconnected, this, [=]() {
            ui->record->append("客户端已经断开了连接...");
            m_tcp->deleteLater();
            m_status->setPixmap(QPixmap(":/disconnect.png").scaled(20, 20));
        });
    });
}

MainWindow::~MainWindow() {
    delete ui;
}

// 启动服务器端的服务按钮
void MainWindow::on_startServer_clicked() {
    unsigned short port = ui->port->text().toInt();
    // 设置服务器监听
    m_server->listen(QHostAddress::Any, port);
    ui->startServer->setEnabled(false);
}

// 点击发送数据按钮
void MainWindow::on_sendMsg_clicked() {
    QString sendMsg = ui->msg->toPlainText();
    m_tcp->write(sendMsg.toUtf8());
    ui->record->append("服务器Say: " + sendMsg);
    ui->msg->clear();
}
```

### 3.2 客户端
#### 3.2.1 通信流程

1. 创建通信的套接字类`QTcpSocket`对象
2. 使用服务器端绑定的IP和端口连接服务器`QAbstractSocket::connectToHost()`
3. 使用`QTcpSocket`对象和服务器进行通信

#### 3.2.2 代码片段

- 头文件

```C++
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_connectServer_clicked();
    void on_sendMsg_clicked();
    void on_disconnect_clicked();

private:
    Ui::MainWindow *ui;
    QTcpSocket* m_tcp;
};
```

- 源文件

```C++
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("TCP - 客户端");

    // 创建通信的套接字对象
    m_tcp = new QTcpSocket(this);
    // 检测服务器是否回复了数据
    connect(m_tcp, &QTcpSocket::readyRead, [=]() {
        // 接收服务器发送的数据
        QByteArray recvMsg = m_tcp->readAll();
        ui->record->append("服务器Say: " + recvMsg);
    });

    // 检测是否和服务器是否连接成功了
    connect(m_tcp, &QTcpSocket::connected, this, [=]() {
        ui->record->append("恭喜, 连接服务器成功!!!");
        m_status->setPixmap(QPixmap(":/connect.png").scaled(20, 20));
    });

    // 检测服务器是否和客户端断开了连接
    connect(m_tcp, &QTcpSocket::disconnected, this, [=]() {
        ui->record->append("服务器已经断开了连接, ...");
        ui->connectServer->setEnabled(true);
        ui->disconnect->setEnabled(false);
    });
}

MainWindow::~MainWindow() {
    delete ui;
}

// 连接服务器按钮按下之后的处理动作
void MainWindow::on_connectServer_clicked() {
    QString ip = ui->ip->text();
    unsigned short port = ui->port->text().toInt();
    // 连接服务器
    m_tcp->connectToHost(QHostAddress(ip), port);
    ui->connectServer->setEnabled(false);
    ui->disconnect->setEnabled(true);
}

// 发送数据按钮按下之后的处理动作
void MainWindow::on_sendMsg_clicked() {
    QString sendMsg = ui->msg->toPlainText();
    m_tcp->write(sendMsg.toUtf8());
    ui->record->append("客户端Say: " + sendMsg);
    ui->msg->clear();
}

// 断开连接按钮被按下之后的处理动作
void MainWindow::on_disconnect_clicked() {
    m_tcp->close();
    ui->connectServer->setEnabled(true);
    ui->disconnect->setEnabled(false);
}
```
