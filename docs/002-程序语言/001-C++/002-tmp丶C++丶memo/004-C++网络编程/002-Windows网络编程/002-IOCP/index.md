# IOCP（I/O完成端口）
IOCP是Windows平台下一种高效的线程同步机制，专门用于处理大量的并发输入/输出操作。它允许应用程序在多个线程之间高效地分配I/O操作，提高了网络应用和文件操作的性能。使用IOCP，可以让应用程序只有在I/O操作完成时才接收通知，从而减少资源的占用和提高应用程序的响应能力。

## 比较
[技术派-epoll和IOCP之比较](https://zhuanlan.zhihu.com/p/106957967)

## 核心概念
- **完成端口**：一个抽象的、与系统关联的I/O操作队列，可以关联多个I/O对象（如socket、文件句柄等）。

- **工作线程**：从完成端口获取已完成的I/O操作通知，并执行相应的处理逻辑。

- **关联**：将I/O对象（如socket）与完成端口绑定，使得该对象上的I/O操作完成后，相关的完成包会被投递到完成端口。

## 头文件

```C++
#include <WinSock2.h>
#include <MSWSock.h>

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Mswsock.lib")
```




<div style="margin-top: 80px;">

---
</div>

## 相关数据结构
### OVERLAPPED
`OVERLAPPED`是 Windows 操作系统中的一个结构体，用于在异步 I/O 操作中传递参数和结果信息。它的定义如下：

```C++
typedef struct _OVERLAPPED {
    ULONG_PTR Internal;
    ULONG_PTR InternalHigh;
    union {
        struct {
            DWORD Offset;
            DWORD OffsetHigh;
        } DUMMYSTRUCTNAME;
        PVOID Pointer;
    } DUMMYUNIONNAME;
    HANDLE hEvent;
} OVERLAPPED, *LPOVERLAPPED;
```

- 成员:
    - `Internal`：用于传递操作的状态或错误码等信息。
    - `InternalHigh`：用于传递操作的附加信息。
    - `DUMMYUNIONNAME`：一个联合体，包含两个成员：
        - `Offset`和`OffsetHigh`：用于指定文件偏移量，通常在文件操作中使用。
        - `Pointer`：用于指定自定义的指针，可以传递额外的数据。
    - `hEvent`：用于指定事件句柄，可以通过事件对象实现同步或异步的通知机制。


<div style="margin-top: 80px;">

---
</div>

## 关键函数
### 前置知识
在函数原型或文档中，`[in]`、`[out]`和`[in, out]`这样的标记用于指示参数是如何被函数使用的。这些标记帮助开发者理解各个参数的作用和期望的数据流向，是API文档中常见的注释方式。下面是每个标记的具体含义：

- `[in]`：这个标记表示参数是输入参数。它被用来从调用者传递数据到函数内部。函数会读取这个参数的值，但不期望修改它。调用者应该在调用函数之前提供这个参数的值。

- `[out]`：这个标记表示参数是输出参数。函数会通过这个参数向调用者返回数据。调用者提供一个变量（或者变量的地址/引用），函数会修改这个变量的内容，以反映操作的结果或输出数据。在调用函数之前，该参数的初始值通常不重要，除非文档特别说明。

- `[in, out]`：这个标记表示参数既是输入又是输出参数。调用者提供初始值，并且函数可能会根据操作的需要读取并修改它。这意味着函数既会读取参数的初始值，也会更新它以反映新的状态或结果。

这些标记对于理解函数如何与其参数交互是非常有用的。例如，当你看到一个标记为[out]的参数时，你就知道需要查看这个参数在函数调用后的值，因为它很可能包含了一些重要的输出信息或函数执行的结果。相反，一个[in]标记的参数只需要在调用前被正确设置，因为函数只会读取它。

在编写代码时，正确处理这些不同类型的参数对于确保程序的正确性和避免潜在的错误非常重要。

<div style="margin-top: 80px;">

---
</div>

### CreateIoCompletionPort()
用于**创建一个新的IO完成端口** 或 **将I/O对象（如socket）与已有的完成端口关联**。

```C++
HANDLE CreateIoCompletionPort(
  [in]      HANDLE    FileHandle,
  [in, out] HANDLE    ExistingCompletionPort,
  [in]      ULONG_PTR CompletionKey,
  [in]      DWORD     NumberOfConcurrentThreads
);
```
- **参数**:
    - `FileHandle`：需要关联的I/O对象句柄，或者是`INVALID_HANDLE_VALUE`以**创建**新的完成端口。
    - `ExistingCompletionPort`：要`关联`的现有完成端口句柄，或者为`NULL`以**创建**新的完成端口。
    - `CompletionKey`：用户定义的值，与每个I/O操作一起返回，用于区分不同的I/O源。<sup>[1]</sup>
    - `NumberOfConcurrentThreads`：指定可以同时访问完成端口的最大线程数(是用户代码线程数)。传递0表示由系统决定。
        - **如何理解**：假设`NumberOfConcurrentThreads`被设置为4，这意味着在任何时刻，最多只有四个线程可以从完成端口获取IO完成事件进行处理。如果有更多的线程尝试从完成端口获取事件，那么超出这个限制的线程将会被阻塞，直到其中一个正在处理的线程完成其工作。这个机制有助于开发者控制应用程序的并发级别，以及管理线程的使用，从而优化程序性能和资源利用率。
        - **设置建议**：如果设置为0，系统会根据当前计算机上的逻辑处理器数量自动选择最优的线程数。这通常是推荐的做法，因为它可以让系统基于当前硬件配置来优化并发操作。

- **返回值**：
    - 如果函数调用`成功`，返回值是**一个指向新创建或已存在的IO完成端口的句柄**。
    - 如果函数`失败`，返回值为`NULL`。失败时，可以通过调用GetLastError函数获取更多错误信息。

注: `参数一` 与 `参数二` 是不能互换的

- 第一个参数是需要被关联的对象，而第二个参数决定了这个对象将被关联到哪个I/O完成端口上


**示例代码:**

```C++
// 创建完成端口
HANDLE sev_iocp = CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, NULL, 0);

// 关联完成端口
if (!CreateIoCompletionPort((HANDLE)server.getSerSocket(), sev_iocp, NULL, 0)) {
    HX::tools::HXprint::getHXprint()->ptr_pError("关联完成端口时出错!");
}
```

<div style="margin-top: 80px;">

---
</div>

### AcceptEx()
`AcceptEx()`是`Windows Sockets 2`的一个扩展函数，用于在服务器端异步地接受新的连接请求。它是专门设计来与IO完成端口（IOCP）模型一起使用的，允许应用程序高效地处理多个并发连接。与传统的`accept()`函数相比，`AcceptEx()`提供了更多的灵活性和性能优势，特别是在高性能网络服务器的开发中。

```C++
BOOL AcceptEx(
  [in]  SOCKET sListenSocket,         // 监听的socket <服务端套接字>
  [in]  SOCKET sAcceptSocket,         // 客户端套接字，必须是通过socket函数创建的，未绑定的套接字
  [out] PVOID lpOutputBuffer,         // 用来接收首批数据及存储两个sockaddr结构的缓冲区
  [in]  DWORD dwReceiveDataLength,    // 在首次接受的数据中期望读取的字节数，如果为0，则表示不接收数据
  [in]  DWORD dwLocalAddressLength,   // 本地地址sockaddr结构的大小，此值必须至少比正在使用的传输协议的最大地址长度多16个字节
  [in]  DWORD dwRemoteAddressLength,  // 远程地址sockaddr结构的大小，此值必须至少比正在使用的传输协议的最大地址长度多16个字节
  [out] LPDWORD lpdwBytesReceived,    // 实际接收到的字节数
  [in, out] LPOVERLAPPED lpOverlapped // 指向OVERLAPPED结构的指针，用于异步操作
);
```

- **参数**:
    - `sListenSocket`：这是引用监听套接字的句柄，它必须处于监听状态，等待客户端的连接请求。
    - `sAcceptSocket`：这是引用将要接受新连接的套接字的句柄。此套接字在调用`AcceptEx`之前必须已经**创建且和关联了完成端口**，但不应绑定到任何地址。
    - `lpOutputBuffer`：指向接收第一批传入数据以及两个套接字地址的缓冲区的指针。缓冲区的大小应至少为`dwReceiveDataLength + dwLocalAddressLength + dwRemoteAddressLength`。
    - `dwReceiveDataLength`：指定`lpOutputBuffer`中用于接收来自发送方的数据的部分的大小（字节）。
    - `dwLocalAddressLength`：指定`lpOutputBuffer`中用于存储本地套接字地址的部分的大小（字节）。
    - `dwRemoteAddressLength`：指定`lpOutputBuffer`中用于存储远程套接字地址的部分的大小（字节）。
    - `lpdwBytesReceived`：一个指针，用于接收在`lpOutputBuffer`中实际接收到的数据字节数。
    - `lpOverlapped`：指向一个`OVERLAPPED`结构的指针，该结构用于异步操作。在操作完成时，可以使用`GetQueuedCompletionStatus`来检索操作的结果。

- **返回值**:
    - 成功：如果函数成功地开始了一个异步操作来接受一个连接，则返回`TRUE`。
    - 失败：如果出现错误，则返回`FALSE`。可以通过调用`WSAGetLastError()`函数来获取更多错误信息。如果`WSAGetLastError()`返回`ERROR_IO_PENDING`，这表明异步操作已成功启动，并且完成将在未来通知。 

**示例代码:**

```C++
#define STR_MAX_SIZE 1024

// IOCP 事件类型
enum IocpEventType {
    ACCEPT, // 等待连接
    RECV,   // 接收
    SEND    // 发送
};

typedef struct HXData
{
    OVERLAPPED Overlapped;          // 重叠结构体必需要放到第一个位置
    SOCKET cli_socket;              // 客户端套接字
    CHAR str_data[STR_MAX_SIZE];    // 收/发 的数据
    WSABUF buf_data;                // 用于描述数据缓冲区
    IocpEventType eventType;        // 事件类型
    DWORD bufferCount;              // 接收的字节数
    DWORD lFlags;                   // 作用未知

public:
    // 构造函数
    HXData(SOCKET cli_socket, IocpEventType eventType) {
        memset(this, 0, sizeof(HXData));
        this->cli_socket = cli_socket;
        this->buf_data.buf = this->str_data;
        this->buf_data.len = sizeof(this->str_data);
        this->bufferCount = 0;
        this->lFlags = 0;
        this->eventType = eventType;
    }
} HXData;

// 等待连接
void accepHX(SOCKET ser_socket, HANDLE sev_iocp)
{
    // 准备一个未关联的, 用于连接客户端的套接字
    SOCKET cli_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    // 关联完成端口
    if (!CreateIoCompletionPort((HWND)cli_socket, sev_iocp, NULL, 0)) {
        HX::tools::HXprint::getHXprint()->ptr_pError("新建客户端套接字, 关联完成端口时出错!");
        return;
    }

    HXData* data = new HXData(cli_socket, ACCEPT);

    char szBuff[STR_MAX_SIZE] = { 0 }; // 单纯只是缓冲区 (void *)
    DWORD dwRecved = 0;

    AcceptEx(
        ser_socket,
        cli_socket,
        szBuff,
        0,
        sizeof(sockaddr) + 16,
        sizeof(sockaddr) + 16,
        &dwRecved,
        (LPOVERLAPPED)data
    );
}

// 投递一个等待连接的任务
accepHX(server.getSerSocket(), sev_iocp);
```


<div style="margin-top: 80px;">

---
</div>

### GetQueuedCompletionStatus()
从完成端口获取一个已完成的I/O操作。

```C++
BOOL GetQueuedCompletionStatus(
    [in]  HANDLE       CompletionPort,               // 完成端口的句柄
    [out] LPDWORD      lpNumberOfBytesTransferred,   // 指向一个变量的指针，该变量接收完成的I/O操作传输的字节数
    [out] PULONG_PTR   lpCompletionKey,              // 指向一个变量的指针，该变量接收与完成的I/O操作关联的完成键
    [out] LPOVERLAPPED *lpOverlapped,                // 指向一个变量的指针，该变量接收指向完成的I/O操作的OVERLAPPED结构的指针
    [in]  DWORD        dwMilliseconds                // 等待操作完成的超时时间，以毫秒为单位
);
```

- **参数**:
    - `CompletionPort`：完成端口的句柄。
    - `lpNumberOfBytesTransferred`：指向接收已传输字节计数的变量。
    - `lpCompletionKey`：接收与完成包关联的完成键。
    - `lpOverlapped`：接收指向`OVERLAPPED`结构的指针，该结构用于异步I/O操作。
    - `dwMilliseconds`：等待完成包的超时时间，单位为毫秒。
        - `INFINITE`：当`dwMilliseconds`参数设置为`INFINITE`时，函数将会无限期等待，直到至少有一个完成包可用。这意味着如果没有任何I/O操作完成，调用线程将会被一直阻塞。
        - `0`：虽然不是一个宏定义，但将`dwMilliseconds`参数设置为0也具有特殊含义。这表示函数将立即检查完成端口，看是否有任何完成包可用，如果没有，则立即返回，不会发生等待。

- **返回值**：
    - 如果函数调用成功且完成了一个I/O操作，返回值为`TRUE`。
    - 如果等待超时，返回值为`FALSE`，并且`*lpOverlapped`会被设置`为NULL`。
    - 如果函数失败或者完成的是一个特殊的完成包（通过`PostQueuedCompletionStatus`投递），返回值也可能是`FALSE`。在这种情况下，应该检查`lpOverlapped`参数来区分这些情况。可以通过`GetLastError`获取更多的错误信息。

**示例代码**:


```C++
// 处理 主循环
while (true)
{
    DWORD dwBytesTranfered = 0;
    ULONG_PTR uKey;
    LPOVERLAPPED data_mae = NULL;
    GetQueuedCompletionStatus(sev_iocp, &dwBytesTranfered, &uKey, &data_mae, INFINITE);

    HXData* data = (HXData*)data_mae;
    switch (data->eventType)
    {
    case ACCEPT:
        // 添加到接收数据任务
        data->eventType = RECV;
        recvHX(data);
        print->ptr_pInfo("客户端已连接!");
        // 再创建一个等待连接的任务
        accepHX(server.getSerSocket(), sev_iocp);
        break;
    case RECV:
        if (dwBytesTranfered == 0)
        {
            print->ptr_pInfo("客户端已断开连接!");
            // 取消对完成端口的关联
            CreateIoCompletionPort((HANDLE)data->cli_socket, NULL, 0, 0);

            closesocket(data->cli_socket);
            delete data;
            continue;
        }
        print->ptr_pInfo("收到消息: %s", data->str_data);
        recvHX(data);
        break;
    default:
        break;
    }
}
```


<div style="margin-top: 80px;">

---
</div>

### WSARecv()
`WSARecv`函数用于在套接字上异步接收数据。

```C++
int WSARecv(
  [in]  SOCKET                             s,                    // 目标套接字
  [in, out] LPWSABUF                       lpBuffers,            // 数据缓冲区数组
  [in]  DWORD                              dwBufferCount,        // 缓冲区数组中的元素数量
  [out] LPDWORD                            lpNumberOfBytesRecvd, // 实际接收的字节数
  [in, out] LPDWORD                        lpFlags,              // 控制接收行为的标志
  [in, out] LPWSAOVERLAPPED                lpOverlapped,         // 指向OVERLAPPED结构的指针，用于异步操作
  [in]  LPWSAOVERLAPPED_COMPLETION_ROUTINE lpCompletionRoutine   // 完成例程的回调函数，当I/O操作完成时被调用
);
```

- **参数**:
    - `s`：指定要接收数据的套接字。
    - `lpBuffers`：指向一个`WSABUF`结构数组的指针，用于指定接收缓冲区的位置和大小。
    - `dwBufferCount`：指定`lpBuffers`数组中的元素数量。
    - `lpNumberOfBytesRecvd`：指向一个`DWORD`变量的指针，用于返回实际接收到的字节数。
    - `lpFlags`：指向一个`DWORD`变量的指针，用于指定接收操作的标志。可以设置为`NULL`。
|值|含义|
|:-|:-|
|MSG_PEEK|窥视传入的数据。数据被复制到缓冲区中，但不会从输入队列中删除。|
|MSG_OOB|处理OOB数据|
|MSG_PARTIAL|此次接收到的数据是客户端发来的一部分，接下来接收下一部分|
|MSG_PUSH_IMMEDIATE|通知传送尽快完成|
|MSG_WAITALL|呼叫者提供的缓冲区已满或连接已关闭或请求已取消或发生错误才把数据发送出去|
      
      ```GPT-3.5
      尽管在大多数情况下flags设置为0就足够了，但在某些特定场景下，你可能会需要设置特定的flags来满足特殊需求，例如：
      
      MSG_PEEK：查看接收缓冲区的数据而不实际从缓冲区中移除它们。这可以用于预览数据。
      MSG_OOB：接收带外数据。这是一种特殊用途的标志，用于处理TCP的紧急数据。
      MSG_PARTIAL：允许接收部分消息，这在处理大型数据传输时可能有用。
      ```
    - `lpOverlapped`：指向一个`WSAOVERLAPPED`结构的指针，用于指定异步操作的参数。这个参数必须指向之前使用`AcceptEx`或`WSARecvFrom`等函数创建的`WSAOVERLAPPED`结构。
    - `lpCompletionRoutine`：指向一个回调函数的指针，用于在异步操作完成时进行回调。可以设置为`NULL`。

- **返回值**:
    - 如果函数调用成功，返回值为`0`。
    - 如果函数调用失败，返回值为`SOCKET_ERROR`，并且可以通过调用`WSAGetLastError`函数获取具体的错误代码。
        - 需要注意的是，对于异步操作来说，返回值为0只表示函数**调用成功**，不代表**数据已经接收完毕**。实际接收到的字节数可以通过`lpNumberOfBytesRecvd`参数传出。 

**示例代码**:

```C++
void recvHX(HXData *data)
{
    if (WSARecv(data->cli_socket, &data->buf_data, 1, &data->bufferCount, &data->lFlags, (LPOVERLAPPED)data, NULL)) {
        if (WSAGetLastError() != WSA_IO_PENDING)
            HX::tools::HXprint::getHXprint()->ptr_pError("创建接收任务时出现错误");
    }
}
```


<div style="margin-top: 80px;">

---
</div>


### WSASend()
`WSASend`函数是Windows套接字API中的一部分，用于向指定的套接字发送数据。

```C++
int WSASend(
  SOCKET                             s,
  LPWSABUF                           lpBuffers,
  DWORD                              dwBufferCount,
  LPDWORD                            lpNumberOfBytesSent,
  DWORD                              dwFlags,
  LPWSAOVERLAPPED                    lpOverlapped,
  LPWSAOVERLAPPED_COMPLETION_ROUTINE lpCompletionRoutine
);
```

- **参数**:
    - `s`：指定要发送数据的套接字。
    - `lpBuffers`：指向一个`WSABUF`结构数组的指针，每个`WSABUF`结构描述要发送的数据缓冲区信息。
    - `dwBufferCount`：指定`lpBuffers`数组中的元素数目。
    - `lpNumberOfBytesSent`：如果函数调用成功，这个参数将返回实际发送的字节数。
    - `dwFlags`：指定一些标志来控制发送操作的行为，可以是`0`或者一些特定的标志，例如`MSG_OOB`（发送紧急数据）或`MSG_DONTROUTE`（不经过路由）等。
    - `lpOverlapped`：指向一个`WSAOVERLAPPED`结构的指针，用于支持异步操作。如果使用同步方式发送数据，可以设置为`NULL`。
    - `lpCompletionRoutine`：指向一个回调函数的指针，用于在异步操作完成后进行回调。如果不需要回调，可以设置为`NULL`。 

- **返回值**:
    - 如果函数调用成功，返回`0`。
    - 如果函数调用失败，返回`SOCKET_ERROR`，并可以通过`WSAGetLastError`函数获取错误代码。
 
- 注意事项：

    - 在使用`WSASend`进行异步发送时，需要使用`WSAOVERLAPPED`结构来指定异步操作的相关参数，并且需要为每个异步发送操作分配一个独立的`WSAOVERLAPPED`结构。在操作完成后，可以通过`GetQueuedCompletionStatus`函数或者`GetOverlappedResult`函数获取发送结果。

    - 如果使用同步方式发送数据，可以将`lpOverlapped`参数设置为`NULL`，此时函数将**阻塞直到发送操作完成**。

    - 在使用异步发送时，**需要确保发送缓冲区数据的有效性，以及在发送操作完成之前保持缓冲区的有效性**(不能释放, 或者修改)。


<div style="margin-top: 80px;">

---
</div>

### PostQueuedCompletionStatus()
手动向完成端口投递一个完成包，常用于通知工作线程执行特定任务或停止执行。

```C++
BOOL PostQueuedCompletionStatus(
    HANDLE       CompletionPort,
    DWORD        dwNumberOfBytesTransferred,
    ULONG_PTR    dwCompletionKey,
    LPOVERLAPPED lpOverlapped
);
```

- **参数**:
    - `CompletionPort`：完成端口的句柄。
    - `dwNumberOfBytesTransferred`：自定义的字节数，可用于传递信息。
    - `dwCompletionKey`：自定义的完成键，可用于区分不同的操作或I/O源。
    - `lpOverlapped`：指向`OVERLAPPED`结构的指针，可用于传递额外信息 
- **返回值:**
    - 如果函数调用成功，返回值为`TRUE`。
    - 如果函数调用失败，返回值为`FALSE`。失败时，可以通过调用`GetLastError`函数获取更多错误信息。

<div style="margin-top: 80px;">

---
</div>

## 注解
### 参考链接
1. 感谢GPT-3.5
2. [c++——iocp模型](https://blog.csdn.net/www_dong/article/details/125667928)
3. 拓展阅读: [采用完成端口（IOCP）实现高性能网络服务器（Windows c++版）](https://www.cnblogs.com/yuanchenhui/p/iocp_windows.html)

### [1] CompletionKey 详解
`CompletionKey`参数在`CreateIoCompletionPort`函数中起着重要的角色，它为开发者提供了一种将特定的用户定义数据与IO完成端口（IOCP）上的每个I/O操作关联的机制。这使得在处理完成通知时，能够更容易地识别和区分来自不同源的I/O操作。

#### 如何理解CompletionKey

当你使用`CreateIoCompletionPort`函数将一个I/O对象（例如，一个文件句柄或socket）关联到完成端口时，你可以为这个操作指定一个`ULONG_PTR`类型的`CompletionKey`。这个键值随后会与所有从该I/O对象产生的I/O完成通知一起返回给处理这些通知的线程。

这意味着，当工作线程调用`GetQueuedCompletionStatus`函数从IOCP获取一个I/O完成通知时，除了得到关于I/O操作本身的信息（如传输的字节数和指向OVERLAPPED结构的指针）之外，还会收到与该I/O操作关联的`CompletionKey`。通过这个`CompletionKey`，工作线程可以确定是哪个I/O对象产生了这个完成通知，进而根据不同的`CompletionKey`执行相应的处理逻辑。

#### 使用场景示例
假设你正在开发一个网络服务器应用程序，该程序同时监听多个不同的端口。每个监听的socket都可以被关联到同一个IOCP上，但是你可以为每个socket指定不同的`CompletionKey`。当一个I/O操作完成时，通过检查返回的`CompletionKey`，你的应用程序可以立即知道是哪个socket的I/O操作完成了，从而快速准确地处理该事件。

#### 优势
使用`CompletionKey`的主要优势在于其提供了一种简单而高效的方式来管理和区分大量的I/O源。这对于构建高性能的并发网络应用程序和文件系统应用程序尤为重要，因为它允许开发者以统一和有序的方式处理各种不同来源的I/O完成通知。

总之，`CompletionKey`是IOCP提供的一个强大功能，通过允许开发者将用户定义的数据与特定的I/O操作关联，它极大地增强了IOCP的灵活性和应用程序的可管理性。希望这个解释能帮助你更好地理解`CompletionKey`的概念和用法。