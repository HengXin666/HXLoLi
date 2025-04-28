# C/C++网络编程详解（Windows版）

## 头文件和库

```C++
#include <WinSock2.h>
#pragma comment(lib,"ws2_32.lib")

#define _WINSOCK_DEPRECATED_NO_WARNINGS
```
---

## WSAStartup() 和 WSACleanup()
`WSAStartup()`是 Windows 套接字 API （Winsock）中的一个函数，用于初始化套接字库。

它的原型如下:

```C++
int WSAStartup(
  WORD      wVersionRequested,
  LPWSADATA lpWSAData
);
```

- 参数说明：
    - `wVersionRequested`：用来指定准备加载动态库的版本号，高字节为库文件的副版本，低字节指定主版本，`MAKEWORD(X,Y)`宏用于生成该参数，其中X为高字节，Y为低字节，通常使用`MAKEWORD(2, 2)`表示请求版本为 2.2。
    - `lpWSAData`：指向一个 WSADATA 结构的指针，用于接收套接字库的信息。
 
    ```C++
    typedef struct WSAData {
        WORD           wVersion;      // 期望调用者使用的socket版本（或实际返回的socket版本，可根据此参数判断返回的版本号是否正确，可通过HIBYTE宏取得高字节，LOBYTE宏取得低字节）
        WORD           wHighVersion;  // 本机Dll支持的最高版本
        unsigned short iMaxSockets;   // 一个进程最多可以打开的套接字数量（2.0版本后忽略此值）
        unsigned short iMaxUdpDg;     // 一个进程发送或接收的最大数据报长度
        char FAR *     lpVendorInfo;  // 厂商专有信息（2.0版本后忽略此值）
        char           szDescription[WSADESCRIPTION_LEN+1]; // DLL的描述信息
        char           szSystemStatus[WSASYS_STATUS_LEN+1]; // DLL的状态信息
    } WSADATA, *LPWSADATA;
    ```

- 返回值：
    - 如果函数调用成功，返回值为 0。
    - 如果函数调用失败，返回值为非零错误代码，可以使用`WSAGetLastError()`函数获取具体的错误代码。
- 函数功能：
    - WSAStartup() 函数必须在使用 Winsock 库中的任何其他套接字函数之前调用。
    - 它将初始化 Winsock 库，并指定了使用的版本。
    - 在成功调用 WSAStartup() 后，可以使用其他套接字函数来创建和操作套接字。

`WSACleanup()`是用于清理套接字库。

示例代码:

```C++
#include <iostream>
#include <Winsock2.h>

#pragma comment(lib, "ws2_32.lib")

int main()
{
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "Failed to initialize winsock" << std::endl;
        return -1;
    }

    // 在这里可以使用其他套接字函数进行网络编程
    // ...

    WSACleanup(); // 无任何参数，直接调用即可

    return 0;
}
```
---

## socket()

```C++
SOCKET socket(
    int af,      //地址类型，常用IPv4地址：AF_INET，和IPv6地址：AF_INET6
    int type,    //套接字类型，常用TCP协议：SOCK_STREAM, UDP协议：SOCK_DGRAM
    int protocol //协议类型，一般填 0，自动选择即可
);
// 返回值，INVALID_SOCKET失败，该宏实则定义为 -1, 并设置 errno 来指示错误类型
// 成功则返回 其文件描述符
```

---

## WSASocket()

```C++
SOCKET WSAAPI WSASocket(
  int                 af,
  int                 type,
  int                 protocol,
  LPWSAPROTOCOL_INFO  lpProtocolInfo,
  GROUP               g,
  DWORD               dwFlags
);
```

**参数**:
- `af (int)`: 指定地址族（Address Family）。常见的地址族有`AF_INET`用于`IPv4`网络协议，`AF_INET6`用于`IPv6`网络协议。此参数决定了套接字使用的网络协议类型。

- `type (int)`: 指定套接字类型。常见的类型有`SOCK_STREAM`用于流式套接字（TCP连接），`SOCK_DGRAM`用于数据报套接字（UDP连接）。

- `protocol (int)`: 特定地址族和套接字类型下，用于指定协议类型。通常，当af和type参数已经决定了协议后，这个参数可以设置为0，由系统自动选择合适的协议。例如，`IPPROTO_TCP`用于TCP协议，`IPPROTO_UDP`用于UDP协议。

- `lpProtocolInfo (LPWSAPROTOCOL_INFO)`: 指向WSAPROTOCOL_INFO结构的指针，该结构描述了要创建的套接字的详细信息。如果只是想使用默认配置，可以将此参数设为NULL。

- `g (GROUP)`: 指定套接字组的标识符。如果不使用套接字组，这个参数应该设置为0。

- `dwFlags (DWORD)`: 用于指定套接字的属性和行为的标志位。常用的标志包括`WSA_FLAG_OVERLAPPED`用于指定套接字支持重叠I/O操作，`WSA_FLAG_MULTIPOINT_C_ROOT`和`WSA_FLAG_MULTIPOINT_C_LEAF`用于多点通信。

**返回值**:
- 成功时返回套接字的句柄。
- 失败时返回`INVALID_SOCKET`。可以通过调用`WSAGetLastError`函数获取错误代码。

---
## bind()

```C++
int bind( 
    SOCKET s,        //创建的socket
    sockaddr * name, //包含地址和端口的结构体
    int namelen      //sockaddr 结构长度
);
// 返回值：返回SOCKET_ERROR失败，该宏被定义为-1，否则成功，返回值为0
```

使用示例:
```C++
#define _WINSOCK_DEPRECATED_NO_WARNINGS // vs环境下必须定义，否则无法使用inet_addr函数

sockaddr_in addr;
addr.sin_family = AF_INET;   // 地址为IPv4协议
addr.sin_port = htons(9999); // 端口为 9999
addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1"); // 具体绑定本机的地址

if (bind(sev, (sockaddr*)&addr, sizeof(addr)/*绑定*/ == -1)) {
    printf("绑定地址端口失败");
    return -1;
}
```
---
## listen()

```C++
int listen(
    SOCKET s,   // 要监听的socket
    int backlog // 等待连接的最大队列长度
);
// 返回值：返回SOCKET_ERROR失败，该宏被定义为-1，否则成功，返回值为0
```
---
## accept() 接收客户端连接

```C++
SOCKET accept(
    SOCKET s,       // 接收的socket
    sockaddr* addr, // 接收到客户端的地址信息
    int * addrlen   // 地址信息长度
);
// 返回值：返回INVALID_SOCKET失败，该宏定义为-1，
// 否则成功返回客户端的套接字，可进行发送和接收消息
```
---

## connect() 连接服务端

```C++
SOCKET accept(
    SOCKET s,       // 接收的socket
    sockaddr* addr, // 目标服务端的地址信息
    int * addrlen   // 地址信息长度
);
// 返回值：返回INVALID_SOCKET失败，该宏定义为-1，
// 否则成功返回客户端的套接字，可进行发送和接收消息
```

示例:

```C++
SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);
sockaddr_in addr;
addr.sin_family = AF_INET;
addr.sin_port = htons(1234);
addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
if (connect(sock, (sockaddr*)&addr, sizeof(addr)) == -1) {
    printf("连接服务器失败!\n");
    return -1;
}
else {
    printf("成功连接服务器!\n");
}
```


---

## send() 发送

```C++
int send(
    SOCKET s,
    char * buf, // 要发送的内容
    int len,    // 内容长度
    int flags   // 一般为0，拷贝到程序中就立即删除内核中的数据,
                // 或MSG_DONTROUTE: 要求传输层不要将数据路由出去，
                // MSG_OOB: 标志数据应该被带外发送
);
// 返回值：-1（或宏SOCKET_ERROR）表示发送失败，否则返回发送成功的字节数
```
---

## recv() 读取

```C++
int recv(
    SOCKET s,   // 套接字
    char * buf, // 接受数据的缓存区
    int len,    // 缓存区大小
    int flags   // 标志，一般填0，将消息拷贝到应用程序中，将内核中的数据删除，
                // 还可以填MSG_PEEK: 只取数据，不从内核中删除数据，
                // MSG_OOB: 处理带外数据
);
// 返回值：小于等于0都表示出错，大于0则表示接收成功的数据大小
```
---

## closesocket()

```C++
int closesocket(
    SOCKET s //要关闭的socket
);
```

该函数就是关闭不用的socket，释放资源.

- 如果 closesocket() 函数成功关闭了套接字，返回值为 0。
- 如果 closesocket() 函数调用失败，返回值为 SOCKET_ERROR（通常定义为 -1）。可以通过调用 WSAGetLastError() 函数获取更具体的错误码。

## gethostbyname() 通过域名获取ip地址
示例:
```C++
//获取主机ip
HOSTENT* host = gethostbyname("www.baidu.com"); //如获取网站IP地址，参数填写域名即可，不需加"http://"
if (host == NULL)
{
    return false;
}
//转化为char*并拷贝返回
cout << inet_ntoa(*(in_addr*)*host->h_addr_list);
```


## 注解
### [1] 查看链接
[C/C++网络编程详解（Windows版）](https://blog.csdn.net/weixin_50964512/article/details/123743421)