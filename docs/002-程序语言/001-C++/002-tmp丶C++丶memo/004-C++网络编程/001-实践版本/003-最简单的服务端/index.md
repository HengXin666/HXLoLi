# 最简单的服务端
Cygwin 安装教程:

[Cygwin安装教程](https://blog.csdn.net/weixin_44778232/article/details/127579150)

server.cpp
```C++
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
using std::string;

int main()
{
    // 1. 创建 socket
    // 创建一个套接字，指定协议族为AF_INET（IPv4网络协议），类型为SOCK_STREAM（面向连接的TCP套接字），并指定协议为IPPROTO_TCP。
    int sockfd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    
    if (sockfd < 0)
    {
        printf("create socket error: errno=%d errmsg=%s\n", errno, strerror(errno));
        return 1;
    }
    else
    {
        printf("create socket success!\n");
    }

    // 2. 绑定 socket
    // 定义服务器监听的IP地址和端口号
    string ip = "127.0.0.1";    // IP地址，这里使用本地回环地址，表示监听本机
    int port = 8080;            // 端口号，可以自定义，但要确保没有被其他程序占用

    // 创建一个 sockaddr_in 结构体对象，并进行初始化
    struct sockaddr_in sockaddr;
    std::memset(&sockaddr, 0, sizeof(sockaddr));        // 初始化结构体 先将结构体对象填充为0
    sockaddr.sin_family = AF_INET;                      // 设置地址族为IPv4
    sockaddr.sin_addr.s_addr = inet_addr(ip.c_str());   // 将字符串形式的IP地址转换为网络字节序的二进制IP地址(统一为大端)
    sockaddr.sin_port = htons(port);                    // 将主机字节序的端口号转换为网络字节序
    
    if (::bind(sockfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) // 此处的类型转换是有说法的！^[1]
    {
        printf("socket bind error: errno=%d, errmsg=%s\n", errno, strerror(errno));
        return 1;
    }
    else
    {
        printf("socket bind success: ip=%s port=%d\n", ip.c_str(), port);
    }

    // 3. 监听 socket
    if (::listen(sockfd, 1024) < 0) // 将套接字和一个最大连接数（这里设置为1024）
    {
        printf("socket listen error: errno=%d errmsg=%s\n", errno, strerror(errno));
        return 1;
    }
    else
    {
        printf("socket listen ...\n");
    }

    while (true)
    {
        // 4. 接收客户端连接
        int connfd = ::accept(sockfd, nullptr, nullptr);
        if (connfd < 0)
        {
            printf("socket accept error: errno=%d errmsg=%s\n", errno, strerror(errno));
            return 1;
        }

        char buf[1024] = {0};

        // 5. 接收客户端的数据
        size_t len = ::recv(connfd, buf, sizeof(buf), 0);
        printf("recv: conn=%d msg=%s\n", connfd, buf);

        // 6. 向客服端发送数据
        ::send(connfd, buf, len, 0);
    }

    // 7. 关闭 socket
    ::close(sockfd);
    return 0;
}
```

这段代码是一个简单的TCP服务器，它通过socket API在本地IP地址127.0.0.1和端口8080上监听来自客户端的连接。下面是代码的详细解释：

1. 创建`socket`：使用`socket`函数创建一个套接字，指定协议族为`AF_INET`（IPv4网络协议），类型为`SOCK_STREAM`（面向连接的TCP套接字），并指定协议为`IPPROTO_TCP`(TCP/IP)。

2. 绑定`socket`：使用`bind`函数将套接字绑定到特定的IP地址和端口。首先定义一个`sockaddr_in`结构体对象，并将其成员变量填充，包括地址族、IP地址、端口等信息。然后调用`bind`函数将套接字与`sockaddr_in`对象绑定在一起。

3. 监听`socket`：使用`listen`函数将套接字置于监听状态，等待客户端的连接请求。将套接字和一个最大连接数（这里设置为1024）作为参数传递给listen函数。

4. 接收客户端连接：使用`accept`函数接受客户端的连接请求，并返回一个新的套接字描述符，用于与客户端进行通信。在这个示例中，我们只接受连接，但不处理连接的客户端信息。

5. 接收客户端数据：使用`recv`函数从客户端接收数据。将接收到的数据存储在名为buf的字符数组中，并返回接收到的字节数。

6. 向客户端发送数据：使用`send`函数将接收到的数据原样发送回客户端。将套接字描述符、数据缓冲区和数据长度作为参数传递给send函数。

7. 关闭`socket`：使用`close`函数关闭服务器的套接字，释放资源。

注意: 如果使用Clion的话, 可能需要 使用任务管理器来解除占用!

(查询被占用的端口)
```cmd
netstat -ano | findstr :8080
```

## 注解:
### [1]

` if (::bind(sockfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) 为什么要转换类型?`

这里需要将`sockaddr_in`类型的结构体指针转换为通用的`sockaddr`类型的结构体指针，是因为`bind`函数的第二个参数要求传入一个指向`sockaddr`类型结构体的指针。而`sockaddr_in`结构体是基于`sockaddr`结构体的扩展，所以可以通过将其强制类型转换为`sockaddr`类型的结构体指针来满足`bind`函数的要求。

实际上，C语言和C++语言中都有类似的问题。在C语言中，如果我们定义了一个派生结构体，它的第一个成员变量必须是与基类结构体完全相同的结构体，否则就会出现严重的问题。在C++语言中，这个问题得到了很好的解决，即使用虚基类继承，使得派生类中只包含一份基类的数据成员，从而避免了重复的数据拷贝问题。但在C语言中，由于缺少虚基类继承的机制，我们只能通过强制类型转换来解决这个问题。

例如以下代码:
```C
#include <stdio.h>

int main(void) {
    struct otosan // 父类结构体
    {
        int a;
    };

    struct kodomoo // 子类结构体
    {
        int a;
        int b;
    };
    
    struct kodomoo kdm = {1, 1};

    printf("%d\n", ((struct otosan *)&kdm)->a);

    return 0;
}
```