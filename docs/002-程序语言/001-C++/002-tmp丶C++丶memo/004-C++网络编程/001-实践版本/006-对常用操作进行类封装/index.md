# 类封装

## 头文件
```C++
//
// Created by Heng_Xin on 2024/1/13.
//
#ifndef C_WI_FI_SERVER_H
#define C_WI_FI_SERVER_H

#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>

using std::string;

namespace hx
{
    namespace server
    {
        class HXSockfd
        {
        public:
            HXSockfd();                               // 创建一个套接字
            HXSockfd(int connfd);                     // 创建一个
            ~HXSockfd();                              // 关闭服务器(析构函数)
            void close();                             // 关闭服务器

            bool connect(const string &ip, int port); // 客户端: 连接服务器
            bool bind(const string &ip, int port);    // 创建服务器
            bool listen(int number);                  // 监视服务器

            int accept();                             // 用于服务端接受客户端连接请求, 并创建一个新的连接套接字(返回值)来处理该连接
            ssize_t recv(char *buff, size_t len);
            ssize_t send(const char *buff, size_t len);

            // ---设置属性---
            bool set_non_blocking();                    // 设置为 非阻塞 IO
            bool set_send_buffer(int size);             // 设置发送缓冲区
            bool set_recv_buffer(int size);             // 设置接收缓冲区
            bool set_linger(bool active, int seconds);  // 设置函数close()关闭TCP连接时的行为
            bool set_keepalive();                       // 自动在规定时间内向对方发送心跳包, 另一方在收到心跳包后就会自动回复, 以告诉对方我仍然在线
            bool set_reuseaddr();                       // 允许服务器绑定一个地址，即使这个地址当前已经存在已建立的连接
        private:
            string s_ip;  // ip地址
            int s_port;   // 端口
            int s_sockfd; // 套接字
        };
    };
};
#endif //C_WI_FI_SERVER_H
```

.cpp

```C++
//
// Created by Heng_Xin on 2024/1/13.
//

#include "./HXsocket.h"

using namespace hx::server;

HXSockfd::HXSockfd() : s_ip(""), s_port(0), s_sockfd(0)
{
    this->s_sockfd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (this->s_sockfd < 0) {
        printf("ERROR: socket fun error: errno = %d, errmsg = %s\n", errno, strerror(errno));
        return;
    }
    printf("INFO: socket fun ok!\n");
}

HXSockfd::HXSockfd(int connfd) : s_ip(""), s_port(0), s_sockfd(connfd)
{

}

HXSockfd::~HXSockfd()
{
    if (this->s_sockfd)
    {
        ::close(this->s_sockfd);
        printf("INFO: Free Server!\n");
    }
}

bool HXSockfd::bind(const string &ip, int port)
{
    this->s_ip = ip;
    this->s_port = port;

    struct sockaddr_in sockaddr;
    std::memset(&sockaddr, 0, sizeof(sockaddr));        // 初始化结构体 先将结构体对象填充为0
    sockaddr.sin_family = AF_INET;                      // 设置地址族为IPv4
    sockaddr.sin_addr.s_addr = inet_addr(ip.c_str());   // 将字符串形式的IP地址转换为网络字节序的二进制IP地址
    sockaddr.sin_port = htons(port);                    // 将主机字节序的端口号转换为网络字节序

    if (::bind(this->s_sockfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0)
    {
        printf("ERROR: socket bind error: errno=%d, errmsg=%s\n", errno, strerror(errno));
        return 0;
    }
    else
    {
        printf("INOF: socket bind success: ip = %s port = %d\n", ip.c_str(), port);
        return 1;
    }
}

bool HXSockfd::connect(const string &ip, int port)
{
    this->s_ip = ip;
    this->s_port = port;

    struct sockaddr_in sockaddr;
    std::memset(&sockaddr, 0, sizeof(sockaddr));        // 初始化结构体 先将结构体对象填充为0
    sockaddr.sin_family = AF_INET;                      // 设置地址族为IPv4
    sockaddr.sin_addr.s_addr = inet_addr(ip.c_str());   // 将字符串形式的IP地址转换为网络字节序的二进制IP地址
    sockaddr.sin_port = htons(port);                    // 将主机字节序的端口号转换为网络字节序

    if (::connect(this->s_sockfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0)
    {
        printf("ERROR: socket connect error: errno=%d, errmsg=%s\n", errno, strerror(errno));
        return 0;
    }
    else
    {
        printf("INOF: socket connect success: ip = %s port = %d\n", ip.c_str(), port);
        return 1;
    }
}

bool HXSockfd::listen(int number)
{
    if (::listen(this->s_sockfd, number) < 0)
    {
        printf("ERROR: socket listen error: errno=%d errmsg=%s\n", errno, strerror(errno));
        return 0;
    }
    else
    {
        printf("INOF: socket listen ...\n");
        return 1;
    }
}

int HXSockfd::accept()
{
    return ::accept(this->s_sockfd, nullptr, nullptr);
}

ssize_t HXSockfd::recv(char *buff, size_t len)
{
    return ::recv(this->s_sockfd, buff, len, 0);
}

ssize_t HXSockfd::send(const char *buff, size_t len)
{
    return ::send(this->s_sockfd, buff, len, 0);
}

void HXSockfd::close()
{
    if (this->s_sockfd)
    {
        ::close(this->s_sockfd);
        printf("INFO: Free Server!\n");
        this->s_sockfd = 0;
    }
}

bool HXSockfd::set_non_blocking()
{
    int flags = fcntl(this->s_sockfd, F_GETFL, 0); // 获取属性
    if (flags < 0)
    {
        printf("ERROR: socket set_non_blocking error: errno = %d, errmsg = %s", errno, strerror(errno));
        return false;
    }

    flags |= O_NONBLOCK; // 添加一个属性
    if (fcntl(this->s_sockfd, F_SETFL, flags) < 0)
    {
        printf("ERROR: socket fcntl error: errno = %d, errmsg = %s", errno, strerror(errno));
        return false;
    }
    return true;
}

bool HXSockfd::set_send_buffer(int size)
{
    int buff_size = size;
    if (setsockopt(this->s_sockfd, SOL_SOCKET, SO_SNDBUF, &buff_size, sizeof(buff_size)) < 0)
    {
        printf("ERROR: socket set_send_buffer error: errno = %d, errmsg = %s", errno, strerror(errno));
        return false;
    }
    return true;
}

bool HXSockfd::set_recv_buffer(int size)
{
    int buff_size = size;
    if (setsockopt(this->s_sockfd, SOL_SOCKET, SO_RCVBUF, &buff_size, sizeof(buff_size)) < 0)
    {
        printf("ERROR: socket set_recv_buffer error: errno = %d, errmsg = %s", errno, strerror(errno));
        return false;
    }
    return true;
}

bool HXSockfd::set_linger(bool active, int seconds)
{
    struct linger l;
    std::memset(&l, 0, sizeof(l));
    l.l_onoff = active ? 1 : 0;
    l.l_linger = seconds;
    if (setsockopt(this->s_sockfd, SOL_SOCKET, SO_LINGER, &l, sizeof(l)) < 0)
    {
        printf("ERROR: socket set_linger error: errno = %d, errmsg = %s", errno, strerror(errno));
        return false;
    }
    return true;
}

bool HXSockfd::set_keepalive()
{
    int flag = 1;
    if (setsockopt(this->s_sockfd, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag)) < 0)
    {
        printf("ERROR: socket set_keepalive error: errno = %d, errmsg = %s", errno, strerror(errno));
        return false;
    }
    return true;
}

bool HXSockfd::set_reuseaddr()
{
    int flag = 1;
    if (setsockopt(this->s_sockfd, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag)) < 0)
    {
        printf("ERROR: socket set_keepalive error: errno = %d, errmsg = %s", errno, strerror(errno));
        return false;
    }
    return true;
}
```

## 客户端代码 & 服务端代码
### 客户端
```C++
//
// Created by Heng_Xin on 2024/1/13.
//
#include <iostream>
#include <stdio.h>
#include "./HX/HXsocket.h"

using namespace hx::server;
using std::string;

int main()
{
    // 1. 创建 socket
    HXSockfd server{};

    // 2. 连接服务端

    server.connect("127.0.0.1", 8080);

    // 3. 向服务端发送数据

    string data = "hello world";
    server.send(data.c_str(), data.size());
  
    // 4. 接收服务端的数据
    char buf[1024] = {0};
    server.recv(buf, sizeof(buf));

    printf("已发送! 收到回复: %s\n", buf);

    // 5. 关闭 socket
    server.close();

    return 0;
}
```

### 服务端

```C++
#include <iostream>

#include <cstring>
#include <string>

#include "./HX/HXsocket.h"

using std::string;
using namespace hx::server;
// cmd :
// netstat -ano | findstr :8080

int main()
{
    // 1. 创建 socket

    // 创建一个套接字，指定协议族为AF_INET（IPv4网络协议），类型为SOCK_STREAM（面向连接的TCP套接字），并指定协议为IPPROTO_TCP。
    HXSockfd server{};


    // 2. 绑定 socket
    // 定义服务器监听的IP地址和端口号
    server.bind("127.0.0.1", 8080);
    // 创建一个 sockaddr_in 结构体对象，并进行初始化


    // 3. 监听 socket
    server.listen(1024);

    while (true)
    {
        // 4. 接收客户端连接
        int connfd = server.accept();
        if (connfd < 0)
        {
            printf("ERROE: accept error!\n");
            return -1;
        }

        HXSockfd client(connfd);
        char buf[1024] = {0};

        // 5. 接收客户端的数据
        size_t len = client.recv(buf, sizeof(buf));
        printf("recv: conn=%d msg=%s\n", connfd, buf);

        // 6. 向客服端发送数据
        client.send(buf, len);

        // #. 注意: 上面的写法是接受到一条消息并且回复后就断开连接了! (client被析构了)
    }

    // 7. 关闭 socket
    server.close();
    return 0;
}
```