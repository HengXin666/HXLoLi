# socket函数详解
## socket() 创建一个套接字

```C++
/**
 * @brief 创建一个套接字
 *
 * 此函数用于创建一个新的套接字。
 *
 * @param domain 协议族/域，如 AF_INET（IPv4）、AF_INET6（IPv6）、AF_UNIX（Unix 域）等
 * @param type 套接字类型，如 SOCK_STREAM（面向连接的流套接字）、SOCK_DGRAM（无连接的数据报套接字）等
 * @param protocol 协议，通常为 0，表示使用默认协议
 * @return 成功时返回套接字描述符，失败时返回 -1
 */
int socket(int domain, int type, int protocol);
```

## bind() 绑定地址到套接字


```C++
/**
 * @brief 绑定地址到套接字
 *
 * 此函数用于将指定的地址绑定到已创建的套接字上。
 *
 * @param sockfd 套接字描述符
 * @param addr 地址结构体指针，包含要绑定的 IP 地址和端口号
 * @param addrlen 地址结构体大小
 * @return 成功时返回 0，失败时返回 -1
 */
int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen);
```

## listen() 监听连接请求


```C++
/**
 * @brief 监听连接请求
 *
 * 此函数用于将套接字设置为监听状态，等待客户端连接请求。
 *
 * @param sockfd 套接字描述符
 * @param backlog 允许的等待连接队列长度
 * @return 成功时返回 0，失败时返回 -1
 */
int listen(int sockfd, int backlog);
```


## accept 服务端: 连接客户端

```C++
/**
 * @brief 接受客户端连接请求
 *
 * 此函数用于接受客户端的连接请求，并创建一个新的套接字与客户端建立连接。
 *
 * @param sockfd 监听套接字描述符
 * @param addr 客户端地址结构体指针
 * @param addrlen 客户端地址结构体大小指针
 * @return 成功时返回新的套接字描述符，失败时返回 -1
 *
 * @note 调用此函数会阻塞，直到有客户端连接请求到达
 */
int accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen);
```

函数说明：

- `accept`: 函数用于接受客户端的连接请求，并创建一个新的套接字与客户端建立连接。
- `sockfd`: 参数是已经监听的套接字描述符，即服务器套接字的文件描述符。
- `addr`: 参数是一个指向存储客户端地址信息的结构体 `sockaddr` 的指针，如果不需要获取客户端地址信息，可以传入 NULL。
- `addrlen`: 参数是一个指向表示 addr 结构体大小的整数指针，如果不需要获取客户端地址信息，可以传入 NULL。
- `accept`: 函数返回一个新的套接字描述符，可以通过该描述符与客户端进行数据交互。

返回值：

- 如果连接成功，`accept` 函数返回一个新的套接字描述符，该描述符用于与客户端进行数据交互。
- 如果连接失败，`accept` 函数返回 -1，并设置 errno 为相应的错误码。

注意事项：

- `accept` 函数是一个阻塞函数，当没有客户端连接请求时，函数会一直等待直到有连接请求到达。
- 当 `accept` 函数返回一个新的套接字描述符后，原始的监听套接字 sockfd 仍然可用，可以继续调用 `accept` 函数接受其他客户端的连接请求。
- 在多线程或多进程环境中使用 `accept` 函数时，需要注意使用适当的同步机制，以避免竞争条件和并发问题。

## connect() 客户端: 建立与服务器的连接

```C++
/**
 * @brief 建立与服务器的连接
 *
 * 此函数用于建立与远程服务器的连接。
 *
 * @param sockfd 套接字描述符
 * @param servaddr 服务器地址结构体指针，包含服务器的 IP 地址和端口号
 * @param addrlen 服务器地址结构体大小
 * @return 成功时返回 0，失败时返回 -1
 */
int connect(int sockfd, const struct sockaddr *servaddr, socklen_t addrlen);
```


## read() 从文件描述符或套接字读取数据 (不推荐)

```C++
/**
 * @brief 从文件描述符或套接字读取数据
 *
 * 此函数用于从套接字中读取数据。
 *
 * @param fd 文件描述符或套接字描述符
 * @param buf 数据缓冲区指针
 * @param count 期望读取的字节数
 * @return 成功时返回实际读取的字节数，失败时返回 -1
 */
ssize_t read(int fd, void *buf, size_t count);
```

read 原则：

数据在不超过指定的长度的时候有多少读多少，没有数据则会一直等待。所以一般情况下：我们读取数据都需要采用循环读的方式读取数据，因为一次read 完毕不能保证读到我们需要长度的数据，read 完一次需要判断读到的数据长度再决定是否还需要再次读取。

## recv() 从套接字接收数据 (推荐)

```C++
/**
 * @brief 从套接字接收数据
 *
 * 此函数用于从套接字中接收数据。
 *
 * @param sockfd 套接字描述符
 * @param buf 数据缓冲区指针
 * @param len 缓冲区长度
 * @param flags 接收标志，用于控制数据传输的行为
 * @return 成功时返回实际接收的字节数，失败时返回 -1, 返回0为对方已调用close()关闭了连接
 */
ssize_t recv(int sockfd, void *buf, size_t len, int flags);
```

`recv()` 函数用于从套接字中接收数据，并将数据存储到指定的缓冲区 `buf` 中。`len` 参数指定了缓冲区的长度，而 `flags` 参数用于控制数据传输的行为，例如设置阻塞或非阻塞模式、指定接收数据的特性等。

`recv()` 函数返回实际接收的字节数，如果出现错误，返回 -1。

- 当socket处于阻塞模式时,继续调用send/recv函数,程序会阻塞在send/recv调用处
- 当socket处于非阻塞模式时,继续调用send/recv函数,会返回**错误码**<sup>参考: [socket的阻塞模式和非阻塞模式(send和recv函数在阻塞和非阻塞模式下的表现)](https://blog.csdn.net/qq_42896106/article/details/118531397)</sup>

> 在非阻塞模式下，如果没有可用的数据，recv() 可能会返回 错误码 ? 混淆怎么办?
>
> ```C++
char buffer[1024];
int recv_size = recv(client_socket, buffer, sizeof(buffer), 0);
if (recv_size == 0)
{
    // 对方已关闭连接
    printf("Peer closed the connection\n");
    close(client_socket);
}
else if (recv_size < 0)
{
    if (errno == EWOULDBLOCK || errno == EAGAIN)
    {
        // 没有可用的数据
        printf("No data available\n");
    }
    else
    {
        // 接收数据出错
        printf("Recv error: %s\n", strerror(errno));
        close(client_socket);
    }
}
else
{
    // 成功接收到数据
    // 处理接收到的数据
}
> ```
>
> [知乎: linux 下用非阻塞的 socket 编程，如何用 recv 接收数据？](https://www.zhihu.com/question/596124033)

需要注意的是，`recv()` 函数是在网络编程中常用的函数，适用于 TCP 和 UDP 协议的套接字。

---

recv 原则:

recv 中有一个`MSG_WAITALL`的参数:

正常情况下`recv`是会等待直到读取到`buff_size`长度的数据，但是这里的`WAITALL`也只是尽量读全，在有中断的情况下`recv`还是可能会被打断，造成没有读完指定的buff_size的长度。所以即使是采用`recv + WAITALL`参数还是要考虑**是否需要循环读取**的问题，在实验中对于多数情况下`recv (使用了MSG_WAITALL)`还是可以读完`buff_size`，

所以相应的性能会比直接read 进行循环读要好一些。<sup>[[read 与 recv 区别]](https://www.cnblogs.com/liyulong1982/p/4980969.html)</sup>

```C++
read(sockfd, buff, buff_size);       
write(sockfd, buff, buff_size);
recv(sockfd, buff, buff_size,MSG_WAITALL);  //阻塞模式接收        
send(scokfd, buff, buff_size,MSG_WAITALL);  //阻塞模式发送
recv(sockfd, buff, buff_size,MSG_DONTWAIT); //非阻塞模式接收        
send(scokfd, buff, buff_size,MSG_DONTWAIT); //非阻塞模式发送
recv(sockfd, buff, buff_size,0);        
send(scokfd, buff, buff_size,0);
```

socket编程经验<sup>(还是上面链接说的)</sup>

- 1）尽量使用`recv(,,MSG_WAITALL)`, `read`必须配合`while`使用，否则数据量大`(240*384)`时数据读不完
- 2）编程时写入的数据必须尽快读出，否则后面的数据将无法继续写入
- 3）最佳搭配如下:
  
  ```C++
  nbytes = recv(sockfd, buff, buff_size, MSG_WAITALL);
  nbytes = send(scokfd, buff, buff_size, MSG_WAITALL);
  ```
## write() 向套接字写入数据 (不推荐)

```C++
/**
 * @brief 向文件描述符或套接字描述符写入数据
 *
 * 此函数用于向套接字中写入数据。
 *
 * @param fd 文件描述符或套接字描述符
 * @param buf 数据缓冲区指针
 * @param count 要写入的字节数
 * @return 成功时返回实际写入的字节数，失败时返回 -1
 */
ssize_t write(int fd, const void *buf, size_t count);
```

参考: [Linux下，write/read，recv/send，recvfrom/sendto的区别](https://blog.csdn.net/sea_snow/article/details/112260750) (**应用场景：read/wirte是通用的文件描述符操作；recv/send 通常应用于TCP；recvfrom/sendto通常应用于UDP**)

需要注意的是，`write()`函数是一个底层的系统调用，它没有提供像`send()`函数那样的额外的网络选项（如`flags`参数）。因此，在使用`write()`函数发送数据时，你可能需要自己处理一些与网络相关的细节，例如设置套接字为非阻塞模式、处理部分写入的情况等。

如果你正在编写网络编程代码，通常建议使用`send()`函数而不是`write()`函数，因为`send()`函数更适合网络通信，并且提供了更多的可选标志和错误处理机制。

## send() 发送数据到目标地址 (推荐)

```C++
/**
 * @brief 发送数据到目标地址
 *
 * 该函数用于发送数据到指定的目标地址。
 *
 * @param sockfd 套接字描述符，用于标识一个已连接的网络端点
 * @param data 指向要发送的数据缓冲区的指针
 * @param size 要发送的数据大小（字节数）
 * @param flags 可选的标志参数，默认为0
 * @return 成功发送的字节数，如果发生错误则返回-1，并且errno被设置为相应的错误代码
 *
 * @note 这个函数可能会在数据发送过程中返回少于请求的字节数，需要进行错误处理和重发操作
 */
ssize_t send(int sockfd, const void *data, size_t size, int flags);
```

## close() 关闭套接字或文件描述符


```C++
/**
 * @brief 关闭套接字或文件描述符
 *
 * 此函数用于关闭已打开的套接字或文件描述符。
 *
 * @param fd 套接字描述符或文件描述符
 * @return 成功时返回 0，失败时返回 -1
 */
int close(int fd);
```

---

## inet_pton() 将文本格式的 IP 地址转换为二进制格式

```C++
/**
 * @brief 将文本格式的 IP 地址转换为二进制格式
 *
 * 此函数用于将提供的文本格式的 IP 地址转换为二进制格式。
 *
 * @param af 协议族/域，如 AF_INET（IPv4）、AF_INET6（IPv6）等
 * @param src 文本格式的 IP 地址字符串
 * @param dst 存储转换后二进制格式的地址的指针
 * @return 成功时返回 1，失败时返回 0
 */
int inet_pton(int af, const char *src, void *dst);
```
