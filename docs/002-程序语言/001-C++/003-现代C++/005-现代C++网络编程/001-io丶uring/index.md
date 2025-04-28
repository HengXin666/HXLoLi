# io_uring
> 前言, 本来打算搞一个协程以异步读写文件, 就顺便复用到EpollLoop的了, 私以为它可以检测的, 然而Epoll只能监控某些类型的文件描述符...然后就爆炸啦! 用AIO来异步又要重写协程控制... (epoll不适用于磁盘IO, 因为磁盘文件总是可读的。)

参考: [epoll 能监听普通文件吗？](https://cloud.tencent.com/developer/article/1835294)

故而只能来学习现代Linuxのio_uring啦~

## 0. 下载
已经封装了的库:

- https://github.com/axboe/liburing/tree/liburing-2.6

自己整理的需要的文件:

- 核心文件: https://github.com/HengXin666/HXNet/tree/main/lib/liburing

## 1. 什么是 `io_uring`

`io_uring` 是 Linux 5.1 及更高版本引入的一种高效异步 I/O 机制, 它提供了一个基于环形队列的接口, 用于高性能的异步 I/O 操作。它的设计目的是减少系统调用开销, 降低上下文切换的频率, 从而提升  I/O 性能。

### 1.1 主要特点

- **环形队列**: `io_uring` 使用两个环形队列——提交队列(Submission Queue, SQ)和完成队列(Completion Queue, CQ)。应用程序将 I/O 请求放入提交队列, 内核处理这些请求并将结果放入完成队列。

- **批量处理**: `io_uring` 支持批量提交和处理 I/O 请求, 通过减少系统调用次数来提高效率。(原来需要多次系统调用(读或写), 现在变成批处理一次提交。)

- **零拷贝**: 支持直接从用户空间到内核空间的数据传输, 减少了不必要的数据拷贝操作。

- **低延迟**: 通过减少系统调用和上下文切换的次数, `io_uring` 能够提供比传统 I/O 模型更低的延迟。

### 1.2 与 [IOCP](../../../002-tmp丶C++丶memo/004-C++网络编程/002-Windows网络编程/002-IOCP/index.md) 的比较

- **设计理念**: `io_uring` 和 Windows 的 I/O Completion Ports (IOCP) 都是为了提升异步 I/O 性能而设计的。它们都通过将 I/O 请求和完成事件排入队列来减少系统调用和上下文切换的开销。

- **实现细节**: 
  - **`io_uring`**: 在 Linux 中, `io_uring` 使用环形队列来处理提交和完成请求。它允许批量提交请求并批量获取完成事件, 从而减少系统调用的开销。
  - **IOCP**: 在 Windows 中, IOCP 使用线程池和完成端口来处理异步 I/O 操作。它允许将完成事件与线程池中的线程进行关联, 并使用队列来处理完成事件。

- **API 和接口**: 
  - **`io_uring`**: 提供了更低层的 API 接口, 允许用户直接操作环形队列。
  - **IOCP**: 提供了更高级的 API, 通常需要与线程池和事件处理机制配合使用。

## 1.3 三种工作模式
1. **中断驱动模式**(interrupt driven)
    
    **默认模式**。可通过`io_uring_enter()`提交 I/O 请求, 然后直接检查 CQ 状态判断是否完成。
    
2. **轮询模式**(polled)
    
    Busy-waiting for an I/O completion, 而不是通过异步 IRQ(Interrupt Request)接收通知。
    
    这种模式需要文件系统(如果有)和块设备(block device)支持轮询功能。 相比中断驱动方式, 这种方式延迟更低([连系统调用都省了](https://www.phoronix.com/scan.php?page=news_item&px=Linux-io_uring-Fast-Efficient)), 但可能会消耗更多 CPU 资源。
    
    目前, 只有指定了`O_DIRECT`flag 打开的文件描述符, 才能使用这种模式。当一个读或写请求提交给轮询上下文(polled context)之后, 应用(application)必须调用`io_uring_enter()`来轮询 CQ 队列, 判断请求是否已经完成。
    
    对一个 io_uring 实例来说, **不支持混合使用轮询和非轮询模式**。
    
3. **内核轮询模式**(kernel polled)
    
    这种模式中, 会 **创建一个内核线程**(kernel thread)来执行 SQ 的轮询工作。
    
    使用这种模式的 io_uring 实例,  **应用无需切到到内核态** 就能触发(issue)I/O 操作。 通过 SQ 来提交 SQE, 以及监控 CQ 的完成状态, 应用无需任何系统调用, 就能提交和收割 I/O(submit and reap I/Os)。
    
    如果内核线程的空闲时间超过了用户的配置值, 它会通知应用, 然后进入 idle 状态。 这种情况下, 应用必须调用`io_uring_enter()`来唤醒内核线程。如果 I/O 一直很繁忙, 内核线性是不会`sleep`的。

## 2. 如何使用
### 2.1 原生调用

1. **初始化 `io_uring`**:
   ```cpp
   struct io_uring ring;
   io_uring_queue_init(QUEUE_SIZE, &ring, 0);
   ```

2. **准备请求**:
   ```cpp
   struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
   io_uring_prep_readv(sqe, fd, &iov, 1, 0);
   ```

3. **提交请求**:
   ```cpp
   io_uring_submit(&ring);
   ```

4. **等待和处理完成**:
   ```cpp
   struct io_uring_cqe *cqe;
   io_uring_wait_cqe(&ring, &cqe);
   io_uring_cqe_seen(&ring, cqe);
   ```

5. **关闭 `io_uring`**:
   ```cpp
   io_uring_queue_exit(&ring);
   ```

> omg, 有点麻烦, pip 库! 我要库! わくわく!~ ~~(不是哥们, 这里不是python捏)~~

### 2.2 liburing库
#### 2.2.1 初始化和退出

```C++
#include <liburing.h>

int main() {
    io_uring ring;
    /**
     * @brief 初始化长度为 32 (一般是2的幂) 的环形队列
     * 给 ring,
     * tag 是 0, 即没有标志
     */
    io_uring_queue_init(32, &ring, 0);

    // C语言魅力时刻 (因为他们没有析构函数)
    io_uring_queue_exit(&ring);
    return 0;
}
```

环形队列就是好比一个人在前面拉一个人在后面吃, 这样就不用频繁移动元素

#### 2.2.2 认识环形队列

```C++
struct io_uring {
    struct io_uring_sq sq; // 请求队列
    struct io_uring_cq cq; // 完成队列 (获取结果)
    unsigned flags;
    int ring_fd;

    unsigned features;
    int enter_ring_fd;
    __u8 int_flags;
    __u8 pad[3];
    unsigned pad2;
};
```

然后队列里面还有: `sqe`、`cqe` (e是指事件, 即完成的表项)

```C++
/*
 * Library interface to io_uring
 */
struct io_uring_sq {
    unsigned *khead; // 环形的头
    unsigned *ktail; // 环形的尾
    // Deprecated: use `ring_mask` instead of `*kring_mask`
    unsigned *kring_mask;
    // Deprecated: use `ring_entries` instead of `*kring_entries`
    unsigned *kring_entries;
    unsigned *kflags;
    unsigned *kdropped;
    unsigned *array;
    struct io_uring_sqe *sqes; // 提交的待完成任务的表项

    unsigned sqe_head;
    unsigned sqe_tail;

    size_t ring_sz; // 环形缓冲区的长度
    unsigned char *ring_ptr; // 环形缓冲区的基地址

    unsigned ring_mask;
    unsigned ring_entries;

    unsigned pad[2];
};

// 同上
struct io_uring_cq {
    unsigned *khead;
    unsigned *ktail;
    // Deprecated: use `ring_mask` instead of `*kring_mask`
    unsigned *kring_mask;
    // Deprecated: use `ring_entries` instead of `*kring_entries`
    unsigned *kring_entries;
    unsigned *kflags;
    unsigned *koverflow;
    struct io_uring_cqe *cqes;

    size_t ring_sz;
    unsigned char *ring_ptr;

    unsigned ring_mask;
    unsigned ring_entries;

    unsigned pad[2];
};
```

那为什么用环形缓冲区呢, 是因为我们早提交的数据, 他一定是希望早完成, (凡是讲究个先来后到嘛, 不然就会出现某个任务可能永远也轮不到它)

#### 2.2.3 添加任务与获取结果

在上面的`main()`的初始化和删除之间:

```C++
// 获取任务队列
io_uring_sqe* sqe = io_uring_get_sqe(&ring);
char buf[16];
/**
 * @brief 向任务队列添加异步读任务
 * sqe 需要添加任务的任务队列指针
 * STDIN_FILENO (输入流) fd (启动程序系统自动打开的文件)
 * buf 存放读取结果的数组
 * 16 一般是需要读取的长度(buf.size())
 * 0 文件偏移量
 */
// sqe->user_data = (u_int32_t)&A; 可以存放用户数据
io_uring_prep_read(sqe, STDIN_FILENO, buf, 16, 0);

// 提交任务队列给内核 (为什么不是sqe, 因为sqe是从ring中get出来的, 故其本身就包含了sqe)
io_uring_submit(&ring);

io_uring_cqe* cqe = nullptr;

// 阻塞等待内核, 返回是错误码; cqe是完成队列, 为传出参数
io_uring_wait_cqe(&ring, &cqe);
// io_uring_wait_cqe_timeout() 有带超时时间的

// 在销毁之前, 我们需要取出数据
int res = cqe->res; // 这个就是对应任务的返回值(read的返回值, 即读取的字节数)
// cqe->user_data 这个是 u_int64_t 到时候就可以放置指针, 从而回复协程 

// 销毁完成队列, 不然会一直在里面滞留(占用空间)
io_uring_cqe_seen(&ring, cqe);
```

当然他们也封装了set/get`data64`的方法:

```C++
// set
// sqe->user_data = (u_int32_t)&A;
io_uring_sqe_set_data64(sqe, (u_int32_t)&A);

// get
// u_int64_t data = cqe->user_data
u_int64_t data = io_uring_cqe_get_data64(cqe);

// 不想要类型强制转换可以使用: 其参数是void *, cqe同
io_uring_sqe_set_data(, void*)
```

注: 但它返回完成, 我们可以协程继续/回调到之前的位置, 此时`buf`已经是收到消息啦~

以上就是基本用法! 当然`io_uring_prep_`开头的还有很多, 支持一般读写/连接...

#### 2.2.4 示例

> 节选自我的项目: https://github.com/HengXin666/HXNet

```C++
bool IoUringLoop::run(std::optional<std::chrono::system_clock::duration> timeout) {
    ::io_uring_cqe* cqe = nullptr;

    __kernel_timespec timespec = {0, 0}; // 设置超时为无限阻塞

    if (timeout.has_value()) {
        auto duration = timeout.value();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000;
        timespec.tv_sec = static_cast<long>(seconds);
        timespec.tv_nsec = static_cast<long>(nanoseconds);
    }

    // 阻塞等待内核, 返回是错误码; cqe是完成队列, 为传出参数
    int res = io_uring_submit_and_wait_timeout(&_ring, &cqe, 1, &timespec, nullptr);
    if (res == -ETIME) {
        return false;
    } else if (res < 0) [[unlikely]] {
        if (res == -EINTR) {
            return false;
        }
        throw std::system_error(-res, std::system_category());
    }

    unsigned head, numGot = 0;
    std::vector<std::coroutine_handle<>> tasks;
    io_uring_for_each_cqe(&_ring, head, cqe) {
        auto* task = reinterpret_cast<IoUringTask *>(cqe->user_data);
        task->_res = cqe->res;
        tasks.emplace_back(task->_previous);
        ++numGot;
    }

    // 手动前进完成队列的头部 (相当于批量io_uring_cqe_seen)
    ::io_uring_cq_advance(&_ring, numGot);
    _numSqesPending -= static_cast<std::size_t>(numGot);
    for (auto&& it : tasks) {
        it.resume();
    }
    return true;
}
```

```C++
/**
 * @brief 链接超时操作
 * @param lhs 操作
 * @param rhs 空连接的超时操作 (prepLinkTimeout)
 * @return IoUringTask&& 
 */
static IoUringTask &&linkOps(IoUringTask &&lhs, IoUringTask &&rhs) {
    lhs._sqe->flags |= IOSQE_IO_LINK;
    rhs._previous = std::noop_coroutine();
    return std::move(lhs);
}

/**
 * @brief 创建未链接的超时操作
 * @param ts 超时时间
 * @param flags 
 * @return IoUringTask&& 
 */
IoUringTask &&prepLinkTimeout(
    struct __kernel_timespec *ts,
    unsigned int flags
) && {
    ::io_uring_prep_link_timeout(_sqe, ts, flags);
    return std::move(*this);
}
```
