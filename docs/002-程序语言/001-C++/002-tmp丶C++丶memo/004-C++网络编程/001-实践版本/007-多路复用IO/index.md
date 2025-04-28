# IO 多路复用

在多路复用IO模型中，会有一个专门的线程不断去轮询多个socket的状态，只有当socket真正有读写事件时，才真正调用实际的IO读写操作。IO多路复用的优势在于，可以处理大量并发的IO，而不用消耗太多CPU/内存。

三种常用的轮询方法：select、poll、 $epoll$ (最常用, 最重要)

本页不会详细说明什么什么什么, 因为这本身好像不是系统学习的 =-=, 暂时性的先去用`epoll` 篇吧

https://blog.csdn.net/baidu_41388533/article/details/110134366