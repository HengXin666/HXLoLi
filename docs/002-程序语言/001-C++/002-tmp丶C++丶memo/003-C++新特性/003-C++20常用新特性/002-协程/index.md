# 协程
## 说明
> 协程就是一个可以挂起（suspend）和恢复（resume）的函数（不能是 main 函数）。你可以暂停协程的执行，去做其他事情，然后在适当的时候恢复到暂停的位置继续执行。协程让我们使用同步方式写异步代码。

C++ 提供了三个方法挂起协程：`co_await`， `co_yield` 和 `co_return`。

C++20协程只是提供协程机制，而不是提供协程库。C++20的协程是无栈协程，无栈协程是一个可以挂起/恢复的特殊函数，是函数调用的泛化，且只能被线程调用，本身并不抢占内核调度。

`C++20` 提供了三个**新关键字(co_await、co_yield 和 co_return)**，如果一个函数中存在这三个关键字之一，那么它就是一个**协程**<sup>[1]</sup>。

- `co_yield some_value`: 保存当前协程的执行状态并挂起，返回some_value给调用者

- `co_await some_awaitable`: 如果some_awaitable没有ready，就保存当前协程的执行状态并挂起

- `co_return some_value:` 彻底结束当前协程，返回some_value给协程调用者

## 注解
### [1]

[官方中文文档: 协程 (C++20)](https://zh.cppreference.com/w/cpp/language/coroutines)