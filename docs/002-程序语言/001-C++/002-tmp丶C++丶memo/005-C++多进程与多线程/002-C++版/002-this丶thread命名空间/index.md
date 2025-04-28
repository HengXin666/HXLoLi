# std::this_thread 命名空间
在`C++11`中不仅添加了线程类，还添加了一个关于线程的命名空间`std::this_thread`，在这个命名空间中提供了四个公共的成员函数，通过这些成员函数就可以对当前线程进行相关的操作了。

## get_id()

调用命名空间`std::this_thread`中的`get_id()`方法可以得到当前线程的线程ID，函数原型如下:

```C++
thread::id get_id() noexcept;
```

关于函数使用对应的示例代码如下:

```C++
#include <iostream>
#include <thread>
using namespace std;

void func()
{
    cout << "子线程: " << this_thread::get_id() << endl;
}

int main()
{
    cout << "主线程: " << this_thread::get_id() << endl;
    thread t(func);
    t.join();
}
```

## sleep_for()
同样地线程被创建后也有这[五种状态](https://subingwen.cn/linux/process/#1-4-%E8%BF%9B%E7%A8%8B%E7%8A%B6%E6%80%81)：<span style="color:red">创建态，就绪态，运行态，阻塞态(挂起态)，退出态(终止态) </span>，关于状态之间的转换是一样的，请参考进程，在此不再过多的赘述。

线程和进程的执行有很多相似之处，在计算机中启动的多个线程都需要占用CPU资源，但是CPU的个数是有限的并且每个CPU在同一时间点不能同时处理多个任务。<span style="color:red">为了能够实现并发处理，多个线程都是分时复用CPU时间片，快速的交替处理各个线程中的任务。因此多个线程之间需要争抢CPU时间片，抢到了就执行，抢不到则无法执行</span>（因为默认所有的线程优先级都相同，内核也会从中调度，不会出现某个线程永远抢不到CPU时间片的情况）。

命名空间`this_thread`中提供了一个休眠函数`sleep_for()`，调用这个函数的线程会马上从`运行态`变成`阻塞态`并在这种状态下休眠一定的时长，因为阻塞态的线程已经让出了CPU资源，代码也不会被执行，所以线程休眠过程中对CPU来说没有任何负担。这个函数是函数原型如下，参数需要指定一个休眠时长，是一个时间段<sup>([处理日期和时间的chrono库](https://subingwen.cn/cpp/chrono/#1-%E6%97%B6%E9%97%B4%E9%97%B4%E9%9A%94duration))</sup>：

```C++
template <class Rep, class Period>
  void sleep_for (const chrono::duration<Rep,Period>& rel_time);
```

示例:

```C++
#include <iostream>
#include <thread>
#include <chrono>
using namespace std;

void func()
{
    for (int i = 0; i < 10; ++i)
    {
        this_thread::sleep_for(chrono::seconds(1));
        cout << "子线程: " << this_thread::get_id() << ", i = " << i << endl;
    }
}

int main()
{
    thread t(func);
    t.join();
}
```

在`func()`函数的`for`循环中使用了`this_thread::sleep_for(chrono::seconds(1));`之后，每循环一次程序都会阻塞1秒钟，也就是说每隔1秒才会进行一次输出。需要注意的是：**程序休眠完成之后，会从阻塞态重新变成就绪态，就绪态的线程需要再次争抢CPU时间片，抢到之后才会变成运行态，这时候程序才会继续向下运行**(休眠结束后需要抢时间片而不是接着向下)。<sup>[1]</sup>

## sleep_until()
命名空间`this_thread`中提供了另一个休眠函数`sleep_until()`，和`sleep_for()`不同的是它的参数类型不一样

- `sleep_until()`：指定线程阻塞到某一个指定的时间点`time_point`类型，之后解除阻塞
- `sleep_for()`：指定线程阻塞一定的时间长度`duration`类型，之后解除阻塞

[time_point类型如何使用](https://subingwen.cn/cpp/chrono/#2-%E6%97%B6%E9%97%B4%E7%82%B9-time-point)

该函数的函数原型如下:

```C++
template <class Clock, class Duration>
  void sleep_until (const chrono::time_point<Clock,Duration>& abs_time);
```

示例:

```C++
#include <iostream>
#include <thread>
#include <chrono>
using namespace std;

void func()
{
    for (int i = 0; i < 10; ++i)
    {
        // 获取当前系统时间点
        auto now = chrono::system_clock::now();
        // 时间间隔为2s
        chrono::seconds sec(2);
        // 当前时间点之后休眠两秒
        this_thread::sleep_until(now + sec);
        cout << "子线程: " << this_thread::get_id() << ", i = " << i << endl;
    }
}

int main()
{
    thread t(func);
    t.join();
}
```

`sleep_until()`和`sleep_for()`函数的功能是一样的，只不过前者是基于`时间点`去阻塞线程，后者是基于`时间段`去阻塞线程，项目开发过程中根据实际情况选择最优的解决方案即可。

## yield()
命名空间`this_thread`中提供了一个非常绅士的函数`yield()`，在线程中调用这个函数之后，<span style="color:red">处于运行态的线程会主动让出自己已经抢到的CPU时间片，最终变为就绪态</span>，这样其它的线程就有更大的概率能够抢到CPU时间片了。使用这个函数的时候需要注意一点，线程调用了`yield()`之后会主动放弃CPU资源，但是这个变为就绪态的线程会马上参与到下一轮CPU的抢夺战中，**不排除它能`继续`抢到CPU时间片的情况**，这是概率问题。

```C++
void yield() noexcept;
```

示例:

```C++
#include <iostream>
#include <thread>
using namespace std;

void func()
{
    for (int i = 0; i < 100000000000; ++i)
    {
        cout << "子线程: " << this_thread::get_id() << ", i = " << i << endl;
        this_thread::yield();
    }
}

int main()
{
    thread t(func);
    thread t1(func);
    t.join();
    t1.join();
}
```

在上面的程序中，执行`func()`中的`for`循环会占用大量的时间，在极端情况下，如果当前线程占用CPU资源不释放就会导致其他线程中的任务无法被处理，或者该线程每次都能抢到CPU时间片，导致其他线程中的任务没有机会被执行。解决方案就是每执行一次循环，让该线程主动放弃CPU资源，重新和其他线程再次抢夺CPU时间片，如果其他线程抢到了CPU时间片就可以执行相应的任务了。

> 结论：
>
> 1. `std::this_thread::yield()`的目的是避免一个线程长时间占用CPU资源，从而导致多线程处理性能下降
>
> 2. `std::this_thread::yield()`是让当前线程主动放弃了当前自己抢到的CPU资源，但是在下一轮还会继续抢


## 参考链接
[爱编程的大丙 命名空间 - this_thread](https://subingwen.cn/cpp/this_thread/)
### [1]

```
这个是和sleep的区别吧? 就是结束休眠后需要再次抢时间片

是的，这是 this_thread::sleep_for 和 sleep 的一个重要区别。使用 this_thread::sleep_for 会让线程进入阻塞状态，也就是暂时停止执行，直到指定的时间间隔过去。当时间间隔结束后，线程并不会立即继续执行，而是需要重新争抢 CPU 时间片，才能再次变为运行状态。

而 sleep 函数也会暂停程序执行，但它是使用操作系统提供的阻塞机制来实现的，因此在休眠期间，线程会被挂起，并不占用 CPU 时间片。当时间到达之后，线程会被唤醒，并且会立即恢复运行状态，不需要重新争抢 CPU 时间片。
```

[C++ std::this_thread::sleep_for()睡眠函数和C语言睡眠函数sleep()、Sleep()有什么区别？（sleep_for()更稳定）（线程休眠）](https://blog.csdn.net/Dontla/article/details/125132538)