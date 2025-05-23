# C++线程同步之互斥锁
解决多线程数据混乱的方案就是进行线程同步，最常用的就是互斥锁，在`C++11`中一共提供了四种互斥锁：

- `std::mutex`：独占的互斥锁，不能递归使用
- `std::timed_mutex`：带超时的独占互斥锁，不能递归使用
- `std::recursive_mutex`：递归互斥锁，不带超时功能
- `std::recursive_timed_mutex`：带超时的递归互斥锁

互斥锁在有些资料中也被称之为互斥量，二者是一个东西。

## std::mutex
不论是在C还是C++中，进行线程同步的处理流程基本上是一致的，C++的`mutex`类提供了相关的API函数:

### lock()
`lock()`函数<span style="color:red">用于给临界区加锁，并且只能有一个线程获得锁的所有权，它有**阻塞线程**的作用</span>，函数原型如下:

```C++
void lock();
```

独占互斥锁对象有两种状态: 锁定和未锁定。如果互斥锁是打开的，调用`lock()`函数的线程会得到互斥锁的所有权，并将其上锁，其它线程再调用该函数的时候由于得不到互斥锁的所有权，就会被`lock()`函数阻塞。当拥有互斥锁所有权的线程将互斥锁解锁，此时被`lock()`阻塞的线程解除阻塞，抢到互斥锁所有权的线程加锁并继续运行，没抢到互斥锁所有权的线程继续阻塞。

### try_lock()

除了使用`lock()`还可以使用`try_lock()`获取互斥锁的所有权并对互斥锁加锁，函数原型如下:

```C++
bool try_lock();
```
二者的区别在于`try_lock()`**不会阻塞线程**，`lock()`**会阻塞线程**:

- 如果互斥锁是未锁定状态，得到了互斥锁所有权并加锁成功，函数返回`true`
- 如果互斥锁是锁定状态，无法得到互斥锁所有权加锁失败，函数返回`false`

### unlock()

当互斥锁被锁定之后可以通过`unlock()`进行解锁，但是需要注意的是**只有拥有互斥锁所有权的线程也就是对互斥锁上锁的线程才能将其解锁**，其它线程是没有权限做这件事情的。该函数的函数原型如下:

```C++
void unlock();
```

### 小结

通过介绍以上三个函数，使用互斥锁进行线程同步的大致思路差不多就能搞清楚了，主要分为以下几步：

1. 找到多个线程操作的共享资源（全局变量、堆内存、类成员变量等），也可以称之为临界资源
2. 找到和共享资源有关的上下文代码，也就是临界区
3. 在临界区的上边调用互斥锁类的`lock()`方法
4. 在临界区的下边调用互斥锁的`unlock()`方法

<span style="color:red">线程同步的目的是让多线程按照顺序依次执行临界区代码，这样做线程对共享资源的访问就从并行访问变为了线性访问，访问效率降低了，但是保证了数据的正确性。</span>

> [!TIP]
> <span style="color:red">当线程对互斥锁对象加锁，并且执行完临界区代码之后，一定要使用这个线程对互斥锁解锁，否则最终会造成线程的死锁。死锁之后当前应用程序中的所有线程都会被阻塞，并且阻塞无法解除，应用程序也无法继续运行。</span>

### 示例代码

```C++
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
using namespace std;

int g_num = 0;  // 为 g_num_mutex 所保护
mutex g_num_mutex;

void slow_increment(int id)
{
    for (int i = 0; i < 3; ++i) 
    {
        g_num_mutex.lock();
        ++g_num;
        cout << id << " => " << g_num << endl;
        g_num_mutex.unlock();

        this_thread::sleep_for(chrono::seconds(1));
    }
}

int main()
{
    thread t1(slow_increment, 0);
    thread t2(slow_increment, 1);
    t1.join();
    t2.join();
}
```
在上面的示例程序中，两个子线程执行的任务的一样的（其实也可以不一样，不同的任务中也可以对共享资源进行读写操作），在任务函数中把与全局变量相关的代码加了锁，两个线程只能顺序访问这部分代码（如果不进行线程同步打印出的数据是混乱且无序的）。另外需要强调一点：

1. 在所有线程的任务函数执行完毕之前，互斥锁对象是不能被析构的，一定要在程序中保证这个对象的可用性。
2. 互斥锁的个数和共享资源的个数相等，也就是说每一个共享资源都应该对应一个互斥锁对象。互斥锁对象的个数和线程的个数没有关系。

## std::lock_guard
`lock_guard`是`C++11`新增的一个模板类，使用这个类，可以简化互斥锁`lock()`和`unlock()`的写法，同时也更安全。这个模板类的定义和常用的构造函数原型如下:

```C++
// 类的定义，定义于头文件 <mutex>
template< class Mutex >
class lock_guard;

// 常用构造函数
explicit lock_guard( mutex_type& m );
```

`lock_guard`在使用上面提供的这个构造函数构造对象时，会自动锁定互斥量，而在退出作用域后进行析构时就会自动解锁，从而保证了互斥量的正确操作，避免忘记`unlock()`操作而导致线程死锁。`lock_guard`使用了RAII技术，就是在类构造函数中分配资源，在析构函数中释放资源，**保证资源出了作用域就释放**。

使用`lock_guard`对上面的例子进行修改，代码如下:

```C++
void slow_increment(int id)
{
    for (int i = 0; i < 3; ++i) 
    {
        // 使用哨兵锁管理互斥锁
        lock_guard<mutex> lock(g_num_mutex);
        ++g_num;
        cout << id << " => " << g_num << endl;
        this_thread::sleep_for(chrono::seconds(1));
    }
}
```

通过修改发现代码被精简了，而且不用担心因为忘记解锁而造成程序的死锁，但是这种方式也有**弊端**，在上面的示例程序中整个`for`循环的体都被当做了**临界区**，多个线程是线性的执行临界区代码的，因此**临界区越大程序效率越低**，还是需要根据实际情况选择最优的解决方案。

## std::recursive_mutex
递归互斥锁`std::recursive_mutex`允许同一线程多次获得互斥锁，可以用来解决同一线程需要多次获取互斥量时死锁的问题，在下面的例子中使用独占非递归互斥量会发生死锁:

```C++
#include <iostream>
#include <thread>
#include <mutex>
using namespace std;

struct Calculate
{
    Calculate() : m_i(6) {}

    void mul(int x)
    {
        lock_guard<mutex> locker(m_mutex);
        m_i *= x;
    }

    void div(int x)
    {
        lock_guard<mutex> locker(m_mutex);
        m_i /= x;
    }

    void both(int x, int y)
    {
        lock_guard<mutex> locker(m_mutex);
        mul(x);
        div(y);
    }

    int m_i;
    mutex m_mutex;
};

int main()
{
    Calculate cal;
    cal.both(6, 3);
    return 0;
}
```

上面的程序中执行了`cal.both(6, 3);`调用之后，程序就会发生死锁，在`both()`中已经对互斥锁加锁了，继续调用`mult()`函数，已经得到互斥锁所有权的线程再次获取这个互斥锁的所有权就会造成死锁（在C++中程序会异常退出，使用C库函数会导致这个互斥锁永远无法被解锁，最终阻塞所有的线程）。要解决这个死锁的问题，一个简单的办法就是使用递归互斥锁`std::recursive_mutex`，它允许一个线程多次获得互斥锁的所有权。修改之后的代码如下

```C++
#include <iostream>
#include <thread>
#include <mutex>
using namespace std;

struct Calculate
{
    Calculate() : m_i(6) {}

    void mul(int x)
    {
        lock_guard<recursive_mutex> locker(m_mutex);
        m_i *= x;
    }

    void div(int x)
    {
        lock_guard<recursive_mutex> locker(m_mutex);
        m_i /= x;
    }

    void both(int x, int y)
    {
        lock_guard<recursive_mutex> locker(m_mutex);
        mul(x);
        div(y);
    }

    int m_i;
    recursive_mutex m_mutex;
};

int main()
{
    Calculate cal;
    cal.both(6, 3);
    cout << "cal.m_i = " << cal.m_i << endl;
    return 0;
}
```
虽然递归互斥锁可以解决同一个互斥锁频繁获取互斥锁资源的问题，但是还是建议`少用`，主要原因如下：

1. 使用递归互斥锁的场景往往都是可以简化的，使用递归互斥锁很容易放纵复杂逻辑的产生，从而导致bug的产生
2. 递归互斥锁比非递归互斥锁效率要低一些。
3. 递归互斥锁虽然允许同一个线程多次获得同一个互斥锁的所有权，但最大次数并未具体说明，一旦超过一定的次数，就会抛出`std::system`错误。

## std::timed_mutex
`std::timed_mutex`是超时独占互斥锁，主要是在获取互斥锁资源时增加了超时等待功能，因为不知道获取锁资源需要等待多长时间，为了保证不一直等待下去，设置了一个超时时长，超时后线程就可以解除阻塞去做其他事情了。

`std::timed_mutex`比`std::_mutex`多了两个成员函数：`try_lock_for()`和`try_lock_until()`:

```C++
void lock();
bool try_lock();
void unlock();

// std::timed_mutex比std::_mutex多出的两个成员函数
template <class Rep, class Period>
  bool try_lock_for (const chrono::duration<Rep,Period>& rel_time);

template <class Clock, class Duration>
  bool try_lock_until (const chrono::time_point<Clock,Duration>& abs_time);
```
- `try_lock_for`函数是当线程获取不到互斥锁资源的时候，让线程阻塞一定的**时间长度**
- `try_lock_until`函数是当线程获取不到互斥锁资源的时候，让线程阻塞到某一个指定的**时间点**
- 关于两个函数的返回值：当得到互斥锁的所有权之后，函数会马上解除阻塞，返回`true`，如果阻塞的时长用完或者到达指定的时间点之后，函数也会解除阻塞，返回`false`

下面的示例程序中为大家演示了`std::timed_mutex`的使用:

```C++
#include <iostream>
#include <thread>
#include <mutex>
using namespace std;

timed_mutex g_mutex;

void work()
{
    chrono::seconds timeout(1);
    while (true)
    {
        // 通过阻塞一定的时长来争取得到互斥锁所有权
        if (g_mutex.try_lock_for(timeout))
        {
            cout << "当前线程ID: " << this_thread::get_id() 
                << ", 得到互斥锁所有权..." << endl;
            // 模拟处理任务用了一定的时长
            this_thread::sleep_for(chrono::seconds(10));
            // 互斥锁解锁
            g_mutex.unlock();
            break;
        }
        else
        {
            cout << "当前线程ID: " << this_thread::get_id() 
                << ", 没有得到互斥锁所有权..." << endl;
            // 模拟处理其他任务用了一定的时长
            this_thread::sleep_for(chrono::milliseconds(50));
        }
    }
}

int main()
{
    thread t1(work);
    thread t2(work);

    t1.join();
    t2.join();

    return 0;
}
```

输出:

```C++
当前线程ID: 125776, 得到互斥锁所有权...
当前线程ID: 112324, 没有得到互斥锁所有权...
当前线程ID: 112324, 没有得到互斥锁所有权...
当前线程ID: 112324, 没有得到互斥锁所有权...
当前线程ID: 112324, 没有得到互斥锁所有权...
当前线程ID: 112324, 没有得到互斥锁所有权...
当前线程ID: 112324, 没有得到互斥锁所有权...
当前线程ID: 112324, 没有得到互斥锁所有权...
当前线程ID: 112324, 没有得到互斥锁所有权...
当前线程ID: 112324, 没有得到互斥锁所有权...
当前线程ID: 112324, 得到互斥锁所有权...
```

在上面的例子中，通过一个`while`循环不停的去获取超时互斥锁的所有权，如果得不到就阻塞1秒钟，1秒之后如果还是得不到阻塞50毫秒，然后再次继续尝试，直到获得互斥锁的所有权，跳出循环体。

关于递归超时互斥锁`std::recursive_timed_mutex`的使用方式和`std::timed_mutex`是一样的，只不过它可以允许一个线程多次获得互斥锁所有权，而`std::timed_mutex`只允许线程获取一次互斥锁所有权。另外，递归超时互斥锁`std::recursive_timed_mutex`也拥有和`std::recursive_mutex`一样的弊端，不建议频繁使用。

## 参考链接
[爱编程的大丙 C++线程同步之互斥锁](https://subingwen.cn/cpp/mutex/)