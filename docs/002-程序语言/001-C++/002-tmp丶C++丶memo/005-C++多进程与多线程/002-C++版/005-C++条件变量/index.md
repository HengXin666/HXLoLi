# C++线程同步之条件变量
条件变量是`C++11`提供的另外一种用于等待的同步机制，它能阻塞一个或多个线程，直到收到另外一个线程发出的通知或者超时时，才会唤醒当前阻塞的线程。条件变量需要和互斥量配合起来使用，`C++11`提供了两种条件变量：

- `condition_variable`：需要配合`std::unique_lock<std::mutex>`进行`wait`操作，也就是阻塞线程的操作。
- `condition_variable_any`：可以和任意带有`lock()`、`unlock()`语义的`mutex`搭配使用，也就是说有四种：
  - `std::mutex`：独占的非递归互斥锁
  - `std::timed_mutex`：带超时的独占非递归互斥锁
  - `std::recursive_mutex`：不带超时功能的递归互斥锁
  - `std::recursive_timed_mutex`：带超时的递归互斥锁
条件变量通常用于生产者和消费者模型，大致使用过程如下：
1. 拥有条件变量的线程获取互斥量
2. 循环检查某个条件，如果条件不满足阻塞当前线程，否则线程继续向下执行
    - 产品的数量达到上限，生产者阻塞，否则生产者一直生产。。。
    - 产品的数量为零，消费者阻塞，否则消费者一直消费。。。
3. 条件满足之后，可以调用`notify_one()`或者`notify_all()`唤醒一个或者所有被阻塞的线程
    - 由消费者唤醒被阻塞的生产者，生产者解除阻塞继续生产。。。
    - 由生产者唤醒被阻塞的消费者，消费者解除阻塞继续消费。。。

## condition_variable
`condition_variable`的成员函数主要分为两部分：线程等待（阻塞）函数 和线程通知（唤醒）函数，这些函数被定义于头文件`<condition_variable>`。

### 等待函数
#### wait()
调用`wait()`函数的线程会被**阻塞**

```C++
// ①
void wait (unique_lock<mutex>& lck);
// ②
template <class Predicate>
void wait (unique_lock<mutex>& lck, Predicate pred);
```
函数①：调用该函数的线程直接被阻塞

函数②：该函数的第二个参数是一个判断条件，是一个返回值为布尔类型的函数

- 该参数可以传递一个有名函数的地址，也可以直接指定一个匿名函数
- 表达式返回`false`当前线程被阻塞，表达式返回`true`当前线程不会被阻塞，继续向下执行

独占的互斥锁对象不能直接传递给`wait()`函数，需要通过模板类`unique_lock`进行二次处理，通过得到的对象仍然可以对独占的互斥锁对象做如下操作，使用起来更灵活。

|公共成员函数|说明|
|:-|:-|
|lock|锁定关联的互斥锁|
|try_lock|尝试锁定关联的互斥锁，若无法锁定，函数直接返回|
|try_lock_for|试图锁定关联的可定时锁定互斥锁，若互斥锁在给定时长中仍不能被锁定，函数返回|
|try_lock_until|试图锁定关联的可定时锁定互斥锁，若互斥锁在给定的时间点后仍不能被锁定，函数返回|
|unlock|将互斥锁解锁|

**如果线程被该函数阻塞，这个线程会释放占有的互斥锁的所有权，当阻塞解除之后这个线程会重新得到互斥锁的所有权，继续向下执行**（这个过程是在函数内部完成的，了解这个过程即可，其目的是为了避免线程的死锁）。

#### wait_for()

`wait_for()`函数和`wait()`的功能是一样的，只不过多了一个阻塞时长，假设阻塞的线程没有被其他线程唤醒，当阻塞时长用完之后，线程就会自动解除阻塞，继续向下执行。

```C++
template <class Rep, class Period>
cv_status wait_for (unique_lock<mutex>& lck,
                    const chrono::duration<Rep,Period>& rel_time);
    
template <class Rep, class Period, class Predicate>
bool wait_for(unique_lock<mutex>& lck,
               const chrono::duration<Rep,Period>& rel_time, Predicate pred);
```
#### wait_until()
`wait_until()`函数和`wait_for()`的功能是一样的，它是指定让线程阻塞到某一个时间点，假设阻塞的线程没有被其他线程唤醒，当到达指定的时间点之后，线程就会自动解除阻塞，继续向下执行。

```C++
template <class Clock, class Duration>
cv_status wait_until (unique_lock<mutex>& lck,
                      const chrono::time_point<Clock,Duration>& abs_time);

template <class Clock, class Duration, class Predicate>
bool wait_until (unique_lock<mutex>& lck,
                 const chrono::time_point<Clock,Duration>& abs_time, Predicate pred);
```

### 通知函数
#### notify_one()
#### notify_all()

```C++
void notify_one() noexcept;
void notify_all() noexcept;
```
- `notify_one()`：唤醒**一个**被当前条件变量阻塞的线程
- `notify_all()`：唤醒**全部**被当前条件变量阻塞的线程

### 生产者和消费者模型
我们可以使用条件变量来实现一个同步队列，这个队列作为生产者线程和消费者线程的共享资源，示例代码如下：

```C++
#include <iostream>
#include <thread>
#include <mutex>
#include <list>
#include <functional>
#include <condition_variable>
using namespace std;

class SyncQueue
{
public:
    SyncQueue(int maxSize) : m_maxSize(maxSize) {}
    
    void put(const int& x)
    {
        unique_lock<mutex> locker(m_mutex);
        // 判断任务队列是不是已经满了
        while (m_queue.size() == m_maxSize)
        {
            cout << "任务队列已满, 请耐心等待..." << endl;
            // 阻塞线程
            m_notFull.wait(locker);
        }
        // 将任务放入到任务队列中
        m_queue.push_back(x);
        cout << x << " 被生产" << endl; 
        // 通知消费者去消费
        m_notEmpty.notify_one();
    }

    int take()
    {
        unique_lock<mutex> locker(m_mutex);
        while (m_queue.empty())
        {
            cout << "任务队列已空，请耐心等待。。。" << endl;
            m_notEmpty.wait(locker);
        }
        // 从任务队列中取出任务(消费)
        int x = m_queue.front();
        m_queue.pop_front();
        // 通知生产者去生产
        m_notFull.notify_one();
        cout << x << " 被消费" << endl;
        return x;
    }

private:
    list<int> m_queue;     // 存储队列数据
    mutex m_mutex;         // 互斥锁
    condition_variable m_notEmpty;   // 不为空的条件变量
    condition_variable m_notFull;    // 没有满的条件变量
    int m_maxSize;         // 任务队列的最大任务个数
};

int main()
{
    SyncQueue taskQ(50);
    // 把类方法变成一个 functional 函数
    auto produce = bind(&SyncQueue::put, &taskQ, placeholders::_1);
    auto consume = bind(&SyncQueue::take, &taskQ);
    thread t1[3];
    thread t2[3];
    for (int i = 0; i < 3; ++i)
    {
        t1[i] = thread(produce, i+100);
        t2[i] = thread(consume);
    }

    for (int i = 0; i < 3; ++i)
    {
        t1[i].join();
        t2[i].join();
    }

    return 0;
}
```
条件变量`condition_variable`类的`wait()`还有一个重载的方法，可以接受一个条件，这个条件也可以是一个返回值为布尔类型的函数，条件变量会先检查判断这个条件是否满足，如果满足条件（布尔值为true），则当前线程重新获得互斥锁的所有权，结束阻塞，继续向下执行；如果不满足条件（布尔值为false），当前线程会释放互斥锁（解锁）同时被阻塞，等待被唤醒。

上面示例程序中的`put()`、`take()`函数可以做如下修改：

- put()函数

```C++
void put(const int& x)
{
    unique_lock<mutex> locker(m_mutex);
    // 根据条件阻塞线程
    m_notFull.wait(locker, [this]() {
        return m_queue.size() != m_maxSize;
    });
    // 将任务放入到任务队列中
    m_queue.push_back(x);
    cout << x << " 被生产" << endl;
    // 通知消费者去消费
    m_notEmpty.notify_one();
}
```
- take()函数

```C++
int take()
{
    unique_lock<mutex> locker(m_mutex);
    m_notEmpty.wait(locker, [this]() {
        return !m_queue.empty();
    });
    // 从任务队列中取出任务(消费)
    int x = m_queue.front();
    m_queue.pop_front();
    // 通知生产者去生产
    m_notFull.notify_one();
    cout << x << " 被消费" << endl;
    return x;
}
```

修改之后可以发现，**程序变得更加精简了，而且执行效率更高了**，因为在这两个函数中的`while`循环被删掉了，但是最终的效果是一样的， $推荐$ 使用这种方式的`wait()`进行线程的阻塞。

## condition_variable_any
`condition_variable_any`的成员函数也是分为两部分：线程等待（阻塞）函数 和线程通知（唤醒）函数，这些函数被定义于头文件 `<condition_variable>`。
### 等待函数
#### wait()
```C++
// ①
template <class Lock> void wait (Lock& lck);
// ②
template <class Lock, class Predicate>
void wait (Lock& lck, Predicate pred);
```
- 函数①：调用该函数的线程直接被阻塞
- 函数②：该函数的第二个参数是一个判断条件，是一个返回值为布尔类型的函数
    - 该参数可以传递一个有名函数的地址，也可以直接指定一个匿名函数
    - 表达式返回false当前线程被阻塞，表达式返回true当前线程不会被阻塞，继续向下执行
- <span style="color:red">可以直接传递给`wait()`函数的互斥锁类型有四种，分别是：
    - `std::mutex`、`std::timed_mutex`、`std::recursive_mutex`、`std::recursive_timed_mutex`</span>
- 如果线程被该函数阻塞，这个线程会释放占有的互斥锁的所有权，当阻塞解除之后这个线程会重新得到互斥锁的所有权，继续向下执行（这个过程是在函数内部完成的，了解这个过程即可，其目的是为了避免线程的死锁）。

#### wait_for()
`wait_for()`函数和`wait()`的功能是一样的，只不过多了一个阻塞时长，假设阻塞的线程没有被其他线程唤醒，当阻塞时长用完之后，线程就会自动解除阻塞，继续向下执行。

```C++
template <class Lock, class Rep, class Period>
cv_status wait_for (Lock& lck, const chrono::duration<Rep,Period>& rel_time);
    
template <class Lock, class Rep, class Period, class Predicate>
bool wait_for (Lock& lck, const chrono::duration<Rep,Period>& rel_time, Predicate pred);
```

#### wait_until()
`wait_until()`函数和`wait_for()`的功能是一样的，它是指定让线程阻塞到某一个时间点，假设阻塞的线程没有被其他线程唤醒，当到达指定的时间点之后，线程就会自动解除阻塞，继续向下执行。

```C++
template <class Lock, class Clock, class Duration>
cv_status wait_until (Lock& lck, const chrono::time_point<Clock,Duration>& abs_time);

template <class Lock, class Clock, class Duration, class Predicate>
bool wait_until (Lock& lck, 
                 const chrono::time_point<Clock,Duration>& abs_time, 
                 Predicate pred);
```

### 通知函数
#### notify_one()
#### notify_all()
```C++
void notify_one() noexcept;
void notify_all() noexcept;
```
- `notify_one()`：唤醒**一个**被当前条件变量阻塞的线程
- `notify_all()`：唤醒**全部**被当前条件变量阻塞的线程

### 生产者和消费者模型
使用条件变量`condition_variable_any`同样可以实现上面的生产者和消费者的例子，代码只有个别**细节**上有所不同:

```C++
#include <iostream>
#include <thread>
#include <mutex>
#include <list>
#include <functional>
#include <condition_variable>
using namespace std;

class SyncQueue
{
public:
    SyncQueue(int maxSize) : m_maxSize(maxSize) {}

    void put(const int& x)
    {
        lock_guard<mutex> locker(m_mutex);
        // 根据条件阻塞线程
        m_notFull.wait(m_mutex, [this]() {
            return m_queue.size() != m_maxSize;
        });
        // 将任务放入到任务队列中
        m_queue.push_back(x);
        cout << x << " 被生产" << endl;
        // 通知消费者去消费
        m_notEmpty.notify_one();
    }

    int take()
    {
        lock_guard<mutex> locker(m_mutex);
        m_notEmpty.wait(m_mutex, [this]() {
            return !m_queue.empty();
        });
        // 从任务队列中取出任务(消费)
        int x = m_queue.front();
        m_queue.pop_front();
        // 通知生产者去生产
        m_notFull.notify_one();
        cout << x << " 被消费" << endl;
        return x;
    }

private:
    list<int> m_queue;     // 存储队列数据
    mutex m_mutex;         // 互斥锁
    condition_variable_any m_notEmpty;   // 不为空的条件变量
    condition_variable_any m_notFull;    // 没有满的条件变量
    int m_maxSize;         // 任务队列的最大任务个数
};

int main()
{
    SyncQueue taskQ(50);
    auto produce = bind(&SyncQueue::put, &taskQ, placeholders::_1);
    auto consume = bind(&SyncQueue::take, &taskQ);
    thread t1[3];
    thread t2[3];
    for (int i = 0; i < 3; ++i)
    {
        t1[i] = thread(produce, i + 100);
        t2[i] = thread(consume);
    }

    for (int i = 0; i < 3; ++i)
    {
        t1[i].join();
        t2[i].join();
    }

    return 0;
}
```

## 总结
总结：以上介绍的两种条件变量各自有各自的特点，`condition_variable` 配合 `unique_lock` 使用更灵活一些，可以在在任何时候自由地释放互斥锁，而`condition_variable_any` 如果和`lock_guard` 一起使用必须要等到其生命周期结束才能将互斥锁释放。但是，`condition_variable_any` 可以和多种互斥锁配合使用，应用场景也更广，而 `condition_variable` 只能和独占的非递归互斥锁（`mutex`）配合使用，有一定的局限性。

## 参考链接
[爱编程的大丙 C++线程同步之条件变量](https://subingwen.cn/cpp/condition/)