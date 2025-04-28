# 原子变量
`C++11`提供了一个原子类型`std::atomic<T>`，通过这个原子类型管理的内部变量就可以称之为原子变量，我们可以给原子类型指定`bool`、`char`、`int`、`long`、`指针`等类型作为模板参数（*不支持浮点类型和复合类型*）。

原子指的是一系列不可被CPU上下文交换的机器指令，这些指令组合在一起就形成了原子操作。在多核CPU下，当某个CPU核心开始运行原子操作时，会先暂停其它CPU内核对内存的操作，以保证原子操作不会被其它CPU内核所干扰。

由于原子操作是通过指令提供的支持，因此它的性能相比锁和消息传递会好很多。相比较于锁而言，原子类型不需要开发者处理加锁和释放锁的问题，同时支持修改，读取等操作，还具备较高的并发性能，几乎所有的语言都支持原子类型。

可以看出原子类型是无锁类型，但是无锁不代表无需等待，因为原子类型内部使用了CAS循环，当大量的冲突发生时，该等待还是得等待！但是总归比锁要好。

`C++11`内置了整形的原子变量，这样就可以更方便的使用原子变量了。在多线程操作中，使用原子变量之后就不需要再使用互斥量来保护该变量了，用起来更简洁。<span style="color:red">因为对原子变量进行的操作只能是一个原子操作（`atomic operation`），原子操作指的是不会被线程调度机制打断的操作，这种操作一旦开始，就一直运行到结束，中间不会有任何的上下文切换。</span>多线程同时访问共享资源造成数据混乱的原因就是因为CPU的上下文切换导致的，使用原子变量解决了这个问题，因此互斥锁的使用也就不再需要了。

> [!TIP]
> CAS全称是Compare and swap, 它通过一条指令读取指定的内存地址，然后判断其中的值是否等于给定的前置值，如果相等，则将其修改为新的值

## atomic 类成员
### 类定义

```C++
// 定义于头文件 <atomic>
template< class T >
struct atomic;
```
通过定义可得知：在使用这个模板类<sup>[2]</sup>的时候，一定要指定模板类型。

### 构造函数

```C++
// ①
atomic() noexcept = default;
// ②
constexpr atomic( T desired ) noexcept;
// ③
atomic( const atomic& ) = delete;
```
- 构造函数①：默认无参构造函数。
- 构造函数②：使用 `desired` 初始化原子变量的值。
- 构造函数③：使用`=delete`显示删除拷贝构造函数, 不允许进行对象之间的拷贝

### 公共成员函数
#### operator=
原子类型在类内部重载了`=`操作符，并且**不允许**在类的外部使用`=`进行对象的拷贝。

```C++
T operator=( T desired ) noexcept;
T operator=( T desired ) volatile noexcept;

atomic& operator=( const atomic& ) = delete;
atomic& operator=( const atomic& ) volatile = delete;
```

#### store()
原子地以 `desired` **替换当前值**。按照 `order` 的值影响内存。

```C++
void store( T desired, std::memory_order order = std::memory_order_seq_cst ) noexcept;
void store( T desired, std::memory_order order = std::memory_order_seq_cst ) volatile noexcept;
```

- `desired`：存储到原子变量中的值
- `order`：强制的内存顺序

#### load()

原子地加载并**返回原子变量的当前值**。按照 `order` 的值影响内存。直接访问原子对象也可以得到原子变量的当前值。

```C++
T load( std::memory_order order = std::memory_order_seq_cst ) const noexcept;
T load( std::memory_order order = std::memory_order_seq_cst ) const volatile noexcept;
```

### 特化成员函数
复合赋值运算符重载，主要包含以下形式：
|模版类型|code|
|:-|:-|
|模板类型T为整形|T operator+= (T val) volatile noexcept;<br>T operator+= (T val) noexcept;<br>T operator-= (T val) volatile noexcept;<br>T operator-= (T val) noexcept;<br>T operator&= (T val) volatile noexcept;<br>T operator&= (T val) noexcept;<br>T operator\|= (T val) volatile noexcept;<br>T operator\|= (T val) noexcept;<br>T operator^= (T val) volatile noexcept;<br>T operator^= (T val) noexcept;<br>
|模板类型T为指针|T operator+= (ptrdiff_t val) volatile noexcept;<br>T operator+= (ptrdiff_t val) noexcept;<br>T operator-= (ptrdiff_t val) volatile noexcept;<br>T operator-= (ptrdiff_t val) noexcept;<br>

以上各个 `operator` 都会有对应的 `fetch_*` 操作，详细见下表：

|操作符|操作符重载函数|等级的成员函数|整形|指针|其他|
|:-:|:-:|:-:|:-:|:-:|:-:|
|+|atomic::operator+=|<span style="color:red">atomic::fetch_add|是|是|否|
|-|atomic::operator-=|<span style="color:red">atomic::fetch_sub|是|是|否|
|&|atomic::operator&=|<span style="color:red">atomic::fetch_and|是|否|否|
|\||atomic::operator\|=|<span style="color:red">atomic::fetch_or|是|否|否|
|^|atomic::operator^=|<span style="color:red">atomic::fetch_xor|是|否|否|

### 内存顺序约束
通过上面的 API 函数我们可以看出，在调用`atomic`类提供的 API 函数的时候，需要指定原子顺序，在`C++11`给我们提供的 API 中使用枚举用作执行原子操作的函数的实参，以指定如何同步不同线程上的其他操作。

定义如下:

```C++
typedef enum memory_order { // --------------          // 简单注解
    memory_order_relaxed,   // relaxed                 // 不约束
    memory_order_release,   // release                 // [释放]
    memory_order_acquire,   // acquire                 // [获取]
    memory_order_consume,   // consume                 // [获取·改]
    memory_order_acq_rel,   // acquire/release         // [释放 + 获取]
    memory_order_seq_cst    // sequentially consistent // [顺序一致性]
} memory_order;
```
- `memory_order_relaxed`， 这是最宽松的规则，它对编译器和CPU不做任何限制，可以乱序
- `memory_order_release` **释放**，设定**内存屏障(Memory barrier)**，保证它之前的操作永远在它之前，但是它后面的操作可能被重排到它前面
- `memory_order_acquire` **获取**, 设定内存屏障，保证在它之后的访问永远在它之后，但是它之前的操作却有可能被重排到它后面，往往和`Release`在不同线程中联合使用
- `memory_order_consume`：改进版的`memory_order_acquire`，开销更小
- `memory_order_acq_rel`，它是`Acquire`和`Release`的结合，同时拥有它们俩提供的保证。比如你要对一个`atomic`自增 1，同时希望该操作之前和之后的读取或写入操作不会被重新排序
- `memory_order_seq_cst` **顺序一致性**， `memory_order_seq_cst` 就像是`memory_order_acq_rel`的加强版，它不管原子操作是属于读取还是写入的操作，只要某个线程有用到`memory_order_seq_cst`的原子操作，线程中该`memory_order_seq_cst`操作前的数据操作绝对不会被重新排在该`memory_order_seq_cst`操作之后，且该`memory_order_seq_cst`操作后的数据操作也绝对不会被重新排在`memory_order_seq_cst`操作前。

### C++20新增成员
在`C++20`版本中添加了新的功能函数，可以通过原子类型来**阻塞线程**，和`条件变量`中的`等待/通知函数`是一样的。

|公共成员函数|说明|
|:-|:-|
|wait`(C++20)`|阻塞线程直至被提醒且原子值更改|
|notify_one`(C++20)`|通知（唤醒）至少一个在原子对象上阻塞的线程|
|notify_all`(C++20)`|通知（唤醒）所有在原子对象上阻塞的线程|

### 类别名
|别名|原始类型定义|
|:-|:-|
|atomic_bool(C++11)|`std::atomic<bool>`|
|atomic_char(C++11)|`std::atomic<char>`|
|atomic_schar(C++11)|`std::atomic<signed char>`|
|atomic_uchar(C++11)|`std::atomic<unsigned char>`|
|atomic_short(C++11)|`std::atomic<short>`|
|atomic_ushort(C++11)|`std::atomic<unsigned short>`|
|atomic_int(C++11)|`std::atomic<int>`|
|atomic_uint(C++11)|`std::atomic<unsigned int>`|
|atomic_long(C++11)|`std::atomic<long>`|
|atomic_ulong(C++11)|`std::atomic<unsigned long>`|
|atomic_llong(C++11)|`std::atomic<long long>`|
|atomic_ullong(C++11)|`std::atomic<unsigned long long>`|
|atomic_char8_t(C++20)|`std::atomic<char8_t>`|
|atomic_char16_t(C++11)|`std::atomic<char16_t>`|
|atomic_char32_t(C++11)|`std::atomic<char32_t>`|
|atomic_wchar_t(C++11)|`std::atomic<wchar_t>`|
|atomic_int8_t(C++11)(可选)|`std::atomic<std::int8_t>`|
|atomic_uint8_t(C++11)(可选)|`std::atomic<std::uint8_t>`|
|atomic_int16_t(C++11)(可选)|`std::atomic<std::int16_t>`|
|atomic_uint16_t(C++11)(可选)|`std::atomic<std::uint16_t>`|
|atomic_int32_t(C++11)(可选)|`std::atomic<std::int32_t>`|
|atomic_uint32_t(C++11)(可选)|`std::atomic<std::uint32_t>`|
|atomic_int64_t(C++11)(可选)|`std::atomic<std::int64_t>`|
|atomic_uint64_t(C++11)(可选)|`std::atomic<std::uint64_t>`|
|atomic_int_least8_t(C++11)|`std::atomic<std::int_least8_t>`|
|atomic_uint_least8_t(C++11)|`std::atomic<std::uint_least8_t>`|
|atomic_int_least16_t(C++11)|`std::atomic<std::int_least16_t>`|
|atomic_uint_least16_t(C++11)|`std::atomic<std::uint_least16_t>`|
|atomic_int_least32_t(C++11)|`std::atomic<std::int_least32_t>`|
|atomic_uint_least32_t(C++11)|`std::atomic<std::uint_least32_t>`|
|atomic_int_least64_t(C++11)|`std::atomic<std::int_least64_t>`|
|atomic_uint_least64_t(C++11)|`std::atomic<std::uint_least64_t>`|
|atomic_int_fast8_t(C++11)|`std::atomic<std::int_fast8_t>`|
|atomic_uint_fast8_t(C++11)|`std::atomic<std::uint_fast8_t>`|
|atomic_int_fast16_t(C++11)|`std::atomic<std::int_fast16_t>`|
|atomic_uint_fast16_t(C++11)|`std::atomic<std::uint_fast16_t>`|
|atomic_int_fast32_t(C++11)|`std::atomic<std::int_fast32_t>`|
|atomic_uint_fast32_t(C++11)|`std::atomic<std::uint_fast32_t>`|
|atomic_int_fast64_t(C++11)|`std::atomic<std::int_fast64_t>`|
|atomic_uint_fast64_t(C++11)|`std::atomic<std::uint_fast64_t>`|
|atomic_intptr_t(C++11)(可选)|`std::atomic<std::intptr_t>`|
|atomic_uintptr_t(C++11)(可选)|`std::atomic<std::uintptr_t>`|
|atomic_size_t(C++11)|`std::atomic<std::size_t>`|
|atomic_ptrdiff_t(C++11)|`std::atomic<std::ptrdiff_t>`|
|atomic_intmax_t(C++11)|`std::atomic<std::intmax_t>`|
|atomic_uintmax_t(C++11)|`std::atomic<std::uintmax_t>`|

## 原子变量的使用
假设我们要制作一个多线程交替数数的计数器，我们使用互斥锁和原子变量的方式分别进行实现，对比一下二者的差异：
### 互斥锁版本

```C++
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
using namespace std;

struct Counter
{
    void increment()
    {
        for (int i = 0; i < 10; ++i)
        {
            lock_guard<mutex> locker(m_mutex);
            m_value++;
            cout << "increment number: " << m_value 
                << ", theadID: " << this_thread::get_id() << endl;
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

    void decrement()
    {
        for (int i = 0; i < 10; ++i)
        {
            lock_guard<mutex> locker(m_mutex);
            m_value--;
            cout << "decrement number: " << m_value 
                << ", theadID: " << this_thread::get_id() << endl;
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

    int m_value = 0;
    mutex m_mutex;
};

int main()
{
    Counter c;
    auto increment = bind(&Counter::increment, &c);
    auto decrement = bind(&Counter::decrement, &c);
    thread t1(increment);
    thread t2(decrement);

    t1.join();
    t2.join();

    return 0;
}
```

### 原子变量版本

```C++
#include <iostream>
#include <thread>
#include <atomic>
#include <functional>
using namespace std;

struct Counter
{
    void increment()
    {
        for (int i = 0; i < 10; ++i)
        {
            m_value++;
            cout << "increment number: " << m_value
                << ", theadID: " << this_thread::get_id() << endl;
            this_thread::sleep_for(chrono::milliseconds(500));
        }
    }

    void decrement()
    {
        for (int i = 0; i < 10; ++i)
        {
            m_value--;
            cout << "decrement number: " << m_value
                << ", theadID: " << this_thread::get_id() << endl;
            this_thread::sleep_for(chrono::milliseconds(500));
        }
    }
    // atomic<int> == atomic_int
    atomic_int m_value = 0;
};

int main()
{
    Counter c;
    auto increment = bind(&Counter::increment, &c);
    auto decrement = bind(&Counter::decrement, &c);
    thread t1(increment);
    thread t2(decrement);

    t1.join();
    t2.join();

    return 0;
}
```

通过代码的对比可以看出，使用了原子变量之后，就不需要再定义互斥量了，在使用上更加简便，并且这两种方式都能保证在多线程操作过程中数据的正确性，不会出现数据的混乱。

原子类型`atomic<T>`可以封装原始数据最终得到一个原子变量对象，操作原子对象能够得到和操作原始数据一样的效果，当然也可以通过`store()`和`load()`来读写原子对象内部的原始数据。

值得注意的是

`m_value += m_value + 1;`并不是原子的! 它实际上包含了多个操作，包括读取 `m_value` 的值、进行加法运算、以及将结果赋值给 `m_value`。这些操作之间可能会被其他线程的操作所干扰，导致结果不符合预期。


## 拓展阅读
[【C/C++ 原子操作】深入浅出：从互斥锁到无锁编程的转变 - 理解C++原子操作和内存模型](https://www.zhihu.com/question/469476598)

[C++ 中，std::atomic 是真正的「原子」吗？](https://www.zhihu.com/question/469476598)

## 注解
### [1]
本文是[爱编程的大丙 - 原子变量](https://subingwen.cn/cpp/atomic/)的学习笔记

### [2]
在C++标准库中，`std::atomic` 模板是用于操作原子类型的模板类，其中 T 可以是任何标量类型，包括基本数据类型（如整数、浮点数）和指针类型。

尽管在模板定义中使用了 `struct` 关键字，但在C++中，结构体和类（class）之间的区别并不大。实际上，`struct` 和 `class` 的主要区别在于默认的访问权限：在 `struct` 中，默认的成员访问权限为公有（`public`）；而在 `class` 中，默认的成员访问权限为私有（`private`）。除此之外，它们在其他方面是相似的，包括可以拥有成员函数、继承等特性。

因此，虽然 `std::atomic` 在定义时使用了 `struct` 关键字，但它实际上是一个类模板，可以用于创建原子操作的各种类型，而且你可以像使用类一样使用它。例如，你可以声明 `std::atomic<int>`、`std::atomic<double>` 等类型的原子变量，并对其进行原子操作。