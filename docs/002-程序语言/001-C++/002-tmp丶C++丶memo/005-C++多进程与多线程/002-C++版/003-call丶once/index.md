# call_once
在某些特定情况下，某些函数只能在多线程环境下调用一次，比如：要初始化某个对象，而这个对象只能被初始化一次，就可以使用`std::call_once()`来保证函数在多线程环境下只能被调用一次。使用`call_once()`的时候，需要一个`once_flag`作为`call_once()`的传入参数，该函数的原型如下:

```C++
// 定义于头文件 <mutex>
template< class Callable, class... Args >
void call_once( std::once_flag& flag, Callable&& f, Args&&... args );
```
参数:
- `flag`：once_flag类型的对象，要保证这个对象能够被多个线程同时访问到
- `f`：回调函数，可以传递一个有名函数地址，也可以指定一个匿名函数
- `args`：作为实参传递给回调函数

多线程操作过程中，std::call_once()内部的回调函数只会被执行一次，示例代码如下:

```C++
#include <iostream>
#include <thread>
#include <mutex>
using namespace std;

once_flag g_flag;
void do_once(int a, string b)
{
    cout << "name: " << b << ", age: " << a << endl;
}

void do_something(int age, string name)
{
    static int num = 1;
    call_once(g_flag, do_once, 19, "luffy");
    cout << "do_something() function num = " << num++ << endl;
}

int main()
{
    thread t1(do_something, 20, "ace");
    thread t2(do_something, 20, "sabo");
    thread t3(do_something, 19, "luffy");
    t1.join();
    t2.join();
    t3.join();

    return 0;
}
```
示例程序输出的结果:

```C++
name: luffy, age: 19
do_something() function num = 1
do_something() function num = 2
do_something() function num = 3
```
通过输出的结果可以看到，虽然运行的三个线程中都执行了任务函数`do_something()`但是`call_once()`中指定的回调函数只被执行了一次，我们的目的也达到了。

## 参考链接
[爱编程的大丙 call_once](https://subingwen.cn/cpp/call_once/)