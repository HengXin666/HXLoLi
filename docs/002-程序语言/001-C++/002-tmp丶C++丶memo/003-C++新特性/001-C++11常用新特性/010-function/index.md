# std::function
`std::function`是 `C++11` 标准库中的一个通用函数封装类，它可以用来存储、拷贝和调用可调用对象（函数、函数指针、成员函数指针、函数对象等）。

## 基本使用方法

```C++
#include <functional>
std::function<返回值类型(参数类型列表)> diy_name = 可调用对象;
```
**注:**

使用`std::function<void()>`, 可以代表任意类型, 如:


```C++
#include <functional>
#include <iostream>

void taskFunction(int x, int y) {
    std::cout << "Task executed with parameters: " << x << ", " << y << std::endl;
}

int main() {
    std::function<void()> task = std::bind(taskFunction, 10, 20);
    task();
    return 0;
}
```

## std::bind 绑定器

> [!TIP]
> 2024年9月29日, 注:
>
> <span style="color:red">现在完全不需要这个鬼东西!, [Lambda表达式](../006-Lambda表达式/index.md) 就是它的上位替代! 职责重叠的!</span>
>
> 例如:

```C++
int func(int x, int y, int z, int &w);

int w = rand();

auto bound = std::bind(func, std::placeholders::_2, 1, std::placeholders::_1, std::ref(w)); //

int res = bound(5, 6); // 等价于 func(6, 1, 5, w);
```

可以写成:

```C++
int func(int x, int y, int z, int &w);

int w = rand();

auto lambda = [&w](int x, int y) { return func(y, 1, x, w); };

int res = lambda(5, 6);
```

> Lambda 表达式有许多优势：
> 
> -   简洁：不需要写一大堆看不懂的 `std::placeholders::_1`，直接写变量名就可以了。
> -   灵活：可以在 Lambda 中使用任意多的变量，调整顺序，而不仅仅是 `std::placeholders::_1`。
> -   易懂：写起来和普通函数调用一样，所有人都容易看懂。
> -   捕获引用：`std::bind` 不支持捕获引用，总是拷贝参数，必须配合 `std::ref` 才能捕获到引用。而 Lambda 可以随意捕获不同类型的变量，按值（`[x]`）或按引用（`[&x]`），还可以移动捕获（`[x = move(x)]`），甚至捕获 this（`[this]`）。
> -   夹带私货：可以在 lambda 体内很方便地夹带其他额外转换操作，比如：
>  	```C++
> 	auto lambda = [&w](int x, int y) { return func(y + 8, 1, x * x, ++w) * 2; };
> 	```

`std::bind`用来将可调用对象与其参数一起进行绑定。绑定后的结果可以使用`std::function`进行保存，并延迟调用到任何我们需要的时候。通俗来讲，它主要有两大作用：

1. 将可调用对象与其参数一起绑定成一个仿函数。
2. 将多元（参数个数为n，n>1）可调用对象转换为一元或者（n-1）元可调用对象，即只绑定部分参数。

### 语法

```C++
// 绑定非类成员函数/变量
auto f = std::bind(可调用对象地址, 绑定的参数/占位符);
// 绑定类成员函/变量
auto f = std::bind(类函数/成员地址, 类实例对象地址, 绑定的参数/占位符);
```

### 占位符
`placeholders::_1`是一个占位符，代表这个位置将在函数调用时被传入的第一个参数所替代。同样还有其他的占位符`placeholders::_2`、`placeholders::_3`、`placeholders::_4`、`placeholders::_5`等……


```C++
#include <iostream>
#include <functional>
using namespace std;

void output(int x, int y)
{
    cout << x << " " << y << endl;
}

int main(void)
{
    // 使用绑定器绑定可调用对象和参数, 并调用得到的仿函数
    bind(output, 1, 2)();
    bind(output, placeholders::_1, 2)(10);
    bind(output, 2, placeholders::_1)(10);

    // error, 调用时没有第二个参数
    // bind(output, 2, placeholders::_2)(10);
    // 调用时第一个参数10被吞掉了，没有被使用
    bind(output, 2, placeholders::_2)(10, 20);

    bind(output, placeholders::_1, placeholders::_2)(10, 20);
    bind(output, placeholders::_2, placeholders::_1)(10, 20);


    return 0;
}
```

输出:

```C++
1  2		// bind(output, 1, 2)();
10 2		// bind(output, placeholders::_1, 2)(10);
2 10		// bind(output, 2, placeholders::_1)(10);
2 20		// bind(output, 2, placeholders::_2)(10, 20);
10 20		// bind(output, placeholders::_1, placeholders::_2)(10, 20);
20 10		// bind(output, placeholders::_2, placeholders::_1)(10, 20);
```

通过测试可以看到，`std::bind`可以直接绑定函数的所有参数，也可以仅绑定部分参数。在绑定部分参数的时候，通过使用`std::placeholders`来决定空位参数将会属于调用发生时的第几个参数。

可调用对象包装器`std::function`是不能实现对类成员函数指针或者类成员指针的包装的，但是通过绑定器`std::bind`的配合之后，就可以完美的解决这个问题了:

```C++
#include <iostream>
#include <functional>
using namespace std;

class Test
{
public:
    void output(int x, int y)
    {
        cout << "x: " << x << ", y: " << y << endl;
    }
    int m_number = 100;
};

int main(void)
{
    Test t;
    // 绑定类成员函数
    function<void(int, int)> f1 = 
        bind(&Test::output, &t, placeholders::_1, placeholders::_2);
    // 绑定类成员变量(公共)
    function<int&(void)> f2 = bind(&Test::m_number, &t);

    // 调用
    f1(520, 1314);
    f2() = 2333;
    cout << "t.m_number: " << t.m_number << endl;

    return 0;
}
/*
在用绑定器绑定类成员函数或者成员变量的时候需要将它们所属的实例对象一并传递到绑定器函数内部。
f1的类型是function<void(int, int)>，通过使用std::bind将Test的成员函数output的地址和对象t绑定，并转化为一个仿函数并存储到对象f1中。

使用绑定器绑定的类成员变量m_number得到的仿函数被存储到了类型为function<int&(void)>的包装器对象f2中，并且可以在需要的时候修改这个成员。
其中int是绑定的类成员的类型，并且允许修改绑定的变量，因此需要指定为变量的引用，由于没有参数因此参数列表指定为void。

示例程序中是使用function包装器保存了bind返回的仿函数，如果不知道包装器的模板类型如何指定，可以直接使用auto进行类型的自动推导，这样使用起来会更容易一些。


作者: 苏丙榅
链接: https://subingwen.cn/cpp/bind/
来源: 爱编程的大丙
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。*/
```

## function 被 auto 替换后不能递归调用

考虑以下代码:

```C++
auto bfs = [&](int i) {
    if (!i)
        return 0;
    return bfs(i - 1) + 1;
};
```
编译器会报错: `E1586: 使用 auto 类型说明符声明的变量不能出现在其自身的初始值设定项中`

显然, 这样也不行:

```C++
function<void()> bfs = [&](int i) {
    if (!i)
        return 0;
    return bfs(i - 1) + 1;
};
```

因为这个是给绑定器使用的

故, 你必须要写全参数列表:
```C++
function<int(int)> bfs = [&](int i) {
    if (!i)
        return 0;
    return bfs(i - 1) + 1;
};
```

`auto dfs = [&](int i)`的写法，lambda里面包含dfs这个名字，所以为了推导lambda表达式的类型需要知道dfs这个名字的类型，而dfs这个名字是auto类型的，推导它的类型又需要知道右边的类型，也就是说使用auto接收lambda表达式，而lambda表达式里面又递归的话，会出现循环推导类型，所以不能编译。

而`function<void(int)> dfs = [&](int i)`不一样，dfs这个名字一开始就直接知道类型，虽然写在左边，但是推导右边的lambda表达式的类型时，dfs这个名字是可见且明确类型的，所以lambda表达式的类型也就是清楚的，类型推导不会遇到困难

- -> [lambda 中的递归调用 （C++11） [duplicate]](https://stackoverflow.com/questions/7861506/recursive-call-in-lambda-c11)

## 注解
### [1] 参考链接
[cpp中文文档 cppreference.com](https://zh.cppreference.com/w/cpp/utility/functional/function)

[爱编程的大丙-可调用对象包装器、绑定器](https://subingwen.cn/cpp/bind/)

[C++11的std::function源码解析](https://blog.csdn.net/weixin_43798887/article/details/116571325)