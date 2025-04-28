# <chrono\>日期库
## 说明
`C++20`引入了一些新的特性和改进，其中包括对`<chrono>`头文件的扩展。 `<chrono>`提供了一组工具和类型，用于处理时间和持续时间的表示和计算。

在`C++20`中，`<chrono>`头文件添加了一些新的特性，包括以下几个方面：

- **新的时钟类型**：`C++20`引入了三个新的时钟类型：`std::utc_clock`、`std::tai_clock`和`std::gps_clock`。这些时钟类型可以提供更精确的时间测量和跟踪功能。

- **日期和时间调整器**：`C++20`引入了`std::chrono::year_month_day_last`、`std::chrono::year_month_day_weekday`和`std::chrono::year_month_weekday_last`等类型，用于表示一些常见的日期和时间调整器。

- **时钟转换**：`C++20`为时钟之间的转换提供了新的工具。例如，`std::chrono::clock_cast`函数可以用于将一个时钟类型的时间点转换为另一个时钟类型的时间点。

- **易用性改进**：`C++20`对`<chrono>`库进行了一些易用性改进。例如，引入了一些新的辅助类型和函数，使得处理日期和时间更加方便。

总体而言，`C++20`的`<chrono>`头文件在时间和持续时间的表示和计算方面提供了更多的功能和灵活性。这些改进使得在C++中处理时间相关的操作更加方便和高效。

> 本文只是抛砖引玉，需要使用请查阅文档!<sup>[1]</sup>

## 示例
### [1]

```C++
#include <iostream>
#include <chrono>
using namespace std;
using namespace std::chrono;

int main()
{
    // creating a year
    auto y1 = year{ 2019 };
    auto y2 = 2019y; // ^[2]:为什么可以自定义 'y'
  
    // creating a mouth
    auto m1 = month{ 9 };
    auto m2 = September;
  
    // creating a day
    auto d1 = day{ 18 };
    auto d2 = 18d;
    year_month_day date1{ 2022y,July, 21d };
    auto date2 = 2022y / July / 21d;
    chrono::year_month_day date3{ Monday[3] / July / 2022 };
  
    cout << date1 << endl;
    cout << date2 << endl;
    cout << date3 << endl;
    return 0;
}
```

## 注解
### [1]

[知乎: 再也不被时间束缚：C++ stdchrono时间库全面解析](https://zhuanlan.zhihu.com/p/662738124)

[C++官方文档](https://zh.cppreference.com/w/cpp/chrono)

### [2]
`C++20`引入了一种新的**用户定义字面量（User-Defined Literal，简称UDL）** 语法，可以通过在数字后面加上一些后缀来创建自定义类型的字面量。其中，y后缀用于创建`std::chrono::year`类型的字面量。

在代码中，2019y使用了这个新的字面量语法，它等价于std::chrono::year{ 2019 }。因此，这两个赋值语句是等价的：

```C++
auto y1 = year{ 2019 };
auto y2 = 2019y; // 这个 y
```
---
$如何创建?$

要定义一个可以使用自定义字面量的类型，你需要进行以下几个步骤：

在命名空间中定义一个字面量运算符函数，用于将字面量转换为你的类型。字面量运算符函数的名称必须以operator""开头，后面跟着你选择的后缀。它可以是全局函数或者在类内部定义。

字面量运算符函数的参数通常是一个无符号长整型（unsigned long long）和一个类型参数（如const char*），用于接收字面量的值和长度。

在字面量运算符函数中，你可以根据传入的值和类型参数创建并初始化你的类型的对象，并将其返回。

下面是一个示例，展示了如何定义一个简单的类型MyLiteralType，并为其定义一个后缀为_my的字面量运算符函数：

```C++
#include <iostream>

struct MyLiteralType {
    int value;

    MyLiteralType(int val) : value(val) {}
};

MyLiteralType operator""_my(unsigned long long val) {
    return MyLiteralType(static_cast<int>(val));
}

int main() {
    MyLiteralType obj = 42_my;
    std::cout << "Value: " << obj.value << std::endl;

    return 0;
}
```

在这个示例中，我们定义了一个MyLiteralType结构体，并为其定义了一个后缀为_my的字面量运算符函数。这个函数接收一个无符号长整型参数val，并将其转换为MyLiteralType对象。在main函数中，我们使用42_my这个字面量初始化了一个MyLiteralType对象，并将其打印出来。

值得注意的是，自定义的字面量后缀不应与标准库或其他库中已有的后缀冲突，以避免产生歧义和不一致的行为。同时，还应该谨慎选择后缀名称，使其能够清晰地表达出你的类型的含义。(By GPT-3.5)