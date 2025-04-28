# ＜charconv＞头文件
## 介绍
`<charconv>`是`C++17`新的标准库头文件，包含了相关类和两个转换函数。**可以完成传统的整数/浮点和字符串互相转换的功能(atoi、itoa、atof、sprintf等)，同时支持输出格式控制、整数基底设置并且将整数和浮点类型对字符串的转换整合了起来**。是独立于<b style="color:red">本地环境、不分配、不抛出的</b>。目的是在常见的`高吞吐量环境`，例如基于文本的交换（ JSON 或 XML ）中，允许尽可能快的实现。

## 为什么这么快
`<charconv>`头文件中的函数相对于以前的转换函数，有以下几点优势，可以让它们更快：

1. 不需要创建和操作中间缓冲区。
> 传统方法通常将数字类型转换为字符串时，需要使用中间缓冲区（例如std::stringstream），这会增加内存分配和复制的开销。而`<charconv>`中的to_chars函数可以直接写入给定的输出缓冲区，避免了中间缓冲区的使用。

2. 执行更少的动态内存分配。
> 在传统方法中，字符串流等类需要进行动态内存分配，这会导致性能下降和额外的开销。而`<charconv>`中的函数可以通过提供用户指定的输出缓冲区来避免动态内存分配。

3. 更少的字符解码，在解析十进制数时能够做到全局最小化的字符扫描。
> 传统方法中的解析器通常需要多次扫描输入字符序列以确定数字类型的结构。而`<charconv>`中的from_chars函数可以根据数字类型的格式执行更少的字符解码，从而更快地识别数字类型并将其转换为目标类型。

因此，`<charconv>`头文件中的函数能够更快速、更高效地进行数字类型和字符串之间的转换，从而提高了程序的性能。(By GPT-3.5)

## 使用示例

1. 使用std::from_chars将字符串转换为整数


```C++
#include <charconv>
#include <iostream>

int main() {
    const char* str = "12345";
    int value = 0;
  
    auto result = std::from_chars(str, str + strlen(str), value); // 将字符串转换为整数

    if (result.ec == std::errc()) {
        std::cout << "转换成功，值为: " << value << std::endl;
    } else {
        std::cout << "转换失败" << std::endl;
    }

    return 0;
}
```

2. 使用std::to_chars将整数转换为字符串

```C++
#include <charconv>
#include <iostream>

int main() {
    int value = 12345;
    char buffer[20];
  
    auto result = std::to_chars(buffer, buffer + sizeof(buffer), value); // 将整数转换为字符串

    if (result.ec == std::errc()) {
        *result.ptr = '\0'; // 添加字符串结尾的空字符
        std::cout << "转换成功，字符串为: " << buffer << std::endl;
    } else {
        std::cout << "转换失败" << std::endl;
    }

    return 0;
}
```

### 综合使用

```C++
#include <iostream>
#include <charconv>

using namespace std;

int main() {
    string s1{ "123456789" };
    int val = 0;
    auto res = from_chars(s1.data(), s1.data() + 4, val); // 把s1的前4个转成整数
    if (res.ec == errc()) {
        cout << "val: " << val << ", distance: " << res.ptr - s1.data() << endl; // val: 1234, distance: 4
    }
    else if (res.ec == errc::invalid_argument) {
        cout << "invalid" << endl;
    }

    s1 = string{ "12.34" };
    double value = 0;
    auto format = chars_format::general;
    res = from_chars(s1.data(), s1.data() + s1.size(), value, format); // 把"12.34"转成小数
    cout << "value: " << value << endl; // value: 12.34

    s1 = string{ "xxxxxxxx" };
    int v = 1234;
    auto result = to_chars(s1.data(), s1.data() + s1.size(), v); // 把整数转成字符串
    cout << "str: " << s1 << ", filled: " << result.ptr - s1.data() << " characters." << endl; // s1: 1234xxxx, filled: 4 characters.

    return 0;
}
```

## 粗略的参数解析
1. std::from_chars函数原型：

```C++
template< class InputIt, class T >
constexpr std::from_chars_result from_chars( InputIt first, InputIt last, T& value );
```

- InputIt: 输入字符序列的迭代器类型（例如const char*或std::string::iterator）。
- first: 输入字符序列的起始迭代器。
- last: 输入字符序列的结束迭代器。
- T: 目标类型，即要将字符序列转换为的数字类型。
- value: 对目标类型的引用，用于存储转换后的结果。
- 返回值: std::from_chars_result结构体，包含转换结果和错误码。

---
2. std::to_chars函数原型：

```C++
template< class OutputIt, class T >
constexpr std::to_chars_result to_chars( OutputIt first, OutputIt last, const T& value );
```

- OutputIt: 输出字符串的迭代器类型（例如char*或std::string::iterator）。
- first: 输出字符串的起始迭代器。
- last: 输出字符串的结束迭代器。
- T: 要从数字类型转换为字符串的值类型。
- value: 要转换的数字类型的值。
- 返回值: std::to_chars_result结构体，包含转换结果和错误码。


## 注解
### [1]
参考文献:

[C++新特性学习笔记--＜charconv＞](https://blog.csdn.net/miaoaa2008/article/details/113858871)

[C++中文参考手册–＜charconv＞](https://zh.cppreference.com/w/cpp/header/charconv)