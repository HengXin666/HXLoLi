# 三向比较运算符
## 说明
`C++20`引入的三向比较运算符（也叫: `三路比较运算符`, 即"`<=>`"运算符）是一个有用的新特性，它可以简化代码并提高效率。

在`C++11`之前，我们通常需要手动编写比较函数来实现对象间的比较，这会导致代码的冗长和重复。同时，为了实现一些数据结构（如排序算法），我们还需要手动编写多个比较函数（如"<"、">"、"<="、">="等），这也增加了代码的维护难度。

而使用三向比较运算符，我们只需要定义一个"<=>"运算符即可实现对象间的所有比较，这不仅可以简化代码，还可以提高代码的可读性和可维护性。

此外，三向比较运算符还可以提高代码的执行效率。因为它可以避免重复比较同一个对象，从而减少了比较次数，提高了程序的性能。

总之，三向比较运算符是一个非常有用和必要的新特性，它可以帮助我们更轻松、更高效地实现对象间的比较。

## 使用


三路比较结果如下
```C++
(a <=> b) < 0  // 如果 a < b 则为 true
(a <=> b) > 0  // 如果 a > b 则为 true
(a <=> b) == 0 // 如果 a 与 b 相等或者等价 则为 true
```
类似于C的strcmp 函数返回-1, 0, 1 一般情况: 自动生成所有的比较操作符, 如果对象是结构体则逐个比较, 可以用下面代码代替所有的比较运算符
```C++
auto X::operator<=>(const Y&) = default;
```
高级情况: 指定返回类型(支持6种所有的比较运算符)

## 示例
假设我们有一个自定义的Person类，其中包含姓名(name)和年龄(age)两个字段。在C++11之前，我们需要手动编写"<"、">"、"<="、">="等多个比较函数来实现Person对象间的比较。而使用三向比较运算符，我们只需要定义一个"<=>"运算符即可实现所有比较。

以下是一个使用三向比较运算符的示例代码：
```C++
#include <compare>
#include <iostream>
#include <string>

class Person {
public:
    std::string name;
    int age;

    auto operator<=>(const Person&) const = default;  // 使用默认的三向比较运算符
};

int main() {
    Person p1{"Alice", 20};
    Person p2{"Bob", 25};
    Person p3{"Charlie", 30};

    std::cout << (p1 <=> p2 == std::strong_ordering::less) << '\n';  // 输出1（true）
    std::cout << (p2 <=> p3 == std::strong_ordering::less) << '\n';  // 输出1（true）
    std::cout << (p1 <=> p3 == std::strong_ordering::less) << '\n';  // 输出1（true）

    return 0;
}
```
注: `std::strong_ordering::less`表示"<"运算符的含义，即如果p1小于p2，则输出true（即1）


`对比:` 以下是一个使用C++11之前的代码实现Person类的示例

```C++
#include <iostream>
#include <string>

class Person {
public:
    std::string name;
    int age;

    bool operator<(const Person& other) const {
        return age < other.age;
    }

    bool operator>(const Person& other) const {
        return age > other.age;
    }

    bool operator<=(const Person& other) const {
        return age <= other.age;
    }

    bool operator>=(const Person& other) const {
        return age >= other.age;
    }
};

int main() {
    Person p1{"Alice", 20};
    Person p2{"Bob", 25};
    Person p3{"Charlie", 30};

    std::cout << (p1 < p2) << '\n';  // 输出1（true）
    std::cout << (p2 < p3) << '\n';  // 输出1（true）
    std::cout << (p1 < p3) << '\n';  // 输出1（true）

    return 0;
}
```
## 参考
### [1]

[C++20详解：三向比较运算符](https://zhuanlan.zhihu.com/p/412135525)

[C++20 的三向比較（Three-way comparison）](https://blog.csdn.net/wlcs_6305/article/details/122726329)