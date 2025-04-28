# optional
## 前言
在 C 时代以及早期 C++ 时代，语法层面支持的 nullable 类型可以采用指针方式： T*，如果指针为 NULL （C++11 之后则使用 nullptr ） 就表示无值状态（empty value）。

在编程中，经常遇到这样的情况:可能返回/传递/使用某种类型的对象。也就是说，可以有某个类型的值，也可以没有任何值。因此，需要一种方法来模拟类似指针的语义，在指针中，可以使用nullptr来表示没有值。处理这个问题的方法是**定义一个特定类型的对象，并用一个额外的布尔成员/标志来表示值是否存在**。`std::optional<>`以一种类型安全的方式提供了这样的对象。

> [!TIP]
> 🚩**注意**： $每个版本可能对某些特征做了改动$.

## 说明

optional是一个模板类：
```C++
template <class T>
class optional;
```
它内部有两种状态，要么有值（`T类型`），要么没有值（`std::nullopt`）。*有点像T\*指针，要么指向一个T类型，要么是空指针(nullptr)。*

`std::optional`有以下几种构造方式：
```C++
#include <iostream>
#include <optional>
using namespace std;

int main() {
    optional<int> o1;           // 什么都不写时默认初始化为nullopt
    optional<int> o2 = nullopt; // 初始化为无值
	optional<int> o3(20);       // 构造函数
    optional<int> o3_1 = 10;    // 赋值构造 用一个T类型的值来初始化
    optional<int> o4 = o3;      // 用另一个optional来初始化
    return 0;
}
```

## 使用方法
### 查询是否有值
查看一个`optional对象`是否有值，可以直接用`if`，或者用`has_value()`

```C++
optional<int> o1;
if (o1) {
	printf("o1 has value\n");
}

if (o1.has_value()) {
	printf("o1 has value\n");
}
```

### 获取值
当一个`optional`**有值**时，可以通过用**指针的方式**(`*`号和`->`号)来使用它，或者用`.value()`拿到它的值

```C++
optional<int> o1 = 100;
cout << *o1 << endl;

optional<string> o2 = "orange";
cout << o2->c_str() << endl;
cout << o2.value().c_str() << endl;

// 使用方法的注意:
int value = myOptional.value();                // 如果optional对象为空，会抛出异常
int value = myOptional.value_or(defaultValue); // 如果optional对象为空，返回默认值 defaultValue (由程序员指定)
```

### 修改值
将一个有值的`optional`变为**无值**，用`.reset()`。该函数会将已存储的T类型对象**析构**掉

```C++
optional<int> o1 = 500;
o1 = 666;
o1.reset();
```
