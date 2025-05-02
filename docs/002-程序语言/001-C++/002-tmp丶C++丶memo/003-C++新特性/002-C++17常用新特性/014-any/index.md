# std::any
## 概述
在C++11中引入的auto自动推导类型变量大大方便了编程，但是auto变量一旦声明，该变量类型不可再改变。

`C++17`中引入了`std::any`类型，该类型变量可以存储 $任何类型$ 的值，
也可以`时刻改变它的类型`，类似于python中的变量。

## 使用方法
### 包含头文件

```C++
#include <any>
```
### 创建std::any对象
可以使用默认构造函数创建一个空的std::any对象，或者使用模板构造函数创建一个存储特定类型的std::any对象

```C++
std::any value; // 创建一个空的std::any对象 (类型是 void)

int number = 42;
std::any value2(number); // 创建一个存储整数类型的std::any对象
```

### 存储和访问值
可以使用赋值运算符或者模板赋值运算符将值存储到std::any对象中，并使用any_cast函数访问存储的值

```C++
std::any value;

int number = 42;
value = number; // 存储一个整数值

int* ptr = std::any_cast<int>(&value); // 将存储的整数值转换为int指针
if (ptr) {
    std::cout << "Value: " << *ptr << std::endl;
} else {
    std::cout << "Value is not an int." << std::endl;
}
```
> 注意：在使用any_cast函数进行类型转换时，需要确保转换后的类型与实际存储的类型匹配，否则会返回nullptr

### 查看当前的类型

```C++
std::any b;
cout << b.type().name() << endl; // void
```


### 检查是否存储了值
可以使用has_value函数检查std::any对象是否存储了值

```C++
std::any value;

if (value.has_value()) {
    std::cout << "Value is stored." << std::endl;
} else {
    std::cout << "Value is not stored." << std::endl;
}
```

### 重置和交换值
可以使用reset函数将std::any对象重置为空对象，使用swap函数交换两个std::any对象的值


```C++
std::any value1 = 42;
std::any value2 = "Hello";

value1.reset(); // 将value1重置为空对象

value1.swap(value2); // 交换value1和value2的值
```
