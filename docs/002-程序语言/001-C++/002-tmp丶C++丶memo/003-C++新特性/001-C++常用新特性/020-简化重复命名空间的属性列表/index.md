# 简化重复命名空间的属性列表
## 概述

在C++中，**属性列表（Attribute List）**<sup>[1]</sup> 是一种用于给代码中的声明或定义添加额外信息或元数据的机制。属性列表可以包含各种属性，这些属性可以提供编译器指令、优化提示、静态分析等信息，从而影响代码的行为和处理方式。

属性列表使用方括号（[]）来表示，放置在声明或定义语句的开头或末尾。属性列表中的每个属性都由一个标识符、可能的参数和一个可选的参数列表组成。属性列表可以单独使用，也可以一次使用多个属性。

以下是一个使用属性列表的示例：

```C++
[[deprecated("这个函数已经被废弃了")]]
static void oldFunction() {};

[[nodiscard]] int calculate() { return 0; };

[[maybe_unused]] void unusedFunction() {};

void new_cpp17_004(void)
{
	oldFunction();    // 编译器报错: 这个函数已经被废弃了
	calculate();      // 编译器警告: 返回值被忽略
	unusedFunction(); // 一个可能未被使用的函数，编译器会发出警告但不会报错
}
```

在上面的示例中，`[[deprecated]]` 属性标记了函数 `oldFunction()` 已经被废弃，编译器会发出警告；`[[nodiscard]]` 属性标记了函数 `calculate()` 的返回值不能被忽略，如果被忽略，编译器会发出警告；`[[maybe_unused]]` 属性标记了函数 `unusedFunction()` 是一个可能未被使用的函数，编译器会发出警告但不会报错。

值得注意的是，属性列表的具体语法和支持的属性取决于编译器的实现和标准的版本。不同的编译器可能会支持不同的属性，并且可能有自定义的属性扩展。因此，建议查阅编译器的文档以了解具体的属性列表语法和支持的属性。

## C++17简化
[[ using `命名空间X` : `属性1(参数列表),...属性n(参数列表)`]] (C++17 起)


```C++
// C++11
[[gnu::always_inline]] [[gnu::hot]] [[gnu::const]] [[nodiscard]]
inline int f(); // 声明 f 带四个属性

[[gnu::always_inline, gnu::const, gnu::hot, nodiscard]]
int f(); // 同上，但使用含有四个属性的单个属性说明符

// C++17
[[using gnu:const, always_inline, hot]] [[nodiscard]]
int f [[gnu::always_inline]] (); // 属性可出现于多个说明符中

int f() {return 0;}
```
## C++17新增属性
### [[fallthrough]]
switch语句中跳到下一条语句，不需要break，让编译器忽略警告。

```C++
int i = 1;
int result;
switch (i) {
case 0:
	result = 1; // warning
case 1:
	result = 2;
	[[fallthrough]]; // no warning
default:
	result = 0;
	break;
}
```

### [[nodiscard]]
所修饰的内容`不可被忽略`，主要用于修饰函数`返回值`

当用于描述函数的返回值时，如果调用函数的地方没有获取返回值时，编译器会给予`警告`

```C++
[[nodiscard]] auto func(int a, int b) {
	return a + b; 
}

int main() {
    func(2, 3); // 警告
    return 0;
}
```

### [[maybe_unused]]
用于描述`暂时没有被使用`的**函数或变量**，以**避免**编译器对此发出**警告**

```C++
[[maybe_unused]] void func()      //没有被使用的函数
{
    cout << "test" << endl;
}

int main()
{
	[[maybe_unused]] int num = 0; //没有被使用的变量
	return 0;
}
```


## 注解
### [1]

[谈谈C++新标准带来的属性（Attribute）](https://zhuanlan.zhihu.com/p/392460397)

### [2]

更多属性 [c++新特性实验(5)声明与定义:属性列表（C++11 起）](https://www.cnblogs.com/sjjg/p/11204502.html)