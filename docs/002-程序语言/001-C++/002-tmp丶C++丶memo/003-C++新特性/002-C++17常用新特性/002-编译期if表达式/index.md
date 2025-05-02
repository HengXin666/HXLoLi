# 编译期if表达式
## 说明
`C++17`引入了**编译期if表达式（constexpr if）**，它允许在编译期根据条件来选择不同的代码执行路径。这在模板元编程和泛型编程中非常有用。

以下是constexpr if的基本语法：

```C++
if constexpr (条件)
{
    // 在编译期满足条件时执行的代码
}
else
{
    // 在编译期不满足条件时执行的代码（可选）
}
```
注意，条件必须在`编译期`求值为 true 或 false

实际代码不会有`if`, 它在编译的时候就已经确定了要编译哪些代码了，类似于宏定义


## 示例
### [1]

```C++
#include <iostream>
using namespace std;

template <bool ok> 
constexpr void func2() {
	// 在编译期进行判断，if和else语句不生成代码
	if constexpr (ok == true) {
		// 当ok为true时，下面的else块不生成汇编代码
		cout << "ok" << endl;
	}
	else {
		// 当ok为false时，上面的if块不生成汇编代码
		cout << "not ok" << endl;
	}
}

int main() {
	func2<true>();  // 输出ok，并且汇编代码中只有 cout << "ok" << endl;
	func2<false>(); // 输出not ok，并且汇编代码中只有 cout << "not ok" << endl;
	return 0;
}
```
### [2]可变参数模版的包的展开

```C++
template<typename T, typename... Ts>
void syszuxPrint(T arg1, Ts... arg_left) {
    std::cout << arg1 << ", ";
    if constexpr(sizeof...(arg_left) > 0) {
        syszuxPrint(arg_left...);
    }
}

int main(int argc, char** argv)
{
    syszuxPrint(719, 7030, "civilnet");
}
```

注意: 去掉 `if constexpr` 的 `constexpr` 就会发生报错: 没有找到 重载函数(不传参时候的)

原因就是 可变参数的展开是在**编译时**候进行的， 所以可以通过编译期的判断来 判断是否需要再次调用 `syszuxPrint()` 函数.<sup>[2]</sup>

### [3]判断类型

```C++
template <typename T>
void PrintValue(T value)
{
	if constexpr (std::is_integral_v<T>)
	{
		std::cout << "整数类型：" << value << std::endl;
	}
	else if constexpr (std::is_floating_point_v<T>)
	{
		std::cout << "浮点类型：" << value << std::endl;
	}
	else
	{
		std::cout << "其他类型" << std::endl;
	}
}

PrintValue(10);      // 输出：整数类型：10
PrintValue(3.14);    // 输出：浮点类型：3.14
PrintValue("Hello"); // 输出：其他类型
```

## 区别: 宏定义的判断

与宏定义相比，编译期if表达式有以下几个不同之处：

- 可以在编译期间进行类型检查：使用编译期if表达式，我们可以根据模板参数 T 是否满足某些条件来选择不同的代码路径。这种方式在编译期进行类型检查，可以避免一些运行时错误。

- 更加`类型安全`：由于编译期if表达式是在编译期间进行求值的，因此它可以在类型安全的情况下进行条件判断。相比之下，宏定义只是简单地进行文本替换，可能会引入一些不必要的类型转换和难以发现的错误。

- 更加`可读性强`：使用编译期if表达式可以使代码更加清晰和易于阅读，因为它们可以直接嵌入到代码中，并且不需要使用繁琐的宏定义语法。

总的来说，与宏定义相比，编译期if表达式提供了更加可读性强、类型安全、灵活的条件判断机制，在模板元编程和泛型编程中非常有用。(By GPT-3.5)

## 注意
正如许多 C++ 著作中提到的, 如果一个函数中使用了大量的 if-else 语句,那么需要重构, 将多个条件语句转化成函数重载会更加清晰. 

同理,如果泛型函数中使用大量的 constexpr if 语句, 也许也需要考虑代码重构.

## 参考文献
### [1]
[C++17 之 "constexpr if"](https://blog.csdn.net/ding_yingzi/article/details/79977747)

[【C++札记】C++17 if-constexpr 提高代码可读性](https://zhuanlan.zhihu.com/p/380856254)(避免不用分支, 但类型错误, 导致编译错误的情况)

### [2]

[知乎:C++的可变参数模板](https://zhuanlan.zhihu.com/p/104450480)