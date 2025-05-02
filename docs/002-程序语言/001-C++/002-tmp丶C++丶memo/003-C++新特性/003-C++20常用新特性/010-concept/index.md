# concept
## 概念
`C++20`引入了一个重要的特性，即**概念（Concepts）**。概念使得我们能够在编写泛型代码时对类型进行约束，从而增强代码的可读性和类型安全性。

概念可以视为一种断言，用于描述类型应该具备的性质。通过定义概念，我们可以明确指定模板参数类型应该满足哪些条件。这样，在编写模板时，我们可以使用概念来约束模板参数，使得只有满足概念要求的类型才能被接受。

在`C++20`中，我们可以使用`requires`子句来定义概念。例如，下面是一个简单的概念定义：
```C++
template<typename T>
concept Integral = std::is_integral_v<T>;
```

这个概念名为`Integral`，它要求模板参数类型`T`必须是整数类型。在使用该概念时，我们可以使用`requires`子句来限制模板参数类型：
```C++
template<typename T>
void foo(T t) requires Integral<T> {
    // 只有满足Integral概念的类型才能进入该函数
}
```
这样，只有当传入的类型`T`是整数类型时，才能调用`foo`函数。

概念还可以进行组合和约束。我们可以通过逻辑运算符（如`&&`、`||`、`!`）来组合概念，以形成更复杂的约束条件。此外，我们还可以使用`requires`子句来指定其他的约束条件。

概念的引入使得模板代码更加清晰、易于理解和维护。它提供了一种在编译时对类型进行约束的机制，减少了运行时错误的可能性，并且提高了代码的可读性和可靠性。

需要注意的是，`C++20`中的概念特性仍然处于实验阶段，编译器的支持程度有限并且可能存在一些限制。在使用概念时，建议查阅编译器的文档以了解其支持情况。

## 示例

*示例By GPT-3.5*

假设我们有一个模板函数 max，用于取两个数中的最大值。我们可以使用概念来限制模板参数类型必须是可比较的类型，如下所示:
```C++
#include <compare>
#include <type_traits>

template <typename T>
concept Comparable = requires(T a, T b) {
  { a <=> b } -> std::convertible_to<std::strong_ordering>;
};

template <Comparable T>
constexpr const T& max(const T& a, const T& b) {
  return (a <=> b) == std::strong_ordering::greater ? a : b;
}
```

这里定义了一个概念 Comparable，要求模板参数类型必须满足以下条件：

- 支持小于等于操作符（<=）
- 返回值类型是 std::strong_ordering 类型

然后我们在 max 函数中使用该概念来限制参数类型，这样，只有满足 Comparable 概念的类型才能被使用于该函数。

接着，我们可以用两种方法来测试这个 max 函数。第一种是使用 int 类型作为参数，因为 int 类型支持小于等于操作符，所以它满足 Comparable 概念，可以成功编译通过：
```C++
int main() {
  constexpr int a = 1, b = 2;
  static_assert(max(a, b) == b);
  return 0;
}
```
另一种是使用 std::string 类型作为参数，因为 std::string 类型不支持小于等于操作符，所以它不满足 Comparable 概念，编译器会报错：

```C++
#include <string>

int main() {
  std::string a = "hello", b = "world";
  static_assert(max(a, b) == b); // 编译出错：no matching function for call to 'max'
  return 0;
}

```


## 注解
### 优秀参考:

[C++20: Concept详解以及个人理解](https://zhuanlan.zhihu.com/p/266086040)

[C++20 概念（concepts）入门](https://blog.csdn.net/lycheng1215/article/details/106149654)