# std::apply
## 说明
`std::apply` 是 `C++17` 中新增的标准库函数，它可以**将一个函数和一个参数包（parameter pack）一起调用**。这个参数包可以是任意数量的参数，不同类型和不同值。*(有点像Qt的槽函数<sup>[1]</sup>)*

## 示例

```C++
#include <iostream>
#include <tuple>

void print(int a, float b, double c) {
    std::cout << a << ", " << b << ", " << c << std::endl;
}

int main() {
    auto args = std::make_tuple(42, 3.14f, 2.71828);
    std::apply(print, args);
    return 0;
}
```

## 注解
### [1]
> 是不是有点相当于qt里面的槽函数?

是的，std::apply 在某种程度上类似于 Qt 中的槽函数。Qt 的槽函数是一种用于处理信号的特殊函数，可以接受任意数量和类型的参数。类似地，std::apply 也可以接受任意数量和类型的参数，并将它们作为参数包传递给目标函数。

然而，两者之间还是存在一些区别的。主要区别如下：

1. 语法和使用方式：Qt 的槽函数使用宏定义和信号槽连接来实现，而 std::apply 是 C++ 标准库提供的函数，用于直接调用函数并传递参数包。
2. 依赖关系：Qt 槽函数是 Qt 框架的一部分，需要依赖 Qt 库。而 std::apply 是 C++ 标准库的一部分，不需要任何额外的依赖。
3. 灵活性：std::apply 可以与任何函数一起使用，而 Qt 的槽函数主要用于与 Qt 的信号机制配合使用。

总的来说，std::apply 和 Qt 的槽函数都提供了处理任意数量和类型参数的能力，但它们的实现方式和使用场景有所不同。(By GPT-3.5)