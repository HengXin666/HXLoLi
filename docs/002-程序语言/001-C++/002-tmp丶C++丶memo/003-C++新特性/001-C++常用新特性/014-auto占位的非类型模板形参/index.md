# auto 占位的非类型模板形参
## 说明
使用 "auto" 作为非类型模板形参的占位符的目的是根据提供的模板实参进行类型推导。

通过在模板参数声明中使用 "auto"，编译器会根据模板实例化时提供的实参来确定参数的类型。这样可以提供灵活性，并使模板函数或类能够适用于不同的类型，而无需显式指定它们。

例如，使用 `template <auto Value>`，`Value` 参数可以推导为整数、浮点数、字符或其他任何类型，取决于提供的实参。这使得模板代码能够无缝处理不同的类型，并促进代码的可重用性

> 我暂时不知道在哪里可以使用它，或者贴近日常的使用场景，我必需它...

## 使用示例
### [1]

```C++
template <auto Value>
void print(int number) {
	cout << Value << endl;
	if (number == Value) {
		std::cout << "Match!" << std::endl;
	}
	else {
		std::cout << "No match." << std::endl;
	}
}


void new_cpp17_003(void)
{
	print<1>(1);   // Match
	print<1.1>(2); // No match
}
```

# auto 占位非类型模版形参 说明2.0

> 2024-12-29 更新

`template<auto val>` 可以推导为**任意类型**，只要该值满足**编译期常量**的要求。这意味着它不仅能推导内置类型（如 `int`、`double`、`char` 等），也能推导自定义类型，只要这些类型是常量表达式（`constexpr`）或在编译期可确定的值。

---

### 示例：推导自定义类型
以下是一个使用自定义类型作为非类型模板参数的示例：

```cpp
#include <iostream>

struct MyStruct {
    int a;
    constexpr MyStruct(int x) : a(x) {}
};

template <auto val>
void printValue() {
    std::cout << "Value: " << val.a << "\n";
}

int main() {
    constexpr MyStruct obj(42); // 编译期常量对象
    printValue<obj>();          // val 推导为 MyStruct 类型
    return 0;
}
```

#### 输出：
```
Value: 42
```

---

### 自定义类型的要求
为了能够作为非类型模板参数，自定义类型必须满足以下条件：
1. **编译期常量**：
   - 变量或值必须用 `constexpr` 修饰，表示在编译期可确定。
2. **可比较性**：
   - 自定义类型需要支持**按值比较**，因为模板实例化时可能需要比较该值是否相等。编译器要求类型具有 `operator==`。

---

### 推导指针或引用
指针或引用也可以作为模板参数，只要它们指向的对象或函数在编译期是常量。例如：

```cpp
#include <iostream>

struct MyStruct {
    int a;
};

template <auto val>
void printAddress() {
    std::cout << "Address: " << val << "\n";
}

int main() {
    constexpr int x = 42;
    constexpr MyStruct obj{10};

    printAddress<&x>();  // val 推导为 const int*
    printAddress<&obj>(); // val 推导为 const MyStruct*

    return 0;
}
```

#### 输出：
```
Address: 0x...  // x 的地址
Address: 0x...  // obj 的地址
```

---

### 常见应用场景
1. **标记类型或状态**：
   自定义类型常用于标记某些状态或行为。
   ```cpp
   struct ModeA {};
   struct ModeB {};

   template <auto Mode>
   void perform() {
       if constexpr (std::is_same_v<decltype(Mode), ModeA>)
           std::cout << "Mode A\n";
       else if constexpr (std::is_same_v<decltype(Mode), ModeB>)
           std::cout << "Mode B\n";
   }

   int main() {
       perform<ModeA>{}; // 输出 "Mode A"
       perform<ModeB>{}; // 输出 "Mode B"
   }
   ```

2. **实现动态元编程**：
   可结合编译期常量值来实现复杂逻辑，例如条件选择、分支优化。

---

### 注意事项
- C++20 开始支持类类型作为非类型模板参数，因此更加灵活。
- 非类型模板参数值**不能包含动态分配的内存**（如 `std::vector`），因为动态分配在编译期不可确定。
- 遇到自定义类型可能需要显式实现比较运算符。