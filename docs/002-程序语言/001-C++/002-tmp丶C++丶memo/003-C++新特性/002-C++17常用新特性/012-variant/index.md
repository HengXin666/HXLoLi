# std::variant
## 描述
`C++17`中提供了`std::variant`类型，意为多变的，可变的类型。

有点类似于**加强版的union，里面可以存放复合数据类型，且操作元素更为方便**。

可以用于表示`多种类型的混合体`，但`同一时间`只能用于代表一种类型的实例。

`variant`提供了`index`成员函数，该函数返回一个索引，该索引用于表示`variant`定义对象时模板参数的索引(起始索引为0)，同时提供了一个函数`holds_alternative<T>(v)`用于查询对象v当前存储的值类型是否是T

## 使用示例

```C++
#include <iostream>
#include <variant> // 导入头文件
using namespace std;

void new_cpp17_005(void)
{
	variant<int, double, string> d;
	cout << d.index() << endl; // 输出当前使用的类型在模版中的索引 0

	d = 3.14;
	cout << d.index() << endl; // 输出当前使用的类型在模版中的索引 1

	d = "string";
	cout << d.index() << endl; // 输出当前使用的类型在模版中的索引 2

	// 查找这个类型<int>在 d 中模版的索引 (不存在则报错)
	cout << holds_alternative<int>(d) << endl;

	// 访问当前正在使用的类型的值, 如果类型错误则抛出 std::bad_variant_access 异常
	std::cout << std::get<string>(d) << std::endl;
}
```

### 提供一些检测类型防止报错的手段

1. 使用 std::holds_alternative 函数检查当前存储的类型
```C++ By GPT-3.5
std::variant<int, float, std::string> myVariant;
myVariant = 42;

if (std::holds_alternative<int>(myVariant)) {
    int value = std::get<int>(myVariant);
    // 对于整数类型的处理
} else if (std::holds_alternative<float>(myVariant)) {
    float value = std::get<float>(myVariant);
    // 对于浮点数类型的处理
} else if (std::holds_alternative<std::string>(myVariant)) {
    std::string value = std::get<std::string>(myVariant);
    // 对于字符串类型的处理
}
```
2. 使用 std::visit 函数进行访问操作
```C++ By GPT-3.5
std::variant<int, float, std::string> myVariant;
myVariant = 3.14f;

std::visit([](auto&& arg) {
    using T = std::decay_t<decltype(arg)>; // 获取当前存储的类型
    if constexpr (std::is_same_v<T, int>) {
        // 对于整数类型的处理
        int value = arg;
    } else if constexpr (std::is_same_v<T, float>) {
        // 对于浮点数类型的处理
        float value = arg;
    } else if constexpr (std::is_same_v<T, std::string>) {
        // 对于字符串类型的处理
        std::string value = arg;
    }
}, myVariant);
```
