# 结构化绑定
## 说明
`C++17`引入了 **结构化绑定（Structured Binding）** 的特性，它使得从元组、数组、结构体等类型中提取成员变量变得更加方便。

通过结构化绑定，可以将一个复合类型的成员`按照顺序`绑定到多个变量上，而无需显式地进行逐个赋值操作。

> [!TIP]
> 结构化绑定, 也可以提取类的成员
>
> 结构化绑定, 不支持直接绑定为`编译期常量`, 即便他们就是

## 示例


```C++
#include <iostream>
#include <tuple>

std::tuple<int, float, std::string> GetPerson()
{
    return std::make_tuple(30, 175.5f, "John Doe");
}

int main()
{
	// C++11 (对比)
	int l_age;
	double l_height;
	string l_name;
	tie(l_age, l_height, l_name) = GetPerson();

    // C++17
    auto [age, height, name] = GetPerson();
    
    std::cout << "Age: " << age << std::endl;
    std::cout << "Height: " << height << std::endl;
    std::cout << "Name: " << name << std::endl;
    
    return 0;
}
```
