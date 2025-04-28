# std::make_from_tuple
## 说明
`C++17` 中的 `std::make_from_tuple` 是一个函数模板，用于从 `std::tuple` 对象中构造对象。它接受一个可调用对象（如函数、函数指针或函数对象）以及一个 `std::tuple` 对象作为参数，并将 `tuple` 中的元素解包并传递给可调用对象进行构造。

这个函数模板的主要目的是提供一种通用的方式来使用 `tuple` 中的元素构造对象，而不需要手动解包和传递参数。它可以方便地与元编程技术结合使用，使代码更加简洁和灵活。

即 $解包tuple作为构造函数参数构造对象$.

## 示例

```C++
#include <iostream>

using namespace std;

class ClassTest {
public:
    string _name;
    size_t _age;
    
    ClassTest(string name, size_t age) : _name(name), _age(age) {
        cout << "name: " << _name << ", age: " << _age << endl;
    }
};

int main() {
    auto param = make_tuple("zhangsan", 19);

	// 使用 std::make_from_tuple 函数将 param 解包并传递给 ClassTest 的构造函数，从而创建了一个 ClassTest 对象
    auto obj = make_from_tuple<ClassTest>(move(param));
    
    // 访问 obj 对象的成员
    cout << "name: " << obj._name << ", age: " << obj._age << endl;
    
    return 0;
}
```
