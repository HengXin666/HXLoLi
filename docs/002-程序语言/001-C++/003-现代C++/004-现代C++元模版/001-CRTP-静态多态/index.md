# CRTP (奇异递归模版)

> [!TIP]
> 实际上并不一定翻译为 `奇异递归模版`, 只是比较多的人这样叫罢了. (国王说没有递归...) | 也有称作 `奇特重现模板模式`...

## 一、CRTP 介绍
我们常说的编译期多态(无需虚函数表、在编译阶段就确定函数调用路径的多态机制), 实际上就是基于CRTP实现的, 下面是常见的编译期多态的实现:

| 技术手段 | 静态多态 | 可组合容器  | 可内联 | 类型安全 | 易用性      |
| :-: | - | - | - | - | - |
| CRTP |✅ | ❌(需封装) |✅|✅ | 中 |
| 模板参数 / Duck Typing   |✅|❌|✅|❌|✅|
| Concepts |✅ | ❌|✅|✅ | ✅(C++20) |
| std::variant + visit |✅ |✅   |✅|✅ | 中 |
| `if constexpr` |✅ | ❌|✅|✅ | 高 |

其中, 基于自定义类的, 就是CRTP:

```cpp
#include <HXprint/print.h>

/**
 * @brief 奇异递归模版 (CRTF)
 * 用于实现静态多态
 */

template <typename T>
struct Man {
    void oi() {
        static_cast<T*>(this)->oi();
    }

    void kawaii() {
        static_cast<T*>(this)->kawaii();
    }

    // 链式调用
    T& zzz() {
        HX::print::print("zzz... -> ");
        return static_cast<T&>(*this);
    }
};

struct LoLi : public Man<LoLi> {
    void oi() {
        HX::print::println("LoLi: o... o ha yo~");
    }

private:
    friend struct Man<LoLi>; // 如果基类访问的不是公有的, 那么需要声明为友元

    void kawaii() {
        HX::print::println("LoLi: hei hei hei~");
    }
};

struct Imouto : public Man<Imouto> {
    void oi() {
        HX::print::println("Imouto: o ha yo~ o ni tya~");
    }

private:
    friend struct Man<Imouto>;

    void kawaii() {
        HX::print::println("Imouto: mu kyu~");
    }
};

/**
 * @brief 使用示例: 静态多态
 * @tparam T 
 * @param man 
 */
template <typename T>
void greet(Man<T>& man) {
    man.zzz().oi();
    man.kawaii();
}

int main() {
    LoLi loli{};
    Imouto imouto{};
    greet(loli);
    greet(imouto);
    return 0;
}
```

上面展现了三种情况,

1. 普通多态调用

2. 对于非公有的方法多态调用

3. 链式调用

坏处就是如果你希望统一存储为 `std::vector<T>` 这种, 其中 `[T = 基类]`, 是不行的...

要么就用 `std::variant` (但是破坏了开闭原则), 要么就只能搞运行时多态了 (再搞个虚基类、用 `std::Any` 什么的)

> 总之就是还是有点局限的~

## 二、C++23中的实现

在读懂这个东西之前, 我们需要学习一些东西~

### 2.1 成员函数的隐式传参

对于以下代码, 考虑我们常说的`this`指针是什么呢?

```cpp
struct A {
    void fun(int a) {
        std::cout << a + this->_v << '\n';
    }

    int _v;
};
```

这个 `this` 指针是怎么传参进去的呢? 亦或者说, 我们怎么知道我们操作的东西呢?

而 `static` 的成员方法, 为什么又不能用 `this` 指针呢? 可能你会说, C++就是这样的.

但是, 如果你有学过py:

```py
class A:
    def __init__(self, v: int):
        self.v = v
    def fun(self, a: int):
        print(a + self.v)
```

你会发现, 这种传入 `self` 的, 是方法, 不传入的, 算是 静态方法~

回到C++, 想必你也能体感到, 这个 this 指针, 实际上就是编译器给我们隐式传入的!

### 2.2 C++23 显式 this

然后C++23它就支持一下语法:

```cpp
struct A {
    void fun(this A const& self, int a) {
        std::cout << a + self._v << '\n';
    }

    int _v;
};

// 调用依旧是:
A a{2233};
a.fun(666); // 不需要 a.fun(a, 666) !
```

这里就没有什么用 (只是我是用不上了)

### 2.3 C++23 CRTP

```cpp
struct Msvc {
    void error(this auto&& self, std::string const& msg) {
        self.impl(msg);
    }
};

struct Tantantan : Msvc {
    void impl(std::string const& msg) const {
        std::cerr << msg << "烫烫烫\n";
    }
};

struct LinGanGu : Msvc {
    void impl(std::string const& msg) const {
        std::cerr << msg << "灵感菇\n";
    }
};

template <typename T>
void error(T const& t, std::string const& msg) {
    t.error(msg);
}
```

如你所见, 它甚至不需要`static_cast`转换.

> 运行示例: https://godbolt.org/z/xYnh1PG1h

### 2.4 Lambda 递归加强

因为 Lambda 本质上就是语法糖, 一个匿名类重载了 `()` 运算符, 因此它也可以显式传参 `this`

这下就有非常简洁的递归 Lambda 写法:

```cpp [z1-C++23]
auto dfs = [&](this auto&& dfs, int i) -> int { // C++23 现代
    return i > 1 ? dfs(i - 1) * i : i;
};
```

```cpp [z1-C++14]
auto dfs = [&](auto&& dfs, int i) -> int { // C++ 14 支持 auto 做参数进行推导
    return i > 1 ? dfs(dfs, i - 1) * i : i;
};
```

```cpp [z1-C++11]
#include <functional>

std::function<int(int)> dfs = [&](int i) { // 有虚函数开销! (因为类型擦除)
    return i > 1 ? dfs(i - 1) * i : i;
};
```