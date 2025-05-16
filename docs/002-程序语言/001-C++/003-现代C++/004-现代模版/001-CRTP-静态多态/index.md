# CRTP (奇异递归模版)

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