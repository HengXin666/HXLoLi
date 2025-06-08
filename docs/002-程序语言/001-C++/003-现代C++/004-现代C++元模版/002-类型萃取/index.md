# 类型萃取

```cpp
#include <HXprint/print.h>

namespace HX {

template <typename T>
struct IsVoid {
    inline static constexpr bool val = false;
};

template <>
struct IsVoid<void> {
    inline static constexpr bool val = true;
};

/**
 * @brief 这个类型是否是 void
 * @tparam T 
 */
template <typename T>
inline constexpr bool IsVoid_V = IsVoid<T>::val;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

template <typename T>
struct RemoveReference {
    using type = T;
};

template <typename T>
struct RemoveReference<T&> {
    using type = T;
};

/**
 * @brief 去掉类型引用
 * @tparam T 
 */
template <typename T>
using RemoveReference_T = RemoveReference<T>::type;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

template <typename T, typename U>
struct IsSame {
    inline static constexpr bool val = false;
};

template <typename T>
struct IsSame<T, T> {
    inline static constexpr bool val = true;
};

/**
 * @brief 判断这两个类型是否完全一样
 * @tparam T 
 * @tparam U 
 */
template <typename T, typename U>
inline constexpr bool IsSame_V = IsSame<T, U>::val;

} // namespace HX

#define INFO_CODE(__CODE__) HX::print::println(#__CODE__": ", __CODE__)

int main() {
    using namespace HX;

    INFO_CODE(IsVoid_V<int>);  // false
    INFO_CODE(IsVoid_V<void>); // true

    using T1 = RemoveReference_T<int>;  // int
    using T2 = RemoveReference_T<int&>; // int

    INFO_CODE((IsSame_V<int, int&>)); // false
    INFO_CODE((IsSame_V<T1, T2>));    // true
    return 0;
}

#undef INFO_CODE
```

核心思想实际上是 `SFINAE ("代换失败不是错误" (Substitution Failure Is Not An Error))`

> 在函数模板的重载决议中会应用此规则: 当模板形参在替换成显式指定的类型或推导出的类型失败时, 从重载集中丢弃这个特化, 而非导致编译失败。

他们的实现是多种多样的, 可能是类的偏特化、变量的偏特化、`auto -> decltype(...)`, `template` 中写 `std::enable_if`, 等等... (或者可以使用现代的`约束与概念`写法(C++20))

```cpp
// 为了证明, 上面的 IsSame_V 实际上可以仅变量的偏特化实现. (IsVoid、RemoveReference_T 同理; 
// 上面只是为了展示C++11的写法 (RemoveReference<T>::type) )

template <typename, typename>
inline constexpr bool IsSame_V2 = false;

template <typename T>
inline constexpr bool IsSame_V2<T, T> = true;
```