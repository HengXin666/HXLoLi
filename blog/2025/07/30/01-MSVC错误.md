---
authors: Heng_Xin
title: MSVC错误 编译期缓存错误和ICE
date: 2025-07-30 13:47:35
tags:
    - C++
---

之前在写反射的时候, 遇到的这个问题; 一开始以为是我自己写错了, 排查了半天, 从 VsCode 换到 MSVC...

最后写了最小复现案例, 然后才发现就是 MSVC sabi!

<!-- truncate -->

## 描述

请见 https://github.com/HengXin666/HXLibs/issues/7

```cpp
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <HXLibs/reflection/MemberName.hpp>
#include <HXLibs/reflection/ReflectionMacro.hpp>
#include <HXLibs/log/Log.hpp>

using namespace HX;

TEST_CASE("对照组: 无宏反射") {
    struct Case01 {
        int a;
        std::string b;
        std::vector<Case01> c;
    };

    struct Case02 {
        int num;
        std::string str;
        std::vector<Case02> arr;
    };

    constexpr auto N = reflection::membersCountVal<Case01>;
    constexpr auto name = reflection::getMembersNames<Case01>();

    CHECK(N == 3);
    CHECK(name[0] == "a");
    CHECK(name[1] == "b");
    CHECK(name[2] == "c");

    constexpr auto N2 = reflection::membersCountVal<Case02>;
    constexpr auto name2 = reflection::getMembersNames<Case02>();

    CHECK(N2 == 3);
    CHECK(name2[0] == "num");
    CHECK(name2[1] == "str");
    CHECK(name2[2] == "arr");
}
```

在线实验: https://godbolt.org/z/zzaWsP5j7

```cpp
#include <variant>
#include <optional>
#include <memory>
#include <type_traits>

namespace HX::meta {

/**
 * @brief 删除 T 类型的 const、引用、v 修饰
 * @tparam T 
 */
template <typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

} // !namespace

using namespace HX;

/**
 * @brief 对于类型 T, 提供类静态成员
 * @tparam T 
 */
template <typename T>
struct StaticObj {
    inline static meta::remove_cvref_t<T> obj;
};

/**
 * @brief 获取类静态成员T
 * @tparam T 
 * @return constexpr utils::remove_cvref_t<T>& 
 */
template <typename T>
inline constexpr meta::remove_cvref_t<T>& getStaticObj() {
    return StaticObj<T>::obj;
}

/**
 * @brief 访问者类 主模版 (不可使用)
 * @tparam T `聚合类`类型
 * @tparam N 成员个数
 * @warning 如果匹配到此模版, 那么有两种可能:
 * @warning 1) 你的类有255+个成员
 * @warning 2) 你的类不是聚合类
 */
template <typename T, std::size_t N>
struct ReflectionVisitor {
    static constexpr auto visit() {
        static_assert(!sizeof(T), "");
    }
};

/**
 * @brief 偏特化模版生成 工具宏
 */
#define _HX_GENERATE_TEMPLATES_WITH_SPECIALIZATION_(N, ...) \
template <typename T>                                       \
struct ReflectionVisitor<T, N> {                            \
    static constexpr auto visit() {                         \
        auto& [__VA_ARGS__] = getStaticObj<T>();            \
        auto t = std::tie(__VA_ARGS__);                     \
        constexpr auto f = [](auto&... fs) {                \
            return std::make_tuple((&fs)...);               \
        };                                                  \
        return std::apply(f, t);                            \
    }                                                       \
};

_HX_GENERATE_TEMPLATES_WITH_SPECIALIZATION_(1, f0);
_HX_GENERATE_TEMPLATES_WITH_SPECIALIZATION_(2, f0, f1);
_HX_GENERATE_TEMPLATES_WITH_SPECIALIZATION_(3, f0, f1, f2);
_HX_GENERATE_TEMPLATES_WITH_SPECIALIZATION_(4, f0, f1, f2, f3);

template <std::size_t N, typename T>
inline constexpr auto getStaticObjPtrTuple() {
 return ReflectionVisitor<meta::remove_cvref_t<T>, N>::visit();
}

#include <string>
#include <vector>
#include <iostream>

template <auto ptr>
inline constexpr std::string_view getMemberName() {
#if defined(_MSC_VER)
    constexpr std::string_view funcName = __FUNCSIG__;
#else
    constexpr std::string_view funcName = __PRETTY_FUNCTION__;
#endif
    return funcName;
}

template <class T>
struct Wrapper {
  using Type = T;
  T v;
};

template <class T>
Wrapper(T) -> Wrapper<T>;

// This workaround is necessary for clang. or msvc
template <class T>
inline constexpr auto wrap(const T& arg) noexcept {
  return Wrapper{arg};
}

int main() {
    struct Case01 {
        int a;
        std::string b;
        std::vector<Case01> c;
    };

    struct Case02 {
        int num;
        std::string str;
        std::vector<Case02> arr;
    };

    constexpr auto tp1 = getStaticObjPtrTuple<3, Case01>();
    constexpr auto tp2 = getStaticObjPtrTuple<3, Case02>();
    // clang 17 <=, 就会报错
    // msvc 得到的是 a 和 a, 而不是 a 和 num
    std::cout << getMemberName<get<0>(tp1)>() << '\n';
    std::cout << getMemberName<get<0>(tp2)>() << '\n';

    // 需要套一层
    std::cout << getMemberName<wrap(get<0>(tp1))>() << '\n';
    std::cout << getMemberName<wrap(get<0>(tp2))>() << '\n';
    return 0;
}
```

发现套一层就好了~

## 还有高手

当我准备整个最小可复现案例的时候:

```cpp [c1-案例]
#include <string>
#include <iostream>

template <auto ptr>
inline constexpr std::string_view getMemberName() {
#if defined(_MSC_VER)
    constexpr std::string_view funcName = __FUNCSIG__;
#else
    constexpr std::string_view funcName = __PRETTY_FUNCTION__;
#endif
    return funcName;
}

template <class T>
struct Wrapper {
  using Type = T;
  T v;
};

template <class T>
Wrapper(T) -> Wrapper<T>;

template <class T>
inline constexpr auto wrap(const T& arg) noexcept {
  return Wrapper{arg};
}

int main() {
    struct Case01 {
        int a;
    };

    struct Case02 {
        int num;
    };

    static Case01 c01;
    static Case02 c02;

    constexpr auto ptr1 = &c01.a;
    constexpr auto ptr2 = &c02.num;

    // MSVC obtains a and a, not a and num
    // std::cout << getMemberName<ptr1>() << '\n';
    // std::cout << getMemberName<ptr2>() << '\n';

    // (50): fatal error C1001: Internal compiler error.
    std::cout << getMemberName<wrap(ptr1)>() << '\n';
    std::cout << getMemberName<wrap(ptr2)>() << '\n';
    return 0;
}
```

```cpp [c1-MSVC编译输出]
example.cpp
<source>(50): fatal error C1001: Internal compiler error.
(compiler file 'msc1.cpp', line 1533)
 To work around this problem, try simplifying or changing the program near the locations listed above.
If possible please provide a repro here: https://developercommunity.visualstudio.com 
Please choose the Technical Support command on the Visual C++ 
 Help menu, or open the Technical Support help file for more information
<source>(50): note: the template instantiation context (the oldest one first) is
<source>(50): note: see reference to function template instantiation 'std::string_view getMemberName(void)' being compiled
INTERNAL COMPILER ERROR in 'Z:\compilers\msvc\14.43.34808-14.43.34810.0\bin\Hostx64\x64\cl.exe'
    Please choose the Technical Support command on the Visual C++
    Help menu, or open the Technical Support help file for more information
```

我靠, 直接 `Internal compiler error (ICE)` 了...

然后我只好反馈给 MSVC 了: https://developercommunity.visualstudio.com/t/MSVC-ICE-when-using-templated-auto-param/10944095