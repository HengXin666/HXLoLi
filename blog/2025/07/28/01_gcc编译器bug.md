---
authors: Heng_Xin
title: gcc 内部编译器 段错误
date: 2025-07-28 09:56:36
tags:
    - C++
---

之前写项目的时候发现一个gcc编译器bug. 尝试给他们提 `issues` 了~...

<!-- truncate -->

## 1. 实际代码

实际代码如下:

```cpp [c1-项目源码]
// 节选: https://github.com/HengXin666/HXLibs/blob/main/include/HXLibs/reflection/json/JsonRead.hpp

template <std::size_t _I>
struct SetObjIdx {
    inline static constexpr std::size_t Idx = _I;
    constexpr SetObjIdx() = default;
};

template <std::size_t... Idx>
constexpr auto makeVariantSetObjIdxs(std::index_sequence<Idx...>) {
    /** 
     * GCC Hock, 在 gcc/x86_64-pc-linux-gnu/15.1.1/lto-wrapper
     * gcc 版本 15.1.1 20250425 (GCC) 中, 不能使用:
     * using CHashMapValType = decltype([] <std::size_t... Idx> (std::index_sequence<Idx...>) {
     *     return std::variant<SetObjIdx<Idx>...>{};
     * }(std::make_index_sequence<N>{}));
     * 否则编译器会崩溃...
     */
    return std::variant<SetObjIdx<Idx>...>{};
}

template <typename T>
constexpr auto makeNameToIdxVariantHashMap() {
    constexpr auto N = membersCountVal<T>;
    constexpr auto nameArr = getMembersNames<T>();
    using CHashMapValType = decltype([] <std::size_t... Idx> (std::index_sequence<Idx...>) {
         return std::variant<SetObjIdx<Idx>...>{};
    }(std::make_index_sequence<N>{}));
    return container::CHashMap<std::string_view, CHashMapValType, N>{
        [&] <std::size_t... Idx> (std::index_sequence<Idx...>) {
            return std::array<std::pair<std::string_view, CHashMapValType>, N>{{
                {nameArr[Idx], CHashMapValType{SetObjIdx<Idx>{}}}... 
            }};
        }(std::make_index_sequence<N>{})
    };
}
```

```cpp [c1-报错]
[build] /home/loli/Loli/code/HXLibs/include/HXLibs/reflection/json/JsonRead.hpp: In instantiation of ‘HX::reflection::internal::makeNameToIdxVariantHashMap<HXTest>()::<lambda(std::index_sequence<_Inds ...>)> [with long unsigned int ...Idx = {0, 1}; std::index_sequence<_Inds ...> = std::integer_sequence<long unsigned int, 0, 1>]’:
[build] /home/loli/Loli/code/HXLibs/include/HXLibs/reflection/json/JsonRead.hpp:120:10:   required from ‘constexpr auto HX::reflection::internal::makeNameToIdxVariantHashMap() [with T = HXTest]’
[build]   116 |         [&] <std::size_t... Idx> (std::index_sequence<Idx...>) {
[build]       |         ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
[build]   117 |             return std::array<std::pair<std::string_view, CHashMapValType>, N>{{
[build]       |             ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
[build]   118 |                 {nameArr[Idx], CHashMapValType{SetObjIdx<Idx>{}}}...
[build]       |                 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
[build]   119 |             }};
[build]       |             ~~~
[build]   120 |         }(std::make_index_sequence<N>{})
[build]       |         ~^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
[build] /home/loli/Loli/code/HXLibs/include/HXLibs/reflection/json/JsonRead.hpp:485:76:   required from ‘static void HX::reflection::internal::FromJson::fromJson(T&, It&&, It&&) [with T = HXTest; It = const char*]’
[build]   485 |             static constexpr auto nameHash = makeNameToIdxVariantHashMap<T>();
[build]       |                                              ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~
[build] /home/loli/Loli/code/HXLibs/include/HXLibs/reflection/json/JsonRead.hpp:528:33:   required from ‘void HX::reflection::fromJson(T&, std::string_view) [with T = HXTest; std::string_view = std::basic_string_view<char>]’
[build]   528 |     internal::FromJson::fromJson(t, json.begin(), json.end());
[build]       |     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~~~~~~~~~~
[build] /home/loli/Loli/code/HXLibs/tests/reflection/01_reflection.cpp:305:25:   required from here
[build]   305 |     reflection::fromJson(newT, s);
[build]       |     ~~~~~~~~~~~~~~~~~~~~^~~~~~~~~
[build] /home/loli/Loli/code/HXLibs/include/HXLibs/reflection/json/JsonRead.hpp:112:38: 编译器内部错误：段错误
[build]   112 |     using CHashMapValType = decltype([] <std::size_t... Idx> (std::index_sequence<Idx...>) {
[build]       |                                      ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
[build]   113 |          return std::variant<SetObjIdx<Idx>...>{};
[build]       |          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
[build]   114 |     }(std::make_index_sequence<N>{}));
[build]       |     ~                                 
[build] 0x26d6c51 diagnostic_context::diagnostic_impl(rich_location*, diagnostic_metadata const*, diagnostic_option_id, char const*, __va_list_tag (*) [1], diagnostic_t)
[build] 	???:0
[build] 0x2730d98 internal_error(char const*, ...)
[build] 	???:0
[build] 0x908d48 template_parms_to_args(tree_node*)
[build] 	???:0
[build] 0x9224dd tsubst_lambda_expr(tree_node*, tree_node*, int, tree_node*)
[build] 	???:0
[build] 0x911d5b tsubst(tree_node*, tree_node*, int, tree_node*)
[build] 	???:0
[build] 0x91be37 tsubst_template_args(tree_node*, tree_node*, int, tree_node*)
[build] 	???:0
[build] 0x91237b tsubst(tree_node*, tree_node*, int, tree_node*)
[build] 	???:0
[build] 0x91be37 tsubst_template_args(tree_node*, tree_node*, int, tree_node*)
[build] 	???:0
[build] 0x91237b tsubst(tree_node*, tree_node*, int, tree_node*)
[build] 	???:0
[build] 0x92541d instantiate_decl(tree_node*, bool, bool)
[build] 	???:0
[build] 0x7b7012 maybe_instantiate_decl(tree_node*)
[build] 	???:0
[build] 0x7c8ad7 mark_used(tree_node*, int)
[build] 	???:0
[build] 0x75491c build_op_call(tree_node*, vec<tree_node*, va_gc, vl_embed>**, int)
[build] 	???:0
[build] 0x942ee2 finish_call_expr(tree_node*, vec<tree_node*, va_gc, vl_embed>**, bool, bool, int)
[build] 	???:0
[build] 0x92541d instantiate_decl(tree_node*, bool, bool)
[build] 	???:0
[build] 0x7b7012 maybe_instantiate_decl(tree_node*)
[build] 	???:0
[build] 0x7c8ad7 mark_used(tree_node*, int)
[build] 	???:0
[build] 0x753e1c build_new_function_call(tree_node*, vec<tree_node*, va_gc, vl_embed>**, int)
[build] 	???:0
[build] 0x942eac finish_call_expr(tree_node*, vec<tree_node*, va_gc, vl_embed>**, bool, bool, int)
[build] 	???:0
[build] 0x92541d instantiate_decl(tree_node*, bool, bool)
[build] 	???:0
[build] Please submit a full bug report, with preprocessed source (by using -freport-bug).
[build] Please include the complete backtrace with any bug report.
[build] 参阅 <https://gitlab.archlinux.org/archlinux/packaging/packages/gcc/-/issues> 以获取指示。
[build] make[3]: *** [CMakeFiles/01_reflection.dir/build.make:79：CMakeFiles/01_reflection.dir/tests/reflection/01_reflection.cpp.o] 错误 1
```

## 2. 最小可复现

尝试最小可复现案例: (https://godbolt.org/z/9EfjWEvTP) & (https://godbolt.org/z/fM84Ex34j)

```cpp [c2-最小复现_1]
#include <variant>
#include <array>

template <std::size_t I>
struct SetObjIdx {};

// 必须得是模板
template <typename T = void>
constexpr auto makeNameToIdxVariantHashMap() {
    constexpr auto N = 3U; // N 可以是任意常量

    // 1. 是 decltype 配合 []<>(){}() + 模板
    using Res = decltype([] <std::size_t... Idx> (std::index_sequence<Idx...>) {
        return std::variant<SetObjIdx<Idx>...>{};
    } (std::make_index_sequence<N>{}));

    // 2. 必须是 <std::size_t... Idx> 模板
    return [&] <std::size_t... Idx> (std::index_sequence<Idx...>) {
        return std::array<Res, N>{
            SetObjIdx<Idx>{}... 
        };
    }(std::make_index_sequence<N>{});
}

int main() {
    constexpr auto res = makeNameToIdxVariantHashMap();
    (void)res;
    return 0;
}
```

```cpp [c2-最小复现_2]
#include <variant>
#include <array>

template <std::size_t I>
struct SetObjIdx { std::size_t _idx; };

// Must be a template
template <typename T = void>
constexpr auto makeNameToIdxVariantHashMap() {
    constexpr auto N = 3U; // N arbitrary

    // 1. decltype 配合 []<>(){}() + 模板
    using Res = decltype([] <std::size_t Idx> (std::index_sequence<Idx>) {
        return SetObjIdx<Idx>{Idx};
    }(std::index_sequence<N>{}));

    // 2. 必须是 <std::size_t... Idx> 模板
    return [&] <std::size_t... Idx> (std::index_sequence<Idx...>) {
        return std::array<Res, N>{
            SetObjIdx<N>{Idx}... 
        };
    }(std::make_index_sequence<N>{});
}

int main() {
    constexpr auto res = makeNameToIdxVariantHashMap();
    return 0;
}
```

```cpp [c2-最小复现_1 编译输出]
<source>: In instantiation of 'makeNameToIdxVariantHashMap<>()::<lambda(std::index_sequence<__var_indices ...>)> [with long unsigned int ...Idx = {0, 1, 2}; std::index_sequence<__var_indices ...> = std::integer_sequence<long unsigned int, 0, 1, 2>]':
<source>:22:6:   required from 'constexpr auto makeNameToIdxVariantHashMap() [with T = void]'
   18 |     return [&] <std::size_t... Idx> (std::index_sequence<Idx...>) {
      |            ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   19 |         return std::array<Res, N>{
      |         ~~~~~~~~~~~~~~~~~~~~~~~~~~
   20 |             SetObjIdx<Idx>{}...
      |             ~~~~~~~~~~~~~~~~~~~
   21 |         };
      |         ~~
   22 |     }(std::make_index_sequence<N>{});
      |     ~^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
<source>:26:53:   required from here
   26 |     constexpr auto res = makeNameToIdxVariantHashMap();
      |                          ~~~~~~~~~~~~~~~~~~~~~~~~~~~^~
<source>:13:26: internal compiler error: Segmentation fault
   13 |     using Res = decltype([] <std::size_t... Idx> (std::index_sequence<Idx...>) {
      |                          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   14 |         return std::variant<SetObjIdx<Idx>...>{};
      |         ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   15 |     } (std::make_index_sequence<N>{}));
      |     ~                     
0x2287785 diagnostic_context::diagnostic_impl(rich_location*, diagnostic_metadata const*, diagnostic_option_id, char const*, __va_list_tag (*) [1], diagnostic_t)
	???:0
0x2298cd6 internal_error(char const*, ...)
	???:0
0x989f38 template_parms_to_args(tree_node*)
	???:0
0x9b8692 tsubst_lambda_expr(tree_node*, tree_node*, int, tree_node*)
	???:0
0x9ab9bc tsubst(tree_node*, tree_node*, int, tree_node*)
	???:0
0x9b2467 tsubst_template_args(tree_node*, tree_node*, int, tree_node*)
	???:0
0x9ab435 tsubst(tree_node*, tree_node*, int, tree_node*)
	???:0
0x99f62f instantiate_decl(tree_node*, bool, bool)
	???:0
0x894821 maybe_instantiate_decl(tree_node*)
	???:0
0x895b67 mark_used(tree_node*, int)
	???:0
0x806f93 build_op_call(tree_node*, vec<tree_node*, va_gc, vl_embed>**, int)
	???:0
0x9d8b67 finish_call_expr(tree_node*, vec<tree_node*, va_gc, vl_embed>**, bool, bool, int)
	???:0
0x99f62f instantiate_decl(tree_node*, bool, bool)
	???:0
0x894821 maybe_instantiate_decl(tree_node*)
	???:0
0x895b67 mark_used(tree_node*, int)
	???:0
0x80330e build_new_function_call(tree_node*, vec<tree_node*, va_gc, vl_embed>**, int)
	???:0
0x9d8b3e finish_call_expr(tree_node*, vec<tree_node*, va_gc, vl_embed>**, bool, bool, int)
	???:0
0x981ebd c_parse_file()
	???:0
0xa8b739 c_common_parse_file()
	???:0
Please submit a full bug report, with preprocessed source (by using -freport-bug).
Please include the complete backtrace with any bug report.
See <https://gcc.gnu.org/bugs/> for instructions.
```

目前解决方案是:

1. 去掉模板 `template <typename T = void>`
2. 把 `decltype([] ...)` 提出到外部模板函数, 然后 `decltype(func(...))` 也是 OK 的.

> 已经报告到 https://gcc.gnu.org/bugzilla/show_bug.cgi?id=121287 中, 不知道有没有重复的... 看到类似但是不是一样的qwq