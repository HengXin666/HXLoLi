# auto
## 一、各个版本更新的新特性

> from: https://cppreference.cn/w/cpp/keyword/auto

用法:  
- 自动存储期说明符 (之前 C++11)
- auto 占位符类型说明符、具有尾随返回类型的函数声明 (起始 C++11)
- 结构化绑定声明 (起始 C++17)
- 缩写函数模板 (起始 C++20)
- 显式转换为 prvalue 副本 (起始 C++23)

参见:
- decltype (`decltype(auto)`) (起始 C++14)
- register (自动存储期说明符) (之前 C++17)

## 二、auto 作为函数返回值的研究

此处将比较 `auto`、`auto&`、`auto&&`、`decltype(auto)`作为返回值时候的区别

> 实验代码: https://github.com/HengXin666/HXTest/blob/main/src/06-std-analyse/test/09-auto/01_auto_return.cpp

我们使用如下宏来查看具体的类型 (以及`clangd`插件)

```cpp
template <typename T>
std::string_view getType() {
#if 1
    auto str = __PRETTY_FUNCTION__;
    std::string_view res{str};
    res = res.substr(res.find('[') + 1);
    res = res.substr(0, res.size() - 1);
    return res;
#else
    auto str = __FUNCSIG__; // MSVC 的 __FUNCSIG__
    #error "not code"
#endif
}

#define infoType(__TYPE__) \
HX::print::println(#__TYPE__": ", getType<__TYPE__>())
```

实验数据:

```cpp
int x = 0;
const int c_x = x;
int& r_x = x;
const int& cr_x = x;
```

### 2.1 auto

```cpp
template <typename T>
auto funA1(T t) {       // 有 const 也不会保留, 仅返回 T
                        // 忽略所有的 & 与 &&
    return t;
}

infoType(decltype(funA1(x)));               // int
infoType(decltype(funA1(c_x)));             // int
infoType(decltype(funA1(r_x)));             // int
infoType(decltype(funA1(cr_x)));            // int
infoType(decltype(funA1(std::move(x))));    // int
```

### 2.2 auto&

```cpp
template <typename T>
auto& funA2(T& t) {     // 有 const 会保留 const
    return t;
}

infoType(decltype(funA2(x)));               // int&
infoType(decltype(funA2(c_x)));             // const int&
infoType(decltype(funA2(r_x)));             // int&
infoType(decltype(funA2(cr_x)));            // const int&
// infoType(decltype(funA2(std::move(x)))); // 没有匹配的函数
```

### 2.3 auto&&

```cpp
template <typename T>
auto&& funA3(T&& t) {   // 有 const 会保留 const
    if constexpr (0) {
        return t; // 返回 & 类型, 和 auto&& + & -> auto&&& 触发引用折叠 [(&&)&] -> auto&
    } else if constexpr (0) {
        return std::move(t); // 返回 && 类型, 和 auto&& + && -> auto&&&& 触发引用折叠 [(&&)&&] -> auto&&
    } else {
        return std::forward<T>(t);
    }
}

// #define __TEST__
#ifdef __TEST__
template <typename T>
auto&& funA3_nr(T t) {
    return t;
}
#endif // !__TEST__

infoType(decltype(funA3(x)));               // int&
infoType(decltype(funA3(c_x)));             // const int&
infoType(decltype(funA3(r_x)));             // int&
infoType(decltype(funA3(cr_x)));            // const int&
infoType(decltype(funA3(std::move(x))));    // inr&&

#ifdef __TEST__
infoType(decltype(funA3_nr(x)));            // int&
#endif // ！__TEST__
```

### 2.4 decltype(auto)

```cpp
template <typename T>
decltype(auto) funA4(T&& t) {   // decltype(auto) 是返回值的完美转发
    if constexpr (0) {          // 如果返回值是引用, 则返回引用
                                // 如果返回值是右值, 则返回右值
                                // 如果返回值无&, 仅返回该类型
                                // 一切操作保留原有的 const
        return t;
    } else if constexpr (0) {
        return std::move(t);
    } else {
        return std::forward<T>(t);
    }
}

template <typename T>
decltype(auto) funA4_nr(T t) {
    return t;
}

infoType(decltype(funA4(x)));               // int&
infoType(decltype(funA4(c_x)));             // const int&
infoType(decltype(funA4(r_x)));             // int&
infoType(decltype(funA4(cr_x)));            // const int&
infoType(decltype(funA4(std::move(x))));    // int&&

infoType(decltype(funA4_nr(x)));            // int
```

> [!TIP]
> `decltype(auto)` 和 `auto&&` 在返回值时候唯一不同的是: 
> > 后者永远是带引用的, 而前者是真正意义的完美转发, 还可以被RVO(消除复制)优化

## 三、todo ...