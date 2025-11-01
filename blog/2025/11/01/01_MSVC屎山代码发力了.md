---
authors: Heng_Xin
title: MSVC屎山代码发力了→requires前向声明问题
date: 2025-11-01 15:09:25
tags:
    - C++
---

MSVC屎山代码发力了, 真无敌了! 明明只是一个 requires 的二阶段查找的问题 なのに...

> 实际上后面发现是 前向声明 问题, 当然, 实际上也利用了 二阶段名称查找

<!-- truncate -->

## 一、缘起

哥们最近在为项目写了Web框架, 方便控制器的定义和使用等等, 并且支持依赖注入 ~~(实际上就是传参)~~

```cpp
#include <HXLibs/net/ApiMacro.hpp>

struct LoliDAO {
    uint64_t select(uint64_t id) {
        return id;
    }
};

HX_CONTROLLER(LoliController) {
    HX_ENDPOINT_MAIN(std::shared_ptr<LoliDAO> loliDAO) {
        using namespace HX::net;
        addEndpoint<GET>("/", [=] ENDPOINT {
            auto id = loliDAO->select(114514);
            co_await res.setStatusAndContent(Status::CODE_200, std::to_string(id))
                        .sendRes();
        })
        .addEndpoint<POST>("/post", [=] ENDPOINT {
            auto id = loliDAO->select(2233);
            co_await res.setStatusAndContent(Status::CODE_200, std::to_string(id))
                        .sendRes();
        });
    }
};

#include <HXLibs/net/UnApiMacro.hpp>

int main() {
    using namespace HX::net;
    HttpServer server{28205};
    // 依赖注入
    server.addController<LoliController>(std::make_shared<LoliDAO>());
    server.syncRun(1);
}
```

其中 `addController` 的实现是:

```cpp
/**
 * @brief 注册控制器到服务器, 可以进行依赖注入, 依次传参即可
 * @tparam T 控制器类型
 * @tparam Args 
 * @param args 被依赖注入的变量
 */
template <typename T, typename... Args>
    requires (std::is_base_of_v<class BaseController, T>)
inline HttpServer& HttpServer::addController(Args&&... args) {
    T {*this}.dependencyInjection(std::forward<Args>(args)...);
    return *this;
}
```

因为 `class BaseController` 的实现暂时是不可见, 但是在正确的使用下, `BaseController` 就是可见的了.

总之, 这里我期望使用 `二阶段查找` 实现约束控制器类型. 期望用户通过宏编写的 控制器 才是有效的.

确实, GCC 14+ / Clang17+ 都是正确的. 但 MSVC ? => https://godbolt.org/z/Kq7Gc5hs1 直接没有匹配的函数好吧.

## 二、描述

最小复现案例:

```cpp
#include <type_traits>
#include <concepts>

struct Test {
    template <typename T>
        requires (requires (T& t) {
            { static_cast<class Base&>(t) };
        })
    void func() {}
};

class Base {};
class A : public Base {};

int main() {
    Test{}.func<A>();
}
```

然后就更厉害了, 我一不小心的尝试, 发现 **MS💩VC** 就是 巨大的 💩 山!

## 三、💩

我的本意是希望说, `{ static_cast<class Base&>(t) }` 实际上就是 `std::is_base_of_v<class Base, T>` 的意思:

AUV, 您猜怎么着?

```cpp
#include <print>
#include <type_traits>
#include <concepts>

struct Test {
    template <typename T>
        requires (requires (T& t) {
            { static_cast<class Base&>(t) };
        })
    void func1() {}

    template <typename T>
        requires (std::is_base_of_v<class Base, T>)
    void func2() {}
};

class Base {};
class A : public Base {};

int main() {
    Test{}.func2<A>(); // 没有报错, 编译通过!
}
```

我就纳闷了, 怎么回事?!

我把 `func1` 删除, 然后就又报错了...

最终, 我把 `func1` 和 `func2` 位置交换:

```cpp
#include <print>
#include <type_traits>
#include <concepts>

struct Test {
    template <typename T>
        requires (std::is_base_of_v<class Base, T>)
    void func2() {}

    template <typename T>
        requires (requires (T& t) {
            { static_cast<class Base&>(t) };
        })
    void func1() {}
};

class Base {};
class A : public Base {};

int main() {
    Test{}.func1<A>(); // 正确
                       // ps: func2, MSVC 会编译报错
}
```

你这不是扯淡吗? MSVC 内部不会是写的 💩 吧?

```cpp
// 可能的 MSVC 内部呆码:
if (classFunc[0] == ???) {
    // mscv 专属操作 todo ...
} else {
    // 通用操作 todo ...
}
```

---

emm, 不对. 没有那么复杂, 实际上是 前向声明的问题. MSVC 好像抽风了, 它把 上一个函数的 requires 块 的 `class Base` 当做前向声明

而不把自己的 requires 内容当做前向声明.

```cpp
struct Test {
    template <typename T>
        requires (requires (T& t) {
            { static_cast<class Base&>(t) };  // 这个 class Base 被func2当做前向声明, func1 没有看到 class Base
        })
    void func1() {}

    template <typename T>
        requires (std::is_base_of_v<class Base, T>) 
    void func2() {}
};

func1(); // err
func2(); // ok: 看到 func1 的 class Base
```

## 四、解决方案

把 `requires` 提到 `函数体前`; 而不是 `requires 紧跟 template 后` (这种写法叫 **template-head requires clause**)

```cpp
struct Test {
    template <typename T>
    void func2() requires (requires (T& t) {
        { static_cast<class Base&>(t) };
    }) {}
};
```

他们都是等价的 (但是 MSVC 可能会有bug, 比如这里...)

> 你不需要修改写法, 怎么顺眼怎么来吧? msvc-std 的 tuple 也是 template-head requires clause 写法的...

---

- 随意丢弃到: [P10992381](https://developercommunity.visualstudio.com/t/requires-expression:-forward-declaration/10992381) 他们爱修不修吧...