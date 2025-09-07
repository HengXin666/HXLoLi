---
authors: Heng_Xin
title: MSVC的requires缺陷
date: 2025-09-07 16:43:37
tags:
    - C++
---

MSVC你真的牛逼大了!, 这种简单活都干不好? win都💩你无敌了 (\💩/ \💩/ \💩/)

<!-- truncate -->

> [!TIP]
> 本文只有恶意. win都💩! MS💩VC! 
> 
> \💩/ \💩/ \💩/ \💩/ \💩/

## msvc签名不完全匹配: requires 缺陷

今天上win, 顺便把之前的msvc bug 修复. 结果发现: ([#13](https://github.com/HengXin666/HXLibs/issues/13))

```cpp
D:\command\Github\HXLibs\tests\client\02_ws_client.cpp
[build] D:\command\Github\HXLibs\include\HXLibs/container/ThreadPool.hpp(338): error C2244: “HX::container::FutureResult<T>::thenTry”: 无法将函数定义与现有的声明匹配
[build] D:\command\Github\HXLibs\include\HXLibs/container/ThreadPool.hpp(338): note: 参见“HX::container::FutureResult<T>::thenTry”的声明
[build] D:\command\Github\HXLibs\include\HXLibs/container/ThreadPool.hpp(338): note: 定义
[build] D:\command\Github\HXLibs\include\HXLibs/container/ThreadPool.hpp(338): note: 'HX::container::FutureResult<internal::RemoveTryWarpImpl<Res>::Type> HX::container::FutureResult<T>::thenTry(Func &&) noexcept &&'
[build] D:\command\Github\HXLibs\include\HXLibs/container/ThreadPool.hpp(338): note: 现有声明
[build] D:\command\Github\HXLibs\include\HXLibs/container/FutureResult.hpp(127): note: 'HX::container::FutureResult<internal::RemoveTryWarpImpl<Res>::Type> HX::container::FutureResult<T>::thenTry(Func &&) noexcept &&'
[build] ninja: build stopped: subcommand failed.
```

经过排查, 发现是 msvc 的缺陷. 要求 requires 在函数签名中应该完全一样, 才会视作函数签名一样.

以下是复现案例: https://godbolt.org/z/EWhrdsoK6

```cpp
#include <utility>
#include <type_traits>

template <typename T = void>
struct Try {
    using TryType = T;
};

namespace internal {

template <typename T>
struct RemoveTryWarpImpl {
    using Type = T;
};

template <typename T>
struct RemoveTryWarpImpl<Try<T>> {
    using Type = T;
};

} // namespace internal

template <typename T>
using RemoveTryWarpType = internal::RemoveTryWarpImpl<T>::Type;

template <typename T = void>
class FutureResult {
public:
    using TryType = Try<RemoveTryWarpType<T>>;

    template <typename Func, typename Res = std::invoke_result_t<Func, TryType>>
    requires (requires (Func func, TryType t) { // 应该写为 FutureResult<T>::TryType
        func(std::move(t));                     // MSVC的错误是签名不完全匹配,
                                                // 原因是requires表达式写在了类模板声明和定义上,
                                                // 但是MSVC对这类情况的解析有bug,
                                                // 具体是 [requires必须一模一样, 否则视为不同函数]。
                                                // requires表达式里类型写法不一致, 导致MSVC无法匹配。
    })
    FutureResult<RemoveTryWarpType<Res>> thenTry(Func&& func) && noexcept;
};

template <typename T>
template <typename Func, typename Res>
    requires (requires (Func func, FutureResult<T>::TryType t) {
            func(std::move(t));
        })
FutureResult<RemoveTryWarpType<Res>> FutureResult<T>::thenTry(
    Func&& func
) && noexcept { return {}; }

int main() {
    FutureResult<int> f;
    auto res = std::move(f).thenTry([](Try<int> x) { 
        return x; 
    });
}
```

更加简单的, 最小复现案例: https://godbolt.org/z/xG1cjjbca

```cpp
#include <utility>
#include <type_traits>

template <typename T = void>
struct Try { using TryType = T; };

template <typename T = void>
class FutureResult {
public:
    using TryType = Try<T>;

    template <typename Func>
    requires requires(Func func, TryType t) { func(std::move(t)); }
    FutureResult thenTry(Func&& func) && noexcept;
};

template <typename T>
template <typename Func>
requires requires(Func func, typename FutureResult<T>::TryType t) { func(std::move(t)); }
FutureResult<T> FutureResult<T>::thenTry(Func&& func) && noexcept {
    return {};
}

int main() {
    FutureResult<int> f;
    std::move(f).thenTry([](Try<int> x) { return x; });
}
```

已经发送给 msvc 了 (https://developercommunity.visualstudio.com/t/MSVC-rejects-definitions-when-requires-c/10961820) (我才不管是否有重复提案呢..tm的这么久了都还有这个bug...)

之前我提的 ice 都不鸟我. 美名其曰 `我们的团队优先考虑对具有广泛客户影响的产品问题采取行动。`. 还是机器人回复. 你们开源节流还有真人吗? 人机!

呵呵, MSVC 和 win都💩