---
authors: Heng_Xin
title: 速记之前遇到的msvc和gcc八嘎
date: 2025-11-08 14:24:53
tags:
    - C++
---

速记, 此bug比较难报告上去.

<!-- truncate -->

> 前几天遇到的, 先记录一下. 有空再看看, 再仔细测测~

## 1. MSVC: RVO 返回值优化失败

在 C++17 之后, 标准强制规定返回值优化.

```cpp
struct A {
  A& operator=(A&&) = delete;
};

A makeA() { return A{}; }

int main() {
  auto a = makeA();
}
```

但是 MSVC 有 bug: https://godbolt.org/z/e973zrxPq

```cpp
struct A {
  A& operator=(A&&) = delete;
};

A msvcSb() { return true ? A{} : A{}; }

int main() {
  auto a = msvcSb();
}
```

> 仅限 msvc v19.41 VS17.11 以及更早的版本出现

## 2. MSVC 协程展开 bug

时间原因, 编写一个完整的协程然后放到 godbolt 上太麻烦了. 这里仅描述, 最小案例:

```cpp
template <typename... Awaiters>
Task<> func(Awaiters&&... awaiters) {
  ((co_await awaiters) && ...);
}
```

类似于这种, 在 C++17 的`折叠表达式`, 配合协程 `co_await`, 在 MSVC 上 (至少本机上), 是无效的.

即 MSVC 直接把 `((co_await awaiters) && ...);` 看作 `;` 了.

代码消失了. 即便你使用 `sizeof...(awaiters)` 然后 print 还是 调试.

你可以看到 `sizeof...(awaiters) != 0` 但是他并不会执行 `(co_await awaiters)`.

## 3. GCC 协程 co_return bug

> [!TIP]
> 我甚至不能确定是什么bug... 不知道怎么描述...

看码:

```cpp
// 似乎 GCC 下存在问题: 调试看到 _isUniqueEventLoop = ture, 但是却走 tryAsync 分支...
co_return _isUniqueEventLoop 
  ? _eventLoop->trySync(task())
  : co_await _eventLoop->tryAsync(task());

// 而下面却正常的:
if (_isUniqueEventLoop) {
  co_return _eventLoop->trySync(task());
}
co_return co_await _eventLoop->tryAsync(task());
```

对比: clang 运行上下代码都没问题, ctest通过. gcc 就不行, 调试看到了但还是走另一个分支. (debug 运行)