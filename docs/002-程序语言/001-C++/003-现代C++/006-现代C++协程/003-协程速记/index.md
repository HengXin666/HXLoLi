# 协程速记

虽然但是, 本篇需要你有具体的认知基础, 这里只是复习...

下面是比较好的参考:

- [协程 (C++20) | cppreference](https://zh.cppreference.com/w/cpp/language/coroutines)

- [许传奇 & 祁宇 & 韩垚, 2021 - C++20 协程原理和应用](https://zhuanlan.zhihu.com/p/497224333)

## 一、协程的过程

![协程的过程](./协程流程图.drawio.svg)

### 1.1 Awaiter

Awaiter 暂停体, 一个支持 `co_await` 的对象.

它需要实现三个函数, 均是以 `await_` 开头的:

> [!TIP]
> 协程的灵活主要就是由 `await_suspend` 决定的

```cpp [z1-简单的Awaiter]
/**
 * @brief 等价于标准库的 `std::suspend_always`/`suspend_never`
 */
template <bool Sleep>
struct SleepAwaiter {
    /**
     * @brief await SleepAwaiter{} 时候, 会首先调用 `await_ready()`, 并根据返回值见机行事
     * @return true  将调用 `await_resume()` 以恢复协程执行
     * @return false 将调用 `await_suspend()` 以暂停当前协程, 然后转移执行权
     */
    constexpr bool await_ready() const noexcept { return !Sleep; }
    constexpr auto await_suspend(std::coroutine_handle<>) const noexcept {}
    constexpr void await_resume() const noexcept {}
};
```

```cpp [z1-烧脑的Awaiter]
/**
 * @brief 默认的协程控制: 在协程挂起(`co_await`)时会退出整个协程链条
 */
template <class T, class P>
struct ExitAwaiter {
    using promise_type = P;

    explicit ExitAwaiter(std::coroutine_handle<promise_type> coroutine)
        : _coroutine(coroutine)
    {}

    bool await_ready() const noexcept { 
        return false; 
    }

    /**
     * @brief 挂起当前协程
     * @param coroutine 这个是`co_await`的协程句柄 (而不是 _coroutine)
     * @return std::coroutine_handle<promise_type> 
     */
    std::coroutine_handle<promise_type> await_suspend(
        std::coroutine_handle<> coroutine
    ) const noexcept {
        _coroutine.promise()._previous = coroutine; // 此处记录 co_await 之前的协程, 方便恢复
        return _coroutine;
    }

    /**
     * @brief co_await 继续
     * @return T 
     * @throw 可能会抛出异常, 正因为如此, 才可以像普通函数一样捕获co_await的异常 (本类设计)
     */
    T await_resume() const {
        return _coroutine.promise().result();
    }

    std::coroutine_handle<promise_type> _coroutine;
};
```

### 1.2 协程对象

对于一个协程对象 `Task`, 需要内部提供 `promise_type`, 并且 `promise_type` 需要至少实现 5 个函数, 分别是:

1. `initial_suspend`
2. `get_return_object`
3. `final_suspend`
4. `unhandled_exception`
5. `return_void` 或者 `return_value`
6. `yield_value` (可选的, 如果使用了 `co_yield` 就需要声明和实现)

> [!TIP]
> `get_return_object` 有两种写法, 一种是直接返回 `Task<T>`, 另一种是返回 `std::coroutine_handle<promise_type>`, 然后 `Task<T>` 中提供对应的构造. (而第一种方法是使用C++11的聚合初始化, 默认生成的构造)
>
> 个人建议使用第二种方法, 因为你可能会不小心定义了构造函数, 此外, 如果是 `Task<void>`, 你需要偏特化一个 `T = void` 的 `Task` 以支持 `return_void` (因为 `return_void` 和 `return_value` 不能同时声明实现), 而这样的 `Task<void>` 就是一个具体的类, 就不能使用 `模版的二阶段名称查找`, 就需要 `Task<void>` 必须在 `promise_type` 之前实现, 否则会报错!

```cpp [z2-协程对象]
#include <HXprint/print.h>
#include <coroutine>

template <bool Sleep>
struct SleepAwaiter {
    constexpr bool await_ready() const noexcept { return !Sleep; }
    constexpr auto await_suspend(std::coroutine_handle<>) const noexcept {}
    constexpr void await_resume() const noexcept {}
};

template <typename T>
struct NoVoidRes {
    using type = T;
};

template <>
struct NoVoidRes<void> {
    using type = NoVoidRes;
};

// 包裹一层, 防止 std::optional<void> 编译报错
template <typename T>
using NoVoidResType = NoVoidRes<T>::type;

template <typename T>
struct Promise;

template <typename T = void>
struct Task {
    // Task() = default; // 不合法, 会破坏聚合类的条件
    using promise_type = Promise<T>;

    std::optional<NoVoidResType<T>> operator()() noexcept {
        if (_coroitine.done()) {
            return {};
        }
        _coroitine.resume();
        if constexpr (std::is_void_v<T>) {
            return {};
        } else {
            return _coroitine.promise()._data;
        }
    }

    std::coroutine_handle<promise_type> _coroitine;
};

template <typename T>
struct Promise {
    SleepAwaiter<true> initial_suspend() noexcept { return {}; }
    Task<T> get_return_object() noexcept {
        return {std::coroutine_handle<Promise<T>>::from_promise(*this)};
    }
    SleepAwaiter<true> final_suspend() noexcept { return {}; }
    void unhandled_exception() { throw; }
    void return_value(T&& t) { _data = std::forward<T>(t); }
    SleepAwaiter<true> yield_value(T&& t) { _data = std::forward<T>(t); return {}; }
    std::optional<NoVoidResType<T>> _data{};
};

template <>
struct Promise<void> {
    SleepAwaiter<true> initial_suspend() noexcept { return {}; }
    Task<void> get_return_object() noexcept {
        return {std::coroutine_handle<Promise<void>>::from_promise(*this)};
    }
    SleepAwaiter<true> final_suspend() noexcept { return {}; }
    void unhandled_exception() { throw; }
    void return_void() noexcept { }
};

Task<int> test() {
    HX::print::println("test");
    co_yield 666;
    HX::print::println("test mo i kai~");
    co_return 2233;
}

Task<> testVoid() {
    HX::print::println("test return void~");
    // 注意没有 co_yield; 这种写法, 必须要 co_yield val;
    co_return;
}

int main() {
    {
        auto task = test();
        auto res = task();
        HX::print::println("R 1: res = ", res);
        res = task();
        HX::print::println("R 2: res = ", res);
        res = task();
        HX::print::println("End: res = ", res);
    }
    {
        auto task = testVoid();
        auto res = task();
        HX::print::println("End: res = ", res);
    }
    return 0;
}
```

### 1.3 协程中调用协程

考虑 `testVoid` 中, 如果需要 `co_await` 一个协程, 应该怎么办?

此处需要深刻理解 `Awaiter` 了!

- `auto operator co_await()` 类型转换函数, 其中 `auto` 返回的应该是一个 `Awaiter`.

然后就会调用该 `Awaiter` 的 `await_ready` 函数, 从而让它运行协程函数?

> [!TIP]
> 那么, `await_ready` 应该返回什么呢?
>
> 下面为了讲解方便, 我们举例: **从协程函数 `Task<>` 调用 -> 协程函数 `Task<string>`** (并且代码是在上面代码基础上编写的)

1. 返回 `true`, 会运行 `await_resume`, 然后就返回了! 即, 回到了 `Task<>`; 这不是我们期望的, 我们希望调用 `Task<string>`. 

2. 返回 `false`, 会运行 `await_suspend`, 那么这个函数是返回什么呢?

我直接宣布答案吧!

`await_suspend` 应该返回协程句柄(`std::coroutine_handle<>`), 而且还应该是 `Task<string>` 的协程句柄!

> [!TIP]
> 为什么呢? 
> 
> - 因为, 如果是返回 `await_suspend` 的参数 (此时是 `Task<>` 的协程句柄), 
> 
> - 亦或者是 `false` (直接运行 `await_resume`) 都是逻辑上有问题的, 因为此时 我们还没有返回值 (`Task<string>` 都还没有开始执行呢!), 会段错误.
>
> - 如果返回 `true`, 那么就是 **不执行** `Task<string>`
>
> 以上都不是我们期望的.

综上; 那么问题来了, 既然传参的 **不是** `Task<string>` 的协程句柄, 那么我们怎么获得 `Task<string>` 的协程句柄呢?

显然, 我们可以在 `auto operator co_await()` 返回的时候, 通过构造函数传入!

> [!NOTE]
> 你可能有疑问, 为什么 `operator co_await()` 传入的是 `Task<string>` 的协程句柄?
>
> 仔细思考一下, 你应该注意到, 此时实际上是 `co_await Task<string>{};`, 也就是 `Task<string>`, 那么显然 `operator co_await()` 也是调用 `Task<string>` 的函数.

现在你发现成功执行了 `Task<string>` 了, 但是返回后, 却没有继续执行 `Task<>` 而是直接返回了!

这是为什么呢? 原来是 `final_suspend` 的问题! 它返回了 `SleepAwaiter<true>` 也就是 `std::suspend_always`.

有难度的来啦!

那么怎么让它恢复执行之前的 `Task<>` 协程呢?

> [!TIP]
> 因为必需是 `Task<string>` 协程结束后才执行, 那么是谁, 在何处调用呢?
>
> 显然 `final_suspend` 就很合适, 它是在协程结束后执行的. 那么是谁调用呢?
>
> 注意到 `final_suspend` 的返回值是一个 `Awaiter`, 因此, 我们可以自定义 `Awaiter`, 让它恢复协程句柄

也就是 `Awaiter` 的 `await_suspend`, 我们如果可以返回之前的 `Task<>` 协程句柄就完美了!

现在问题就变成:

1. 怎么定义 `Awaiter`, 让它可以返回 **之前的** `Task<>` 协程句柄

2. 通用的, 如果 **不存在** 之前的协程句柄, 应该怎么操作?

现在我来回答:

1. 我们可以记录之前的协程句柄, 在 `co_await` 的 `Awaiter` 的 `await_suspend` 处, 因为此时 `await_suspend` 的传参正好就是 **调用者** 协程句柄, 因此我们直接把它缓存在 `promise_type` 中, 当 `promise_type` 调用 `final_suspend` 时候, 就可以 `return Awaiter{上一个协程句柄};` 这样传入. 然后 `Awaiter` 只要负责好 `await_suspend` 返回即可.

2. 如果 `上一个协程句柄` 不存在, 可以返回 `std::noop_coroutine()` 函数的调用结果(简单理解: 一个空的协程).

3. 怎么判断 `上一个协程句柄` 是不存在? 看: `1.4 std::coroutine_handle<>`

```cpp
#include <HXprint/print.h>
#include <coroutine>

template <bool Sleep>
struct SleepAwaiter {
    constexpr bool await_ready() const noexcept { return !Sleep; }
    constexpr auto await_suspend(std::coroutine_handle<>) const noexcept {}
    constexpr void await_resume() const noexcept {}
};

template <typename T>
struct NoVoidRes {
    using type = T;
};

template <>
struct NoVoidRes<void> {
    using type = NoVoidRes;
};

template <typename T>
using NoVoidResType = NoVoidRes<T>::type;

template <typename T>
struct Promise;

template <typename T = void>
struct Task {
    using promise_type = Promise<T>;

    std::optional<NoVoidResType<T>> operator()() noexcept {
        if (_coroutine.done())
            return {};
        _coroutine.resume();
        if constexpr (!std::is_void_v<T>) {
            return _coroutine.promise()._data;
        }
        return {};
    }

    template <bool Sleep>
    struct Awaiter {
        constexpr bool await_ready() const noexcept { return !Sleep; }
        constexpr auto await_suspend(std::coroutine_handle<> nowCoroutine) noexcept {
            _coroutine.promise()._mae = nowCoroutine;
            return _coroutine;
        }
        constexpr NoVoidResType<T> await_resume() const noexcept { 
            return *_coroutine.promise()._data;;
        }
        std::coroutine_handle<promise_type> _coroutine;
    };
    
    Awaiter<true> operator co_await() {
        return {_coroutine};
    }

    ~Task() noexcept {
        if (_coroutine)
            _coroutine.destroy();
    }

    std::coroutine_handle<promise_type> _coroutine{nullptr};
};

struct MaeAwaiter {
    constexpr bool await_ready() const noexcept { return false; }
    constexpr std::coroutine_handle<> await_suspend(std::coroutine_handle<>) const noexcept {
        if (_mae)
            return _mae;
        return std::noop_coroutine();
    }
    constexpr void await_resume() const noexcept {}
    std::coroutine_handle<> _mae{nullptr};
};

template <typename T>
struct Promise {
    SleepAwaiter<true> initial_suspend() noexcept { return {}; }
    Task<T> get_return_object() noexcept {
        return {std::coroutine_handle<Promise<T>>::from_promise(*this)};
    }
    MaeAwaiter final_suspend() noexcept { return {_mae}; }
    void unhandled_exception() { throw; }
    void return_value(T&& t) { _data = std::forward<T>(t); }
    SleepAwaiter<true> yield_value(T&& t) { _data = std::forward<T>(t); return {}; }
    
    std::optional<NoVoidResType<T>> _data{};
    std::coroutine_handle<> _mae{};
    ~Promise() noexcept { HX::print::println("~Promise"); }
};

template <>
struct Promise<void> {
    SleepAwaiter<true> initial_suspend() noexcept { return {}; }
    Task<void> get_return_object() noexcept {
        return {std::coroutine_handle<Promise<void>>::from_promise(*this)};
    }
    MaeAwaiter final_suspend() noexcept { return {_mae}; }
    void unhandled_exception() { throw; }
    void return_void() noexcept { }
    // SleepAwaiter<true> yield_value() { return {}; } // 这个不是协程函数

    std::coroutine_handle<> _mae{};
    ~Promise() noexcept { HX::print::println("~Promise<void>"); }
};

Task<int> test() {
    HX::print::println("test");
    co_yield 666;
    HX::print::println("test mo i kai~");
    co_return 2233;
}

Task<> testVoid() {
    HX::print::println("test return void {");
    auto res = co_await []() -> Task<std::string> {
        HX::print::println("in the co_await");
        co_return std::string{"123"};
    }();
    HX::print::println("} // test return void: ", res);
    co_return;
}

int main() {
    {
        auto task = test();
        auto res = task();
        HX::print::println("R 1: res = ", res);
        res = task();
        HX::print::println("R 2: res = ", res);
        res = task();
        HX::print::println("End: res = ", res);
    }
    {
        auto task = testVoid();
        auto res = task();
        HX::print::println("End: res = ", res);
    }
    return 0;
}
```

### 1.4 std::coroutine_handle<>

它是协程句柄的 `<void>` 特化, 可以使用 `nullptr` 进行初始化:

```cpp
coroutine_handle<> _coroutine{};
coroutine_handle<> _coroutine{nullptr}; // 等价
```

一般是作为 `传参`与 `存储协程句柄` 的, 如:

```cpp [z3-传参]
constexpr std::coroutine_handle<> await_suspend(std::coroutine_handle<>) const noexcept;
```

```cpp [z3-协程句柄]
coroutine_handle<> _coroutine{};
```

为什么可以作为 `存储协程句柄` 的呢? 我们使用的不是 `std::coroutine_handle<Promise<T>>` 吗?

那是因为, `coroutine_handle<T>` 有类型转换, 可以转换为 `coroutine_handle<>`.

```cpp
_EXPORT_STD template <class _Promise>
struct coroutine_handle {
    // ...
    constexpr operator coroutine_handle<>() const noexcept {
        return coroutine_handle<>::from_address(_Ptr);
    }
    // ...
};
```

因此, 可以使用其 `存储协程句柄`, 就像是一个 `视图` 一样, 非常的轻量. (从下面源码也可以看出来, 它只有一个 `void *` 大小)

```cpp [z4-源码 (MSVC)]
template <>
struct coroutine_handle<void> {
    constexpr coroutine_handle() noexcept = default;
    constexpr coroutine_handle(nullptr_t) noexcept {}

    coroutine_handle& operator=(nullptr_t) noexcept {
        _Ptr = nullptr;
        return *this;
    }

    _NODISCARD constexpr void* address() const noexcept {
        return _Ptr;
    }

    _NODISCARD static constexpr coroutine_handle from_address(void* const _Addr) noexcept { // strengthened
        coroutine_handle _Result;
        _Result._Ptr = _Addr;
        return _Result;
    }

    constexpr explicit operator bool() const noexcept {
        return _Ptr != nullptr;
    }

    _NODISCARD bool done() const noexcept { // strengthened
        return __builtin_coro_done(_Ptr);
    }

    void operator()() const {
        __builtin_coro_resume(_Ptr);
    }

    void resume() const {
        __builtin_coro_resume(_Ptr);
    }

    void destroy() const noexcept { // strengthened
        __builtin_coro_destroy(_Ptr);
    }

private:
    void* _Ptr = nullptr;
};
```