---
authors: Heng_Xin
title: HXLibs 协程串行调度器探索
date: 2026-06-04 00:22:00
tags:
    - C++
    - 协程
    - HXLibs
---

# HXLibs 协程串行调度器探索

> 记录 `SerialExecutor` 三轮迭代的探索过程与踩坑经验。当前版本仍非最终方案, 调度策略尚有优化空间。
>
> 相关提交:
> - [`50c931c`](https://github.com/HengXin666/HXLibs/commit/50c931ce0324501c5de0df668cfd7e028282e850) — [feat] 实现了串行调度器 (生命周期敏感)
> - [`ac74126`](https://github.com/HengXin666/HXLibs/commit/ac741268248b8430117a883d6e7b884f0e245989) — [fix] 修复 constexpr 编译报错
> - [测试用例](https://github.com/HengXin666/HXLibs/blob/50c931c/tests/coroutine/04_SerialExecutor.cpp)

## 0x00 调度点: 核心思想

在 C++20 无栈协程中实现串行调度, 核心难点在于 **链式调度中的控制流分裂**。为此引入「调度点」概念来简化思考:

> **调度点**: 协程恢复执行后所处的位置。一个协程中每个 `co_await` 都是一个潜在的调度点。

假设要串行执行 3 个协程 `[A, B, C]`:

```
初始状态: 3 个独立调度点(各自可以并发)
    ┌── A ──┐  ┌── B ──┐  ┌── C ──┐
    │  ...  │  │  ...  │  │  ...  │
    └───────┘  └───────┘  └───────┘

进入串行队列后: 调度点收归为 1
    A → B → C
    └────────┘  (FIFO 队列, 同一时刻只有一个在执行)

任务 A 完成后: 调度点分裂为 2
    ├─→ 恢复 A 的调用者 (返回)
    └─→ 启动 B            (继续串行)
```

**核心操作**就在 `start()` 中 `co_await task` 之后的这两行:

```cpp
co.resume();       // 分裂点 1: 回到调用者
// ...
co_await start();  // 分裂点 2: 继续下一个串行任务
```

用调度点视角来看整个设计, 三个版本的差异就非常清晰了——它们本质上是在探索 **「谁持有运行中的调度点」** 和 **「调度点如何被清理」** 这两个问题。

## 0x01 v1: `_SerialScheduler` — 简陋但可行

> 文件: [`_SerialScheduler.hpp`](https://github.com/HengXin666/HXLibs/blob/50c931c/include/HXLibs/coroutine/executor/_SerialScheduler.hpp), 日期 2026-05-30

第一个版本的目标是 **跑通**: 不关心优雅性, 先让串行调度工作起来。

### 核心数据结构

```cpp
std::queue<std::tuple<Task<> const&, std::coroutine_handle<>>> _q;  // 等待队列
std::list<BackgroundTask> _btoq;   // 后台任务所有权队列
std::size_t _runCnt{};             // 当前运行的任务数 (0 或 1)
```

- `_q`: FIFO 任务队列, 存储的是外部 `Task<>` 的 **引用** + 被挂起的调用者协程句柄
- `_btoq`: 所有权队列 —— 调度器自己创建的链式任务必须有人持有, 否则协程帧会泄漏或悬挂
- `_runCnt`: 充当简易的自旋锁(只能为 0 或 1)

### SerialAwaiter: 调度入口

```cpp
bool await_suspend(std::coroutine_handle<> co) const noexcept {
    if (!_mtx._runCnt) {         // 没有任务在跑 → 我来当第一个
        ++_mtx._runCnt;
        _mtx._q.push({_task, std::noop_coroutine()});
        return false;             // 不挂起, 立刻开始执行
    }
    _mtx._q.push({_task, co});   // 已有任务在跑 → 排队 + 挂起
    return true;
}
```

关键点: 第一个入队的任务不挂起 (`return false`), 直接在当前协程帧中继续。后续任务则挂起到队列中。

### start(): 链式调度核心

```cpp
Task<> start() {
    auto [task, co] = std::move(_q.front());
    _q.pop();
    co_await task;                // 执行任务 → 从此处恢复时, 任务已完成
    co.resume();                  // 调度点分裂①: 恢复被挂起的调用者
    --_runCnt;
    if (_q.empty()) co_return;    // 队列空, 结束链式调度
    // 启动下一个后台任务 —— 这就是分裂点②
    _btoq.insert(_btoq.end(), [this, co]() -> BackgroundTask {
        co_await start();         // 递归链式调用
    }())->via(_btoq).detach();
}
```

### v1 的问题

1. **依赖 EventLoop** — `serial()` 结尾有 `co_await _loop.makeTimer().yield()`, 耦合了事件循环
2. **`_runCnt` 语义模糊** — 用计数器做布尔量的事情, 意图不清晰
3. **BackgroundTask 太重** — 为了 RAII 引入了完整的 promise_type + task 类型, 增加了概念负担
4. **析构不安全** — 如果 `_btoq` 在协程运行中被清空, `BackgroundPromise::final_suspend()` 中注释掉的 `_btoq->erase(_it)` 暗示了这里有过问题

## 0x02 v3: `SerialExecutorPro` — 尝试用 SerialTask 简化

> 文件: [`SerialExecutorPro.hpp`](https://github.com/HengXin666/HXLibs/blob/50c931c/include/HXLibs/coroutine/executor/SerialExecutorPro.hpp), 日期 2026-05-31
>
> ⚠️ 实际迭代路径是 v1 → v3 → v2。v3 是一次「把 start() 返回 SerialTask」的实验, 最终被废弃。

### 改动动机

v1 的 `BackgroundTask` 和 `Task<>` 是两套不同的协程类型, 让代码膨胀。能否让 `start()` 直接返回一个 **可被 co_await 的 SerialTask**, 统一调度链上的所有协程?

### 核心改动

```cpp
// SerialTask 现在可以作为 awaiter:
constexpr ExitAwaiter<void, promise_type> operator co_await() const noexcept {
    return ExitAwaiter<void, promise_type>{_handle};
}

// start() 返回 SerialTask 而非 Task<>
SerialTask start() {
    // ...
    co_await task;
    co.resume();
    // ...
    [this, &runList]() -> SerialTask {
        co_await start().via(runList);  // 链式: co_await SerialTask
    }().via(runList).detach();
}

// serial() 中:
co_await start().via(_runList->runList);  // 直接 co_await SerialTask
co_await _loop.makeTimer().yield();
```

### 废弃原因

从代码的 `@todo 一个废弃的方案` 可以看出这不是最终选择。推测原因:

1. **SerialTask 复用 `ExitAwaiter`** 导致语义复杂 — `ExitAwaiter` 的 `await_suspend` 会记录 `_previous`, 这对于链式调度是多余的
2. **依然依赖 EventLoop** — `yield()` 没有被移除
3. **`.via().detach()` 链式调用** 在 `start()` 中出现了两次, 使用模式不一致

不过 v3 贡献了关键改进: **`shared_ptr<State>`** 管理内部状态, 解决了 v1 中 `_btoq` 所有权混乱的问题。

## 0x03 v2: `SerialExecutor` — 回归与收敛

> 文件: [`SerialExecutor.hpp`](https://github.com/HengXin666/HXLibs/blob/ac74126/include/HXLibs/coroutine/executor/SerialExecutor.hpp), 日期 2026-05-31
>
> 在 [`ac74126`](https://github.com/HengXin666/HXLibs/commit/ac741268248b8430117a883d6e7b884f0e245989) 中修复了 `await_suspend` 缺少 `constexpr` 的编译错误 (MSVC)。

v2 保留了 v3 的 `shared_ptr<State>`, 但将 `start()` 恢复为返回 `Task<>`, 并移除了 EventLoop 依赖。

### 状态管理: `shared_ptr<State>`

```cpp
struct State {
    bool isRun{};
    std::queue<std::tuple<Task<> const&, std::coroutine_handle<>>> q;
    std::list<SerialTask::HandleType> runList;
};

const std::shared_ptr<State> _runList = std::make_shared<State>();
```

这是 v2 最关键的架构决策: **State 的生命周期与 SerialExecutor 对象解耦**。即使 `SerialExecutor` 被移动/销毁, 只要还有 `shared_ptr<State>` 存活, 正在运行的串行链就不会段错误。`clear()` 在析构函数中安全地销毁所有悬挂的协程句柄。

### SerialTask: 更轻量的 RAII

```cpp
struct SerialTask {
    struct promise_type {
        StopAwaiter<false> final_suspend() noexcept {
            _list->erase(_it);   // 完成后自动从 runList 中移除
            return {};
        }
        // ...
        std::list<HandleType>* _list{};
        std::list<HandleType>::iterator _it;
    };
    // ...
};
```

关键设计: `SerialTask` 在 `final_suspend` 中 **自动从 `runList` 中删除自己**。这意味着:
- 正常完成 → `erase` 触发 → `SerialTask` 析构 → 协程帧释放
- `clear()` 调用 → 遍历 `runList` 手动 `destroy()` 所有句柄

### 对比 v1 的改进

| 维度 | v1 | v2 |
|------|----|----|
| 运行标记 | `_runCnt` 计数器 | `isRun` 布尔 |
| 状态所有权 | `_btoq` (裸 list) | `shared_ptr<State>` |
| EventLoop | 需要 | 不需要 |
| 后台任务析构 | `BackgroundTask::~BackgroundTask()` | `SerialTask::promise_type::final_suspend()` 自清理 |
| `clear()` 安全性 | 仅清空队列 | 清空队列 + 手动 destroy runList 中所有句柄 |

### start() 的链式调度

```cpp
Task<> start() {
    auto runListPtr = _runList;
    auto& [isRun, q, runList] = *runListPtr;
    auto [task, co] = std::move(q.front());
    q.pop();
    co_await task;
    co.resume();                          // 调度点分裂①
    if (!isRun || q.empty()) {
        isRun = false;
        co_return;
    }
    [this]() -> SerialTask {
        co_await start();                 // 调度点分裂②: 递归
    }().via(runList).detach();
}
```

注意 `runListPtr = _runList` 的拷贝 —— 即使在链式调度过程中 `this` 被析构 (`_runList` 引用计数减 1), 局部的 `runListPtr` 仍持有状态, 保证不会访问已释放的内存。

## 0x04 所有权与生命周期

串行调度器最棘手的部分是协程帧的所有权。这里解释 HXLibs 中三种关键的所有权语义。

### Task<> 的所有权

```cpp
template <typename T, typename P = Promise<T>, typename Awaiter = ExitAwaiter<T, P>>
struct Task {
    ~Task() noexcept {
        if constexpr (std::is_same_v<typename P::DeleStrategy, StopAwaiter<false>>) {
            return;  // DeleStrategy = StopAwaiter<false> → 不析构 (detach 语义)
        }
        // 否则: RAII 析构协程帧
        if (_handle) {
            _handle.destroy();
        }
    }
};
```

- 默认 `Promise<T>` 的 `DeleStrategy` 是 `PreviousAwaiter` → **析构时 destroy**
- `Promise<T, Init, StopAwaiter<false>>` → **析构时不 destroy**(用于 detach 场景)

这就是 `SerialTask` 的 promise_type 不设置显式 `DeleStrategy` 的原因 —— 它依赖 `runList` 中的 `erase` 来触发自然析构。

### 为什么 `_q` 只存引用

```cpp
std::queue<std::tuple<Task<> const&, std::coroutine_handle<>>> _q;
```

队列存 `Task<> const&` 而非 `Task<>` 值, 有两个原因:
1. **避免所有权转移**: 调用者保有 `Task<>` 的所有权, 调度器只是借用
2. **防止析构导致的段错误**: 如果 `_q` 持有 `Task<>` 值, 队列清空时会触发 `~Task()` → `handle.destroy()`, 而此时协程可能还在被其他地方引用

`@warning` 注释特别指出: *需要保证在有任务的时候, t 应该在其生命周期内*。这也是 `whenAny` 使用需要小心的原因 —— 如果 `whenAny` 提前退出, 其内部的 `Task<>` 被析构, 而调度器还持有引用, 就会触发野指针。

### 为什么 v1 需要 BackgroundTask

v1 中 `start()` 递归调用自身创建了新的协程帧:

```cpp
_btoq.insert(_btoq.end(), [this, co]() -> BackgroundTask {
    co_await start();
}())->via(_btoq).detach();
```

这个 lambda 创建了一个协程帧, 它必须有所有者。`_btoq` (list of `BackgroundTask`) 就是这个角色 —— 每个 `BackgroundTask` 持有一个 `coroutine_handle`, 析构时 `destroy()`。

v2 中这个角色由 `SerialTask` + `runList` 替代: `SerialTask` 本身也是 RAII 持有句柄, 但额外通过 promise 将自己注册到 `runList` 中, `final_suspend` 时自动注销。

## 0x05 踩坑记录

### 坑 1: 协程析构与 Timer

代码注释中提到: *"目前是解决了 timer 的问题(协程析构也有踩坑, 很深)"*

当 `_q` 中的 `Task<> const&` 指向一个内部使用了 `co_await timer` 的 Task, 而该 Task 在 timer 触发前被析构时:
- Timer 的回调可能仍然持有已析构的协程句柄
- 需要确保 `clear()` 时同时取消定时器

v2 移除了 EventLoop 依赖后, 这个问题交给了调用者管理 —— 调用者需要保证传入的 Task 在串行完成前不析构。

### 坑 2: `_runCnt--` 的玄学段错误

v1 的 `start()` 中有注释:

```cpp
--_runCnt; // 不能在 resume() 后使用 _q.pop(), 会导致玄学段错误 (MSVC)
```

这是因为 `co.resume()` 之后, 被恢复的协程可能立刻再次调用 `serial()` → 修改 `_q`。如果此时继续使用 `_q.pop()` 或访问 `_q`, 会产生数据竞争或 use-after-move。v2 通过局部变量捕获 `_runList` 的 shared_ptr 副本解决了此问题。

### 坑 3: whenAny + 串行调度的组合

用户明确指出:

> "如果是 whenAny, 后面退出了, 内部还有在 co_await 的, 就会 free 野指针了"

```cpp
// 危险场景:
co_await whenAny(
    serialExecutor.serial(someTask),  // 内部持有 someTask 的引用
    timeoutTask                       // 超时先触发
);
// whenAny 返回 → serialExecutor 中的 Task 引用悬空
```

v2 的 `clear()` 可以清理队列, 但 **AIO task 没有取消机制** —— 如果 `co_await` 已经在异步 I/O 中, 无法撤销。用户提到需要用超时取消模式处理, 这属于后续优化方向。

### 坑 4: `_btoq` 的自删除困境

v1 的 `BackgroundPromise::final_suspend()` 中有一行被注释掉的代码:

```cpp
// _btoq->erase(_it);
```

这暴露了设计上的尴尬: 如果 BackgroundTask 在 `final_suspend` 时从 `_btoq` 中删除自己, 会导致自身析构, 而 `final_suspend` 还没返回 —— 这是一个微妙的生命周期问题。最终选择了让 `_btoq` 持有所有权直到外部清空。

## 0x06 待优化方向

用户确认当前版本**并非最终方案**, 以下方向待探索:

1. **AIO 任务的取消机制**: 当前 `clear()` 无法取消已在执行的异步 I/O, 需要超时取消或协作式取消
2. **调度策略优化**: 当前是严格 FIFO, 是否支持优先级?
3. **`shared_ptr<State>` 的成本**: 每次 `serial()` 都会原子操作引用计数, 可以考虑用侵入式引用计数或 RCU
4. **与 `whenAll` / `whenAny` 的安全组合**: 需要明确的生命周期契约

## 0x07 小结

三轮迭代的演进路线 (v1 → v3 → v2) 本质上是在回答三个问题:

1. **调度点如何分裂?** → `co_await task` 后 `co.resume()` + `co_await start()`
2. **谁持有运行中的调度点?** → v1 用 `_btoq`, v2/v3 用 `runList` + promise RAII
3. **状态如何安全共享?** → v2 用 `shared_ptr<State>` 解耦生命周期

核心思想「调度点」—— 串行化就是调度点收归, 任务完成后调度点分裂 —— 贯穿了整个设计过程。
