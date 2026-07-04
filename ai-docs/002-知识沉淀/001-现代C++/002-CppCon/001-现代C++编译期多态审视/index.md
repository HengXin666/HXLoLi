---
title: "现代C++编译期多态审视"
created_at: "2026-07-04"
model: "GPT-5"
skill: "hx-look-video"
authors: "Heng_Xin"
tags: ["现代C++", "C++20", "编译期多态", "接口设计"]
---

# 现代C++编译期多态审视

> [!NOTE]
> 本文由 AI 辅助沉淀, 需要用户 review 后再提交.

## 0x00 这篇视频真正要回答什么?

视频讨论的不是一句粗暴的「以后别写 `virtual`」, 而是一个更有工程价值的问题:

> 当程序的类型集合、接口约束和调度路径能够在编译期表达时, 是否还需要把多态性推迟到运行期?

来源信息:

- 视频: [C++虚函数可以彻底抛弃了? 现代C++编译期多态实战 | CppCon](https://www.bilibili.com/video/BV1gqTs66EpK/)
- UP: 程序员-智能译站
- 发布时间: 2026-07-03 16:19:59
- 分 P: P1 `[中文] C++虚函数可以彻底抛弃了？现代C++编译期多态实战 | CppCon`
- Transcript 来源: FunASR ASR, 因 Bilibili 公开视频 `subtitle.list` 为空
- 中间产物: `/tmp/hx-look-video/manual-BV1gqTs66EpK-p1/transcript.md`

ASR 备注: transcript 中把一些 C++ 术语识别错了, 例如「基类」常被识别成「鸡类」, `variant` 被识别成近似音. 本文按 C++ 语境修正术语, 但观点依据仍来自转写内容.

## 0x01 核心结论

从 C++20 起, `virtual` 更像是一种明确的运行期边界工具, 而不是默认接口建模工具.

视频把虚函数常见用途拆成三类:

1. 约束对象必须提供某个接口.
2. 根据配置切换对象行为.
3. 在同一个容器中保存多种派生类型.

对应的现代 C++ 替代路径是:

- 用 `concept` / `requires` 绑定接口, 替代纯虚基类作为接口契约.
- 用类模板 + CTAD 保存具体类型, 把类型选择留在编译期.
- 确实需要有限运行期选择时, 用 `std::variant<Ts...>` 表达封闭类型集合.
- 需要保存多种类型的集合时, 用 `std::tuple<std::vector<Ts>...>` 或类似结构按类型分桶.

我的审视结论:

- 如果类型集合是封闭的, 且性能、局部性、编译期检查重要, 优先考虑 C++20 静态多态.
- 如果类型集合由插件、ABI、动态库、脚本层、用户扩展决定, `virtual` 仍然是合理边界.
- 如果为了不用 `virtual` 结果手写类型擦除、手写 vtable 或到处传 `void*`, 那通常是在退化设计.

## 0x02 `virtual` 的问题不是慢, 而是动态性被强加

视频开头强调, 虚函数当然有用. 它自然地支持接口约束、行为配置和异构容器. 但它也把一些成本和约束强加给调用方:

- 常常需要指针或引用来承载动态分发.
- 需要考虑空指针、对象所有权、虚析构、向下转型等运行期风险.
- 编译器更难看穿真实调用目标, 优化空间受限.
- 接口表达容易被基类形状绑死.

对应 transcript 时间点:

- `00:00:20` 讲到虚函数的第一类用途是表达特定接口.
- `00:00:33` 讲到虚对象可用于配置行为.
- `00:00:50` 讲到通过共同基类把多种派生类型放进同一容器.
- `00:01:13` 开始解释虚基类通常迫使你使用指针.
- `00:01:31` 讲到更静态的描述能提前暴露错误.
- `00:01:59` 讲到静态描述能给编译器更多优化机会.

这不是说间接访问一定错误, 而是说如果业务并不需要运行期开放性, 那么动态分发就是被过早引入的复杂度.

## 0x03 用 `concept` 绑定接口

视频第一组例子是「接口绑定」. 传统写法大概是:

```cpp
struct FooInterface {
    virtual ~FooInterface() = default;
    virtual int func() = 0;
};

struct Foo : FooInterface {
    int func() override;
};
```

C++20 的表达方式可以是:

```cpp
template <class T>
concept FooLike = requires(T value) {
    { value.func() } -> std::same_as<int>;
};

template <FooLike T>
int use(T const& value) {
    return value.func();
}
```

这带来的变化:

- 接口从「必须继承某个基类」变成「满足某组语义约束」.
- 对象可以按值传递, 不必为了多态引入指针.
- 不需要虚析构时, 类型可以回到 Rule of Zero.
- 错误会以 concept 约束失败的形式出现在编译期.

视频在 `00:05:38` 附近引入 concept 绑定接口, `00:06:07` 强调没有虚函数后可以依赖零法则, `00:06:38` 之后说明 concept 比虚接口绑定更松散.

现代 C++ 视角下, 这其实是把「接口」从继承层级中解耦出来. 这非常适合:

- 泛型算法.
- policy / strategy 这类静态策略注入.
- 内部基础库的可组合能力约束.
- 测试中注入假类型.

但它不适合直接替代 ABI 边界. `concept` 是编译期机制, 不提供运行期稳定接口.

## 0x04 CTAD 让对象配置留在编译期

第二组例子处理「一个对象内部持有某种实现」.

传统虚函数版本通常持有 `std::unique_ptr<Base>` 或 `Base&`. 非虚版本可以让宿主类型参数化:

```cpp
template <FooLike T>
class Holder {
public:
    explicit Holder(T value)
        : value_(std::move(value)) {}

    int run() {
        return value_.func();
    }

private:
    T value_;
};

Holder holder{Foo{}};
```

借助 CTAD, 使用方不需要手写 `Holder<Foo>`. 类型在构造时确定, 后续就不再改变.

视频在 `00:09:58` 到 `00:10:20` 讲这个思路, 并在 `00:10:38` 附近补上 concept 约束, 让模板参数不仅能被推导, 还必须满足接口契约.

这里的关键取舍是:

- 如果对象实现类型在构造时已经确定, 模板持有具体类型更直接.
- 如果对象实现类型必须运行期替换, 则需要继续考虑 `std::variant`、类型擦除或 `virtual`.

## 0x05 `std::variant` 只适合封闭的运行期选择

视频没有否认运行期配置. 它给出的现代 C++ 方案是:

```cpp
template <FooLike... Ts>
class RuntimeHolder {
public:
    template <class T>
        requires SameAsAny<std::remove_cvref_t<T>, Ts...>
    void set(T&& value) {
        value_ = std::forward<T>(value);
    }

    int run() {
        return std::visit([](auto& value) {
            return value.func();
        }, value_);
    }

private:
    std::variant<Ts...> value_;
};
```

这里的重点不是 `variant` 很神奇, 而是它表达了一个封闭集合: 运行时可以在 `Ts...` 里切换, 但不能出现列表之外的类型.

视频在 `00:11:45` 之后引入类型列表和 `variant`, `00:12:29` 讲到直接赋一个 variant 不接受的类型会产生很难看的模板错误, 所以后续用类似 `SameAsAny` 的 concept 把错误前移为约束失败.

需要警惕:

- `std::variant` 仍然有运行期分派成本.
- 如果类型集合经常变化, 维护 `Ts...` 会变成负担.
- 如果外部用户能扩展类型集合, `variant` 不再自然.

因此我的判断是: `variant` 是封闭世界里的动态选择, 不是开放多态的万能替代品.

## 0x06 异构容器: 从 `vector<Base*>` 到 `tuple<vector<Ts>...>`

第三类问题是异构集合.

传统方案:

```cpp
std::vector<std::unique_ptr<Base>> objects;
```

静态多态方案:

```cpp
template <FooLike... Ts>
class Store {
public:
    template <class T>
        requires SameAsAny<std::remove_cvref_t<T>, Ts...>
    void push(T&& value) {
        auto& bucket = std::get<std::vector<std::remove_cvref_t<T>>>(items_);
        bucket.push_back(std::forward<T>(value));
    }

private:
    std::tuple<std::vector<Ts>...> items_;
};
```

视频在 `00:23:37` 之后把每种类型分别存在一个 `vector<T>` 中, 即 `tuple<vector<Ts>...>`. 它在 `00:25:23` 之后强调, 如果每种类型都有很多对象, 这种结构有很好的静态局部性, 编译器也能更深入理解操作.

这个结构的优势:

- 每个 `vector<T>` 连续存储真实对象, 缓存局部性好.
- 不需要基类指针, 不需要额外分配每个对象.
- 对某个具体类型批量处理时, 能得到更直接的代码生成.

它的代价:

- 每种类型至少有一个空 `vector` 的开销.
- 跨类型插入顺序天然丢失.
- 算法需要适配 `tuple` 展开和分桶结构.

视频在 `00:26:04` 到 `00:28:15` 讨论了顺序问题: 单一类型内部顺序稳定, 但类型之间的相对顺序丢失. 如果业务依赖全局稳定顺序, 需要额外元数据, 也就重新引入存储成本.

## 0x07 缺点: 编译成本、二进制体积和维护门槛

视频在 `00:29:26` 之后集中讲缺点:

- 模板实例化会增加编译单元大小.
- 类型列表到处传播会放大代码可见范围.
- 二进制体积可能变大.
- 编译时间和编译器内存占用可能显著上升.
- 元编程会提高维护门槛.
- 如果没有 LTO 等优化, 链接器未必能很好消除重复.

这部分对现代 C++ 工程尤其重要. 静态多态不是免费午餐, 它只是把成本从运行期转移到编译期和源码复杂度上.

我的落地标准:

- 热路径、低延迟、固定类型集合: 值得考虑.
- 普通业务层、非性能瓶颈、团队模板能力一般: 不要为了范式纯粹性替换 `virtual`.
- ABI 边界、动态插件、第三方扩展: 保留 `virtual` 或明确类型擦除.
- 需要跨类型稳定顺序: 不要盲目分桶, 先确认访问模式.

## 0x08 关于「C++20 后不需要 virtual」的边界

视频在 `00:32:34` 提出大胆主张: 从 C++20 开始, 如果二进制从源码一起构建, 可以不必使用 `virtual`. 但讲者立刻排除了动态加载、不可见代码和链接期无法获得源码的情况.

我认为这句话应该被改写成:

> 在封闭世界假设下, C++20 已经提供足够好的语言工具, 让很多原本依赖 `virtual` 的设计可以转为编译期多态.

这不是:

> 所有 `virtual` 都应该被删除.

视频在 `00:38:19` 之后也提醒, 不要第二天就去代码库里搜索 `virtual` 然后挨个提 PR. 这些技巧只是工具箱中的工具, 尤其适合低延迟场景; 如果关注吞吐量或不关心性能, 收益就未必明显.

## 0x09 设备管理例子给出的工程启发

最后的练习是网络设备管理:

- 每种设备有独立发现逻辑.
- 每个设备实例有自己的状态.
- 需要周期性 `update`.
- 设备类型集合在系统里相对明确.

虚函数方案能统一存成 `vector<unique_ptr<DeviceInterface>>`, 但静态函数 `find` 不能成为虚接口的一部分. 视频在 `00:42:57` 到 `00:43:16` 明确指出这个限制.

C++20 方案可以把静态发现函数也放入 concept:

```cpp
template <class T>
concept Device = requires(T device, Environment& env) {
    { T::find(env) } -> std::same_as<std::vector<T>>;
    { device.update() } -> std::same_as<void>;
};
```

然后用类型列表约束设备管理器:

```cpp
template <Device... Ts>
class DeviceManager {
public:
    explicit DeviceManager(Environment& env)
        : devices_(find_all<Ts...>(env)) {}

    void update() {
        std::apply([](auto&... buckets) {
            (update_bucket(buckets), ...);
        }, devices_);
    }

private:
    std::tuple<std::vector<Ts>...> devices_;
};
```

视频在 `00:47:01` 之后说明, concept 能约束 `update`, 也能约束静态 `find`. 这是纯虚接口很难自然表达的地方.

但 `DeviceManager` 的访问顺序变成类型列表顺序, 不是设备发现顺序. 视频在 `00:49:49` 之后指出: 这可能是优点, 也可能是缺点. 如果某些设备更重要, 可以把它的类型放前面; 如果真实发现顺序有语义, 这种结构就要谨慎.

## 0x0A 我会怎么在 HXLibs / 现代 C++ 项目里使用这套思路?

在自己的现代 C++ 代码里, 我会把选择流程写成这样:

1. 先问类型集合是不是封闭的.
   - 是: 可以考虑 `concept + template`.
   - 否: 优先保留运行期多态边界.

2. 再问调用路径是不是热路径.
   - 是: 静态多态、分桶存储、去指针化值得尝试.
   - 否: 可读性和 ABI 稳定性优先.

3. 再问是否需要跨类型统一顺序.
   - 是: `tuple<vector<Ts>...>` 需要额外顺序元数据, 收益会被稀释.
   - 否: 分桶结构能换来更好局部性.

4. 最后问团队是否能维护模板错误和元编程.
   - 能: 用 concept 把约束写清楚.
   - 不能: 不要把业务代码变成模板迷宫.

这套思路对 HXLibs 这类偏底层的现代 C++ 库尤其有价值. 库内部可以用 concept 表达协议和约束, 用模板保留静态信息; 对外边界则需要根据用户扩展模型决定是否暴露 `virtual`、类型擦除或模板接口.

## 0x0B 时间线摘要

- `00:00:00`: 引入主题: 如何消除虚函数, 先分析虚函数的三个用途.
- `00:01:13`: 虚函数常迫使指针间接访问, 静态描述能提前暴露错误.
- `00:04:15`: 开始讲接口绑定.
- `00:05:38`: 用 concept 替代纯虚接口表达函数约束.
- `00:07:42`: 对比虚函数版本的指针操作和非虚版本的直接对象传递.
- `00:09:18`: 进入多态类型持有问题.
- `00:10:10`: 使用 CTAD 让宿主类型保存具体实现类型.
- `00:11:45`: 引入类型列表和 `std::variant` 支持封闭集合内的运行期切换.
- `00:14:48`: 强调 `variant` 仍有运行期查找成本, 不应无理由使用.
- `00:23:37`: 用 `tuple<vector<Ts>...>` 保存多种类型集合.
- `00:25:23`: 分桶结构带来局部性和编译器可见性.
- `00:26:04`: 讨论跨类型顺序丢失.
- `00:29:26`: 讨论编译时间、二进制体积和复杂性代价.
- `00:32:34`: 提出「封闭源码构建下未必需要 virtual」的主张.
- `00:38:19`: 提醒不要机械删除代码库里的 `virtual`.
- `00:42:49`: 用设备管理例子对比虚函数和 concept 方案.
- `00:47:01`: concept 可以绑定静态 `find` 和实例 `update`.

## 0x0C 后续可验证问题

- 用一个小 benchmark 对比三种结构:
  - `vector<unique_ptr<Base>>`
  - `vector<variant<Ts...>>`
  - `tuple<vector<Ts>...>`
- 对比 GCC / Clang 在 `-O3`、`-flto`、PGO 下的去虚拟化效果.
- 测量二进制体积和编译时间, 不只测运行速度.
- 在 HXLibs 中找一个真实接口, 尝试分别写成 `virtual`、concept 模板接口、类型擦除三版, 比较可维护性.

## 0x0D 概要

如果一个系统的类型集合本来就是封闭的, 为什么还要把接口检查、对象布局和调用目标都推迟到运行期? C++20 的 `concept`、CTAD、`variant` 和参数包展开让我们可以把很多「多态」重新表达为编译期事实. 但这不是对 `virtual` 的道德审判, 而是一种边界识别: 当动态性是业务事实时, `virtual` 仍然清晰; 当动态性只是历史写法时, 现代 C++ 给了我们更静态、更可优化、更早失败的选择.
