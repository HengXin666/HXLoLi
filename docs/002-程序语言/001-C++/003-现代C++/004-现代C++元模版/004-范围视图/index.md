# 剖析 std::ranges / std::views 库的胖次

## 一、起因

最近打算学习一下 C++26 的 std::executor 库 (看看如何把 HXLibs 的接口对其兼容, 以及写一个该接口规范的线程池), 发现其一些内容实际上思想是和 std::ranges / std::views 库相关的, 所以决定先学习一下 std::ranges / std::views 库.

学习之后, 发现我对 std::ranges 这些认知都太浅了. 总觉得就仅仅是 记住这海量的 api 即可...

然后知道它是 范围的(begin/end 不用写了)、 管道符的、lazy 的、编译期的.

说实话有点抽象? 或者是根本无法适应使用

## 二、试试实践

我让 GPT 给我一个企业级别的实际意义的原始代码 (非ranges的), 然后我自己修改为 ranges 的代码. 看看仅以我的认知是否可以进行?

```cpp [ranges01-老登写法]
struct Order {
    int id;
    int userId;
    double price;
    bool valid;
    std::string tag;
};

struct NoRangesTest {
    std::vector<int> process(const std::vector<Order>& orders) {
        std::vector<int> result;
        // step1: 收集有效订单
        std::vector<Order> validOrders;
        for (size_t i = 0; i < orders.size(); ++i) {
            if (orders[i].valid == true) {
                validOrders.push_back(orders[i]);
            }
        }
        // step2: 过滤 price > 100 且 tag != "test"
        std::vector<Order> filtered;
        for (size_t i = 0; i < validOrders.size(); ++i) {
            if (validOrders[i].price > 100.0) {
                if (validOrders[i].tag != "test") {
                    filtered.push_back(validOrders[i]);
                }
            }
        }
        // step3: 按 price 排序(降序)
        std::sort(
            filtered.begin(), filtered.end(),
            [](const Order& a, const Order& b) { return a.price > b.price; }
        );
        // step4: 去重 userId(保留第一次出现的)
        std::unordered_set<int> seen;
        std::vector<Order> dedup;
        for (size_t i = 0; i < filtered.size(); ++i) {
            if (seen.find(filtered[i].userId) == seen.end()) {
                seen.insert(filtered[i].userId);
                dedup.push_back(filtered[i]);
            }
        }
        // step5: 取前 10 个
        std::vector<Order> top;
        for (size_t i = 0; i < dedup.size(); ++i) {
            if (i < 10) {
                top.push_back(dedup[i]);
            }
        }
        // step6: 提取 userId
        for (size_t i = 0; i < top.size(); ++i) {
            result.push_back(top[i].userId);
        }
        return result;
    }
};
```

```cpp [ranges01-现代写法]
struct RangesTest {
    auto process(std::vector<Order> const& orders) {
        auto filtered = orders | views::filter([](Order const& o) {
            // step1: 收集有效订单
            return o.valid
            // step2: 过滤 price > 100 且 tag != "test"
            && o.price > 100.0
            && o.tag != "test";
        });
        std::vector<Order> arr{filtered.begin(), filtered.end()};
        // step3: 按 price 排序(降序)
        ranges::sort(arr, std::greater<>{}, &Order::price);
        std::unordered_set<int> st;
        auto res = arr
        // step4: 去重 userId(保留第一次出现的)
            | views::filter([&](Order const& o) {
                return st.insert(o.userId).second;
            })
        // step5: 取前 10 个
            | views::take(10)
        // step6: 提取 userId
            | views::transform([](Order const& o) { return o.userId; });
        return res;
        // tip: 此次偷懒了, 因为 C++23 才有 ranges::to<std::vector<int>>, 所以我直接返回
    }
};
```

编写后, 我就深刻体会到, 实际上有时候我连 ranges 和 views 都分不清... 有的东西是不能直接使用管道符的.

这也激起我的兴趣. 所以我要手撕一个.

## 三、手撕 ranges / views
### 3.1 ranges

我们先从最基础的 FilterView 开始~

> [!TIP]
> 我们不手撕其迭代器, 因为它的概念太多了qwq...

看看它是如何被使用的:

```cpp
auto filter = std::ranges::filter_view{std::vector<int>{1,3,3,2,5}, [](auto) {
    return true;
}};

for (auto v : filter) {
    (void)v;   
}
```

也就是其是一个筛选器, 用于筛选出满足条件的元素. 传入的是一个 `容器Range` 和 `筛选函数`:

所以我们可以设计如下接口:

```cpp
// F = [](RangeType<R>) -> bool {}
template <Range R, Pred<RangeType<R>> P>
struct FilterView {
    FilterView() requires (std::default_initializable<R>
			      && std::default_initializable<P>)
	= default;

    constexpr FilterView(R range, P pred) 
        : _base{std::move(range)}
        , _pred{std::move(pred)}
    {}

    constexpr auto begin() {
        // @todo: cache 首次查找后可以缓存, 日后无需再查找
        return std::ranges::find_if(_base, _pred);
    }
    constexpr auto end() {
        return std::ranges::end(_base);
    }
private:
    R _base{};
    P _pred;
};
```

> 由于我们并不实现迭代器, 所以它实际上非常简单. 就是存储了 range, 以及 筛选函数. 在触发 `begin()` 时候, 才进行查找操作 (之后会缓存这个查找的结果(此处未实现))

其中, 一些细节需要约束:

1. `R` 必须是 range 概念
2. `P` 必须是函数对象, 并且传入是 `R` 元素类型参数, 并且返回值可以转换为 `bool`

故有:

```cpp
template <typename T>
concept Range = requires(T& t) {
    std::ranges::begin(t);
    std::ranges::end(t);
};

template <typename T>
using RangeType = decltype(*std::ranges::begin(std::declval<T&>())); // 注意必须是 左值引用

template <typename F, typename Arg>
concept Pred = requires(F&& func, Arg&& arg) {
    { std::forward<F>(func)(std::forward<Arg>(arg)) } -> std::convertible_to<bool>;
};
```

如果你观察了标准库, 实际上 `FilterView` 还需要实习一些接口. 比如: `size()`, `empty()`, `base()` 等等:

> [!TIP]
> 标准库是使用 CRTP 技术来实现这些接口的, 我们可以学习一下标准库是如何约束 CRTP 的.

```cpp [ranges02-ViewInterface]
template <typename Crtp>
    requires (std::is_object_v<Crtp> && std::same_as<Crtp, std::remove_cvref_t<Crtp>>)
struct ViewInterface {
    constexpr auto& base() noexcept {
        return derived()._base;
    }
    constexpr auto& base() const noexcept {
        return derived()._base;
    }
    constexpr auto data() noexcept {
        return std::to_address(std::ranges::begin(base()));
    }
    constexpr auto data() const noexcept {
        return std::to_address(std::ranges::begin(base()));
    }
    constexpr bool empty() const noexcept {
        return std::ranges::end(base()) - std::ranges::begin(base());
    }
    constexpr std::size_t size() const noexcept {
        return std::ranges::end(base()) - std::ranges::begin(base());
    }
private:
    constexpr Crtp& derived() noexcept {
        static_assert(std::derived_from<Crtp, ViewInterface>);
        return static_cast<Crtp&>(*this);
    }
    constexpr Crtp const& derived() const noexcept {
        static_assert(std::derived_from<Crtp, ViewInterface>);
        return static_cast<Crtp const&>(*this);
    }
};
```

```cpp [ranges02-完整代码]
namespace HX::ranges {

template <typename T>
concept Range = requires(T& t) {
    std::ranges::begin(t);
    std::ranges::end(t);
};

template <typename T>
using RangeType = decltype(*std::ranges::begin(std::declval<T&>())); // 注意必须是 左值引用

template <typename F, typename Arg>
concept Pred = requires(F&& func, Arg&& arg) {
    { std::forward<F>(func)(std::forward<Arg>(arg)) } -> std::convertible_to<bool>;
};

template <typename Crtp>
    requires (std::is_object_v<Crtp> && std::same_as<Crtp, std::remove_cvref_t<Crtp>>)
struct ViewInterface {
    constexpr auto& base() noexcept {
        return derived()._base;
    }
    constexpr auto& base() const noexcept {
        return derived()._base;
    }
    constexpr auto data() noexcept {
        return std::to_address(std::ranges::begin(base()));
    }
    constexpr auto data() const noexcept {
        return std::to_address(std::ranges::begin(base()));
    }
    constexpr bool empty() const noexcept {
        return std::ranges::end(base()) - std::ranges::begin(base());
    }
    constexpr std::size_t size() const noexcept {
        return std::ranges::end(base()) - std::ranges::begin(base());
    }
private:
    constexpr Crtp& derived() noexcept {
        static_assert(std::derived_from<Crtp, ViewInterface>);
        return static_cast<Crtp&>(*this);
    }
    constexpr Crtp const& derived() const noexcept {
        static_assert(std::derived_from<Crtp, ViewInterface>);
        return static_cast<Crtp const&>(*this);
    }
};

// F = [](RangeType<R>) -> bool {}
template <Range R, Pred<RangeType<R>> P>
struct FilterView : public ViewInterface<FilterView<R, P>> {
    FilterView() requires (std::default_initializable<R>
			      && std::default_initializable<P>)
	= default;

    constexpr FilterView(R range, P pred) 
        : _base{std::move(range)}
        , _pred{std::move(pred)}
    {}

    constexpr auto begin() {
        // @todo: cache 首次查找后可以缓存, 日后无需再查找
        return std::ranges::find_if(_base, _pred);
    }
    constexpr auto end() {
        return std::ranges::end(_base);
    }
private:
    R _base{};
    P _pred;
    friend ViewInterface<FilterView<R, P>>;
};

// 这里的推导指引非常重要!
template <Range R, Pred<RangeType<R>> P>
FilterView(R&&, P) -> FilterView<std::views::all_t<R>, P>;

} // namespace HX::ranges

int main() {
    std::vector<int> arr{1,3,3,2,5};
    auto all = std::views::all(arr);
    (void)all.base();
    auto filter = ranges::FilterView{arr, [](auto v) {
        log::hxLog.warning(v);
        return true;
    }};
    [[maybe_unused]] auto base = filter.base();
    for (auto v : filter) {
        log::hxLog.info(v);
    }
}
```

如果你直接编写完成. 那么实际上你的实现是错误的!, 你并不支持 `ranges::FilterView{std::vector<int>{}, [](auto v) { return true; }}` 这种传入 `右值` 的情况.

必须要使用推导指引:

```cpp
template <Range R, Pred<RangeType<R>> P>
FilterView(R&&, P) -> FilterView<std::views::all_t<R>, P>;
```

你可以简单理解成 `std::views::all_t<R>` 会把 **左值** 转换成 `ranges::ref_view{}`, 而 **右值** 则是 `ranges::owning_view{}` (确保其有所有权, 而不是 `dangling`)

### 3.2 views

这里更加nb! 明天再说!