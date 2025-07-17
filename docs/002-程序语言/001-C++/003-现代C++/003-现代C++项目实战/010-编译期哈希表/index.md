# 编译期哈希表

## 一、需求产生

最近在重构 基于反射的 Json 解析库, 在写反序列化部分.

发现, 需要一个哈希表, 因为我可以把结构体字段反射为 `index - name` 的 `std::array`, 但是在反序列化的时候, 需要从 `name` 快速映射到 `index`, 然后在通过一些操作进行赋值...

此处就需要一个哈希表. 然后正打算使用 `std::unordered_map` 直接开造.

但是还是很强迫的想到性能, 毕竟 STL 的哈希表是挺慢的...

我本身是打算直接 `std::unordered_map<std::size_t, std::size_t>` 的, 其 Key 的含义是 hashCode, Val 则是索引.

> [!TIP]
> 因为什么呢? 
> 
> 因为一边解析 json 的时候, 肯定需要解析完整个 `nameStr` 才可以得到 nameStr 的嘛, 然后即便是 `std::string_view` 做 HashKey 也一样要再次遍历一次字符串才可以得到哈希值的... 虽然也是 $O(Len)$ 但是常数就比较大了...

所以, 实际上我们可以直接把 **计算哈希的过程和遍历的过程合为一**, 这样只需要哈希一个整数, 其性能应该会有所提升吧?

> 但是你知道的, STL 的哈希表的底层实现大部分是采用 `链地址法+质数桶`, 这在某些情况下可能会出现`长链`.
>
> 并且链表这种东西是 CPU 缓存不友好的... 是否有更好的实现呢?

再者, 实际上我们的元素数量是已知的 (此处数量实际上就是反射的成员个数), 是否可以 把它们存储在 一个大小为 $N$ 的 `std::array` 里面呢?

也就是把 $N$ 个 `string` 映射到一个长度为 $N$ 的数组中!

## 二、完美哈希

> 查看了 ylt 的 json 解析, 发现他们就是使用了之前所说的哈希表, 编译期就直接决定了!

感觉很有意思就打算自己尝试实现一个. 查阅资料学习到...

### 2.1 生日悖论

我们可以从一个人人都听过的 `生日悖论` 问题出手:

问题:

> 随机选 $n$ **个人**, 生日分布在 **365天** (忽略闰年), 求至少两个人生日相同的概率。

推导:

- **所有人生日都不同的概率**: 

假设每个人生日独立, 均匀分布:

$
第一个人: 365/365 \\
第二个人: 364/365 \\
第三个人: 363/365 \\
... \\
第n个人: (365 - n + 1)/365
$

所以:

$$
P(\text{无冲突}) = \prod_{k=0}^{n-1} \frac{365 - k}{365}
$$

取反得到生日悖论概率:

$$
P(\text{有冲突}) = 1 - P(\text{无冲突})
$$

近似展开:

当 n 远小于 365, 使用对数近似:

$$
\ln P(\text{无冲突}) = \sum_{k=0}^{n-1} \ln\left(1 - \frac{k}{365}\right) \approx -\sum_{k=0}^{n-1} \frac{k}{365}
$$

这是泰勒展开, 保留一阶:

$$
\approx -\frac{1}{365} \cdot \sum_{k=0}^{n-1} k = -\frac{1}{365} \cdot \frac{n(n-1)}{2}
$$

取指数:

$$
P(\text{无冲突}) \approx e^{-\frac{n(n-1)}{2 \times 365}}
$$

因此:

$$
P(\text{有冲突}) \approx 1 - e^{-\frac{n(n-1)}{2 \times 365}}
$$

- n=23 时. 冲突概率就接近 **50%**
- n=57 时. 冲突概率接近 **99%**

这就是生日悖论的核心。

### 2.2 完美哈希问题

完美哈希的基本问题:

- 给定 **n个键**
- 映射到 **m个桶** (一般我们期望 `n == m`)
- 哈希函数是理想随机的

冲突概率和生日问题完全对应

> [!TIP]
> 生日问题是 $n$ 个人分配到 $m = 365$ 天中, 不冲突的概率 **的对立面** (即至少有两个人生日同一天).
>
> 而完全哈希则是这个对立面的对立面, 表示 这 $n$ 个键不冲突的概率.

无冲突概率:

$$
P(\text{无冲突}) = \prod_{k=0}^{n-1} \left(1 - \frac{k}{m}\right)
$$

近似为:

$$
P(\text{无冲突}) \approx e^{-\frac{n(n-1)}{2m}}
$$


完美哈希成功概率

> 若期望无冲突, 即成功构造静态完美哈希, 需要确保:

$$
P(\text{无冲突}) \approx e^{-\frac{n(n-1)}{2m}} \approx 1\\
\Rightarrow e^{-\frac{n(n-1)}{2m}} \approx e^0\\
\Rightarrow -\frac{n(n-1)}{2m} \approx 0\\
\Rightarrow \frac{n(n-1)}{2m} \approx 0
$$

即, 要求:

$$
\lim_{m \to \infty} \frac{n(n-1)}{2m} = 0
$$

因为上面的比较难求, 我们可以对立的求

$$
\frac{n(n-1)}{2m} \ll 1
$$

则有:

$$
\Rightarrow m \gg \frac{n(n-1)}{2}
$$

**常见经验值:**

- 如果想让完美哈希一次成功, $m$ 需要大概:

$$
m \approx n^2
$$

这是最保守的。

实际算法 (如 CHD, BDZ) 通常通过二级哈希法减少空间到:

$$
m \approx 1.23n \sim 2n
$$

但那是多步法, 不是一次随机哈希。$^{[By \ GPT]}$

### 2.3 工程化完美哈希

通过上面的计算我们知道, 在随机选择一个通用哈希函数的情况下, 将 $n$ 个元素 映射到 $m$ 个桶中, 不冲突的概率为:

$$
P(\text{无冲突}) = \prod_{k=0}^{n-1} \left(1 - \frac{k}{m}\right) \approx e^{-\frac{n(n-1)}{2m}}
$$

我们由于我们的目的就是把 $n$ 个元素映射到 $n$ 个桶中. (即 $m = n$)

那么:

$$
P(\text{有冲突}) = 1 - e^{-\frac{n(n-1)}{2m}}
$$

此时为了提高构造成功率, 我们可以再次选择一个通用的哈希函数, 那么选择了 $k$ 个哈希函数后, 依然有冲突的概率为:

$$
P(\text{有冲突}) = (1 - e^{-\frac{n(n-1)}{2m}})^k
$$

根据高中知识有: 

$f(x)=a^x$, 当 $a \in (0, 1)$ 时候, 单调且指数级别的 $f(x) \to 0$ 变化.

体感上, 只要尝试的够多, $m$ 本身也够大, 总能做到完美哈希.

> 故这种计算在编译期也是可以做到的!

> [!NOTE]
> 您还可以学习: 
> - https://github.com/serge-sans-paille/frozen
> - https://stevehanov.ca/blog/index.php?id=119

## 三、代码实现

我们先看看需要实现什么吧~

通过上面的推导, 我们需要实现一个在编译期的时候就可以使用的随机通用哈希函数.

- 这个可以通过哈希的时候指定一个随机的种子就可以了~

那也就是需要一个 **编译期随机数生成器**

### 3.1 编译期随机数生成器

```cpp [c1-linear_congruential_engine]
namespace frozen { // frozen 开源库的实现

// https://github.com/serge-sans-paille/frozen

template <class UIntType, UIntType a, UIntType c, UIntType m>
class linear_congruential_engine {
    static_assert(std::is_unsigned<UIntType>::value,
                  "UIntType must be an unsigned integral type");

    template <class T, UIntType M>
    static constexpr UIntType modulo(T val,
                                     std::integral_constant<UIntType, M>) {
        // the static cast below may end up doing a truncation
        return static_cast<UIntType>(val % M);
    }

public:
    using result_type = UIntType;
    static constexpr result_type multiplier = a;
    static constexpr result_type increment = c;
    static constexpr result_type modulus = m;
    static constexpr result_type default_seed = 1u;

    linear_congruential_engine() = default;
    constexpr linear_congruential_engine(result_type s) {
        seed(s);
    }

    void seed(result_type s = default_seed) {
        state_ = s;
    }
    constexpr result_type operator()() {
        using uint_least_t =
            bits::select_uint_least_t<bits::log(a) + bits::log(m) + 4>;
        uint_least_t tmp =
            static_cast<uint_least_t>(multiplier) * state_ + increment;

        state_ = modulo(tmp, std::integral_constant<UIntType, modulus>());
        return state_;
    }

private:
    result_type state_ = default_seed;
};


using minstd_rand =
    linear_congruential_engine<std::uint_fast32_t, 48271, 0, 2147483647>;

using default_prg_t = minstd_rand;

} // namespace frozen
```

```cpp [c1-XorShift32]
// 想到了小彭老师教的伪随机数生成器, 加上 constexpr 就是编译期可以使用的了
struct XorShift32 {
    uint32_t a;

    constexpr XorShift32(size_t seed = 0) 
        : a(static_cast<uint32_t>(seed + 1)) 
    {}

    using result_type = uint32_t;

    constexpr uint32_t operator()() noexcept {
        uint32_t x = a;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        return a = x;
    }
};

using default_prg_t = XorShift32;
```

### 3.2 编译期哈希表

> 摊牌了, 后面我也只会研究 frozen 的源码了, 因为这应该都可以搞了论文了吧qwq... 我不可能从0到1发明出来的...

好在他们的源码还是挺清晰的, 注释也到位, 纯头文件, 代码量也不多, 我把本次项目需要的东西, 全部展开到单个文件, 方便我看. 也就 1000 来行的纯模板元编程... [(具体Cpp文件)](https://github.com/HengXin666/HXTest/blob/main/src/06-std-analyse/demo/05-pmh/02_cp_pmh_test.cpp), ~~花了半个早上和小半个下午, 配合昨天下午查的资料的基础, 搞明白了 =-= (核心实际上上面的数学推导(自己推的), 剩下的就只管相信数学就好)~~

我们由小往上的看:

我们也就需要实现一个这样的东西:

```cpp
template <typename Key, typename Val, std::size_t N, 
          typename Hash = elsa<Key>, 
          typename KeyEqual = std::equal_to<Key>>
class StaticHashMap {
    inline static constexpr std::size_t StorageSize 
        = nextHighestPowerOfTwo(N);

    using container_type = std::array<std::pair<Key, Val>, N>;
    using tables_type = PmhTables<StorageSize, Hash>;

    KeyEqual const _equal;
    container_type _items;
    tables_type _tables;
public:
    using key_type = Key;
    using mapped_type = Val;
    using value_type = typename container_type::value_type;

    constexpr StaticHashMap(container_type items, Hash const& hash,
                            KeyEqual const& equal) 
        : _equal{equal}
        , _items{items}
        , _tables{makePmhTables<StorageSize>(
            _items, hash, GetKey{}, default_prg_t{114514})} 
    {}

    constexpr StaticHashMap(container_type items) 
        : StaticHashMap{items, Hash{}, KeyEqual{}} 
    {}

    constexpr Val const& at(Key const& key) const {
        auto& kv = lookup(key);
        if (_equal(kv.first, key))
            return kv.second;
        else
            throw std::out_of_range("unknown key");
    }

    constexpr auto const& lookup(Key const& key) const noexcept {
        return _items[_tables.lookup(key)];
    }
};

// test
int main() {
    using namespace std::string_view_literals;
    constexpr StaticHashMap<std::string_view, std::size_t, 4> mp{
        std::array<std::pair<std::string_view, std::size_t>, 4>{
            std::pair<std::string_view, std::size_t>{"1", 1},
            {"2", 1},
            {"3", 1},
            {"4", 1},
        }
    };

    static_assert(mp.at("1") == 1, "111"); // 编译期
}
```

其中:

- `std::equal_to<Key>`是对比 Key 的 `==` 运算符重载的功能

    > 为什么需要这个我后面说~, 也是很重要的!

- `GetKey{}`是获取 `std::pair` 所谓 Key 的元素的模板实例

- `default_prg_t{114514}` 是前文提到的随机数生成器

- `nextHighestPowerOfTwo(n)` 是获取 >= n 的最小为 2的幂的数

```cpp
auto constexpr nextHighestPowerOfTwo(std::size_t v) {
    // https://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
    v--;
    for (std::size_t i = 1; i < sizeof(std::size_t) * 8; i <<= 1)
        v |= v >> i;
    v++;
    return v;
}
```

- `typename Hash = elsa<Key>` 的 `elsa` 则是他们库的核心出装, 不同于标准库的 `std::hash`, 他们是有一个重载是支持输入 `种子` 的, 这也是切换通用哈希函数的核心实现!

```cpp [c2-elsa主模板]
template <class T = void>
struct elsa {
    static_assert(std::is_integral<T>::value || std::is_enum<T>::value,
                  "only supports integral types, specialize for other types");

    constexpr std::size_t operator()(T const& value, std::size_t seed) const {
        std::size_t key = seed ^ static_cast<std::size_t>(value);
        key = (~key) + (key << 21); // key = (key << 21) - key - 1;
        key = key ^ (key >> 24);
        key = (key + (key << 3)) + (key << 8); // key * 265
        key = key ^ (key >> 14);
        key = (key + (key << 2)) + (key << 4); // key * 21
        key = key ^ (key >> 28);
        key = key + (key << 31);
        return key;
    }
};

// 工具, 方便 elsa<>(T{}, ...) 自动转发到 elsa<T>(...)
template <>
struct elsa<void> {
    template <class T>
    constexpr std::size_t operator()(T const& value, std::size_t seed) const {
        return elsa<T>{}(value, seed);
    }
};
```

```cpp [c2-elsa字符串偏特化]
template <typename String>
constexpr std::size_t hashString(const String& value) {
    std::size_t d = 5381;
    for (const auto& c : value) 
        d = d * 33 + static_cast<size_t>(c);
    return d;
}

// https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
// 根据实验设置删除最低位。
template <typename String>
constexpr std::size_t hashString(const String& value, std::size_t seed) {
    std::size_t d = (0x811c9dc5 ^ seed) * static_cast<size_t>(0x01000193);
    for (const auto& c : value)
        d = (d ^ static_cast<size_t>(c)) * static_cast<size_t>(0x01000193);
    return d >> 8;
}

template <typename _CharT>
struct elsa<std::basic_string_view<_CharT>> {
    constexpr std::size_t operator()(std::basic_string_view<_CharT> const& value) const {
        return hashString(value);
    }
    constexpr std::size_t operator()(std::basic_string_view<_CharT> const& value,
                                     std::size_t seed) const {
        return hashString(value, seed);
    }
};
```

而最最核心的是 `using tables_type = PmhTables<StorageSize, Hash>;`

### 3.3 pmh 算法创建的完美哈希函数

我们先看一下它的类吧:

```cpp
// 表示 pmh 算法创建的完美哈希函数
template <std::size_t M, class Hasher>
struct PmhTables {
    uint64_t _first_seed; // 用于映射一个元素时候的种子
    std::array<SeedOrIndex, M> _first_table;  // 记录种子或者索引
    std::array<std::size_t, M> _second_table; // 第二层哈希表
    Hasher _hash; // elsa<> 对象

    template <typename KeyType>
    constexpr std::size_t lookup(const KeyType& key) const {
        return lookup(key, _hash);
    }

    // 查找给定的键, 以在 carray<Item， N 中找到其预期索引>
    // 总是返回有效的索引, 之后必须使用 KeyEqual 测试来确认。
    template <typename KeyType, typename HasherType>
    constexpr std::size_t lookup(const KeyType& key,
                                 const HasherType& hasher) const {
        auto const d =
            _first_table[hasher(key, static_cast<size_t>(_first_seed)) % M];
        // 如果不是种子就直接返回索引
        if (!d.isSeed()) {
            return static_cast<std::size_t>(d.value());
        } // 这是缩小 uint64 -> size_t 但应该没问题
        else {
            // 如果是种子, 就作为种子查第二个哈希表, 得到索引
            return _second_table[
                hasher(key, 
                       static_cast<std::size_t>(d.value())
                ) % M];
        }
    }
};
```

你只需要知道这几个成员就好了:

```cpp
uint64_t _first_seed; // 用于映射一个元素时候的种子
std::array<SeedOrIndex, M> _first_table;  // 记录种子或者索引
std::array<std::size_t, M> _second_table; // 第二层哈希表
```

### 3.4 创建 pmh 表

> [!TIP]
> 这里是算法的核心!

库通过工厂函数, 编译期创建并初始化 pmh 表 (代码我在原本英文注释的基础上添加了我手写的中文注释):

> 不用着急看完, 先看 `makePmhBuckets` 函数的实现! 然后看 `getSortedBuckets` 的实现, 特别是`桶结构`.

> [!NOTE]
> 理清楚桶结构和 G、H 表, 那你就已经懂了~

```cpp [c4-工厂函数]
// 为给定的项目、哈希函数、prg 等制作 pmh 表。
template <std::size_t M, class Item, std::size_t N, class Hash, class Key,
          class PRG>
PmhTables<M, Hash> constexpr makePmhTables(const std::array<Item, N>& items,
                                           Hash const& hash, Key const& key,
                                           PRG prg) {
    // 第 1 步: 将所有密钥放入存储桶中 (得到一个哈希种子, 可以把元素都映射到桶中)
    // 桶
    auto stepOne = makePmhBuckets<M>(items, hash, key, prg);

    // 第 2 步: 对存储桶进行排序，以首先处理项目最多的存储桶。
    auto buckets = stepOne.getSortedBuckets();

    // G 成为生成的 pmh 函数中的第一个哈希表
    std::array<SeedOrIndex, M> G; // 默认构造为 “index 0”
                                  // 索引 或者 哈希种子

    // H 成为生成的 pmh 函数中的第二个哈希表
    constexpr std::size_t UNUSED = (std::numeric_limits<std::size_t>::max)();
    std::array<std::size_t, M> H; // 纯索引
    H.fill(UNUSED);

    // 第 3 步: 将存储桶中的项目映射到哈希表中。
    for (const auto& bucket : buckets) { // 获取每个桶
        auto const bsize = bucket.size();

        // 桶大小为 1
        if (bsize == 1) {
            // 在 G 中存储 （单个） 项的索引
            G[bucket.index] = {false, static_cast<uint64_t>(bucket[0])}; // 是索引
        } else if (bsize > 1) {
            // 桶大小不为 1, 那需要保证桶元素全部可以映射到 二级哈希表 H 中的 空的 独立的位置 (也就是不冲突)
            // 反复尝试不同的 H 或 d, 直到找到一个哈希函数将桶中的所有物品放入空闲槽中
            SeedOrIndex d{true, prg()}; // d 是种子

            // 记录桶元素 索引到的 二级哈希表 H 中的位置的索引
            // 因为会匹配失败, 所以需要反悔, 因此我们可以分开记录, 这样就只需要 bucketSlots.clear() 即可
            cvector<std::size_t, decltype(stepOne)::bucket_max> bucketSlots;

            while (bucketSlots.size() < bsize) {
                // 计算桶中, 第 idx = bucketSlots.size() 个元素索引到的 items 的 key 的哈希值
                auto slot = hash(
                    key(
                        items[bucket[bucketSlots.size()]]
                    ),
                        static_cast<size_t>(d.value())
                    ) % M;

                // 如果这个哈希值映射到 [0, M) 的 二级哈希表 H 中, 
                // 如果这个位置是空的 或者 没有被在本次记录中之前的桶元素索引到
                if (H[slot] != UNUSED || !allDifferentFrom(bucketSlots, slot)) {
                    // 如果不满足, 就清空记录, 然后换哈希函数 (也就是换哈希种子)
                    bucketSlots.clear();
                    d = {true, prg()};
                    continue;
                }
                bucketSlots.push_back(slot);
            }

            // 将成功的种子放入 G 中, 并将索引放入其槽中的项目
            G[bucket.index] = d; // 给一级哈希表记录 种子
            // 二级哈希表记录上本次记录
            for (std::size_t i = 0; i < bsize; ++i)
                H[bucketSlots[i]] = bucket[i];
        }
    }

    // H 表中任何未使用的条目都必须更改为零。
    // 这是因为哈希处理不应失败或返回越界条目。
    // 将用户提供的 KeyEqual 应用于查询并且
    // 通过哈希找到的 key。将此类查询发送到 0 不会有什么坏处。
    for (std::size_t i = 0; i < M; ++i)
        if (H[i] == UNUSED)
            H[i] = 0;

    return {stepOne.seed, G, H, hash};
}
```

```cpp [c4-allDifferentFrom]
// 检查项目是否出现在 cvector 中
template <class T, size_t N>
constexpr bool allDifferentFrom(cvector<T, N>& data, T& a) {
    for (std::size_t i = 0; i < data.size(); ++i)
        if (data[i] == a)
            return false;
    return true;
}
```

```cpp [c4-SeedOrIndex]
// 一级哈希表的元素类型 (items的索引 或者 哈希种子 的共用体)
// 表示数据项数组的索引或要使用的种子一个哈希器。
// 种子必须具有 1 的高位, 值必须具有 0 的高位。
struct SeedOrIndex {
    using value_type = uint64_t;

private:
    inline static constexpr value_type MINUS_ONE 
        = (std::numeric_limits<value_type>::max)();

    inline static constexpr value_type HIGH_BIT 
        = ~(MINUS_ONE >> 1);

    value_type _value = 0;

public:
    constexpr value_type value() const {
        return _value;
    }
    constexpr bool isSeed() const {
        return _value & HIGH_BIT;
    }

    constexpr SeedOrIndex(bool is_seed, value_type value) 
        : _value{is_seed ? (value | HIGH_BIT) 
                         : (value & ~HIGH_BIT)} 
    {}

    constexpr SeedOrIndex() = default;
};
```

### 3.5 创建桶

```cpp
// 目的: 找到一个哈希种子把 key 映射到 桶中, 并且即便冲突也不会大于桶的大小
// 返回是 桶, 其中 包含了哈希种子 和 桶数组, 数组中记录的是 items Key 的索引 (具体请看桶结构)
template <size_t M, class Item, size_t N, class Hash, class Key, class PRG>
PmhBuckets<M> constexpr makePmhBuckets(const std::array<Item, N>& items,
                                       Hash const& hash, Key const& key,
                                       PRG& prg) {
    using result_t = PmhBuckets<M>;
    result_t result{};
    bool rejected = false;
    // 继续作，直到放置完所有项目，且不超过 bucket_max
    while (1) {
        for (auto& b : result.buckets) {
            b.clear();
        }
        result.seed = prg(); // 生成一个种子
        rejected = false;
        for (std::size_t i = 0; i < N; ++i) {
            // 元素映射到桶
            auto& bucket = result.buckets[hash(key(items[i]), static_cast<size_t>(result.seed)) % M];
            if (bucket.size() >= result_t::bucket_max) {
                rejected = true;
                break;
            }
            bucket.push_back(i); // bucket 记录桶中映射的元素索引 (items 的索引)
        }
        if (!rejected) {
            return result;
        }
    }
}
```

### 3.6 桶结构 & 排序桶

```cpp [c3-桶结构]
template <size_t M>
struct PmhBuckets {
    // 第 0 步: 存储桶最大值为 2 * sqrt M
    // TODO:  想出这个理由，不应该是 O（log M） 吗？
    static constexpr auto bucket_max = 2 * (1u << (_log(M) / 2)); // 这里是数学, 不用管

    using bucket_t = cvector<std::size_t, bucket_max>;
    std::array<bucket_t, M> buckets; // 桶 [i][j] (是二维数组)
                                     // buckets[i] 表示第 i + 1 个桶
                                     // buckets[i][j] 表示第 i + 1 个桶的第 j + 1 个元素,
                                     // 其映射到 items 索引为 buckets[i][j] 的元素
    uint64_t seed; // 计算桶索引使用的哈希种子
public:
    // 表示对存储桶的引用。之所以使用此方法，是因为存储桶
    // 必须进行排序，但存储桶很大(元素是cvector, 显然不能这样(编译期一般是拷贝或者交换))，因此比排序引用慢
    struct BucketRef {
        unsigned index;             // 原本元素的索引
        const bucket_t* ptr;

        // 转发 bucket 的一些接口
        using value_type = typename bucket_t::value_type;
        using const_iterator = typename bucket_t::const_iterator;

        constexpr auto size() const {
            return ptr->size();
        }
        constexpr const auto& operator[](std::size_t idx) const {
            return (*ptr)[idx];
        }
    };

    // 为每个存储桶创建一个 bucket_ref
    template <std::size_t... Is>
    std::array<BucketRef, M> constexpr makeBucketRefs(
        std::index_sequence<Is...>
    ) const {
        return {{BucketRef{Is, &buckets[Is]}...}};
    }

    // 为每个存储桶创建一个bucket_ref并按大小对它们进行排序
    std::array<BucketRef, M> constexpr getSortedBuckets() const {
        std::array<BucketRef, M> result{
            makeBucketRefs(std::make_index_sequence<M>())
        };
        // 编译期排序, 按照 桶大小 从大到小 排序
        std::sort(result.begin(), result.end(), [](BucketRef const& b1, BucketRef const& b2) {
            return b1.size() > b2.size();
        });
        return result;
    }
};
```

```cpp [c3-_log]
template <class T>
auto constexpr _log(T v) { // 获取 v 的二进制长度
    std::size_t n = 0;
    while (v > 1) {
        n += 1;
        v >>= 1;
    }
    return n;
}
```

### 3.7 编译期 vector

我们需要的实际上只是编译期的 `push_back`, 它的元素我们可以直接分配到 `T[]` 中; 然后通过 `指针指向` 来保证那些元素是有效、以及计算vector有效长度.

```cpp
template <class T, std::size_t N>
class cvector {
    T _data[N] = {}; // 标量类型 T 的零初始化,
                     // 否则为 default-initialized
    std::size_t _dsize = 0;

public:
    // Container typdefs
    using value_type = T;
    using reference = value_type&;
    using const_reference = const value_type&;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using iterator = pointer;
    using const_iterator = const_pointer;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    // Constructors
    constexpr cvector(void) = default;
    constexpr cvector(size_type count, const T& value) : _dsize(count) {
        for (std::size_t i = 0; i < N; ++i)
            _data[i] = value;
    }

    // Iterators
    constexpr iterator begin() noexcept {
        return _data;
    }
    constexpr iterator end() noexcept {
        return _data + _dsize;
    }

    // Capacity
    constexpr size_type size() const {
        return _dsize;
    }

    // Element access
    constexpr reference operator[](std::size_t index) {
        return _data[index];
    }
    constexpr const_reference operator[](std::size_t index) const {
        return _data[index];
    }

    constexpr reference back() {
        return _data[_dsize - 1];
    }
    constexpr const_reference back() const {
        return _data[_dsize - 1];
    }

    // Modifiers
    constexpr void push_back(const T& a) {
        _data[_dsize++] = a;
    }
    constexpr void push_back(T&& a) {
        _data[_dsize++] = std::move(a);
    }
    constexpr void pop_back() {
        --_dsize;
    }

    constexpr void clear() {
        _dsize = 0;
    }
};
```

### 3.8 为什么需要 `KeyEqual`

你可能注意到, `StaticHashMap` 有一个 `KeyEqual` 模板参数. 为什么需要比较Key呢?

原来, 完美哈希只是保证了对于这 $n$ 个已知元素不发生冲突, 但是外界的输入是会发生冲突的.

此时我们得到的 index 也是合法的 (因为 % M)

那我们怎么区分这个到底是不是因为冲突的 key 映射, 导致映射到一个错误但是有效的元素呢?

这很简单, 我们判断一下 key 是否为 index 对应元素 的 key 就好了! 所以需要 `==`

### 3.9 总结

完整实验代码: https://github.com/HengXin666/HXTest/blob/main/src/06-std-analyse/demo/05-pmh/04_hx_pmh_map.cpp

此处的完美哈希, 是把 $n$ 个元素 映射到 $m = n$ 个桶中, 桶的最大容量是 $BZ = 2 \times \sqrt{m}$

然后进行分配:

1. 找到一个哈希种子, 可以把 `items` 的所有 `key` 都映射到桶中, 并且桶的容量不超过 $BZ$.

2. 排序桶, 按照桶元素的数量从大到小排序, 到时候优先处理大桶

3. 创建 一级哈希表 $G$ (索引或者哈希种子) 和 二级哈希表 $D$, 其映射关系是:

```cpp
constexpr auto at(Key const& key) {
    if (G 是 索引)
        return G.val()
    else // 是 哈希种子
        return D[hash(key, G.val()) % M];
}
```

4. 从大到小排序, 处理桶

    4.1 获取到桶 ( `for 桶 in 排序引用桶` )

    -- 小优化: 如果 `桶.size() == 1`, 那么 `G[桶.index]` 是索引 `G.val = 桶[0]`

    4.2 随机生成一个哈希种子

    4.3 获取到桶的每一个元素 ( `for (i = 0; i < 桶.size(); ++i)` )

    4.3 尝试把元素 (`items[桶[i]]`) 即 `items` 对应的 `key`  映射到 二级表 $D$ 中

    4.4 如果 $D$ 位置为空, 则 没问题; 如果 **不是**, 则需要重新生成一个哈希种子 (goto 4.3)

    4.5 记录 `G[桶.index] = 哈希种子`, 并且确定 二级表 $D$ 的映射.

5. 返回 `1. 找到的哈希种子` 和 $G$, $D$ 作为 pmh表.

最终的查找逻辑:

- 通过 `1. 找到的哈希种子` 进行 hash, 得到 $G$ 的位置, 根据 `3. 映射关系` 进行映射.

## 附、比较探测的编译期哈希表

> 测试过 1024 个元素, 也就需要编译 6 秒罢了...
>
> 这个是之前学习的时候 gpt 给的

```cpp
#include <array>
#include <cassert>
#include <string_view>
#include <cstdint>
#include <iostream>

// 简单 FNV-1a 哈希, 可添加 seed
constexpr uint64_t fnv1a_seed(std::string_view str, uint64_t seed = 0) {
    constexpr uint64_t FNV_OFFSET = 1469598103934665603ULL;
    constexpr uint64_t FNV_PRIME = 1099511628211ULL;
    uint64_t h = FNV_OFFSET ^ seed;
    for (char c : str) {
        h ^= static_cast<uint8_t>(c);
        h *= FNV_PRIME;
    }
    return h;
}

template <typename String>
constexpr std::size_t hash_string(const String& value) {
  std::size_t d = 5381;
  for (const auto& c : value) d = d * 33 + static_cast<size_t>(c);
  return d;
}

// https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
// With the lowest bits removed, based on experimental setup.
template <typename String>
constexpr std::size_t hash_string(const String& value, std::size_t seed) {
  std::size_t d = (0x811c9dc5 ^ seed) * static_cast<size_t>(0x01000193);
  for (const auto& c : value)
    d = (d ^ static_cast<size_t>(c)) * static_cast<size_t>(0x01000193);
  return d >> 8;
}

constexpr uint64_t mix(uint64_t x) {
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33;
    return x;
}

// 通用哈希函数：整型原样返回，字符串使用 FNV-1a
template <typename Key>
constexpr uint64_t hash_func(const Key& key, uint64_t seed = 0) {
    if constexpr (std::is_integral_v<Key>) {
        uint64_t x = static_cast<uint64_t>(key);
        return mix(x + seed * 0x9e3779b97f4a7c15ULL);
    } else if constexpr (std::is_same_v<Key, std::string_view>) {
        return hash_string(key, seed);
    } else {
        static_assert(!sizeof(Key*), "Unsupported key type");
    }
}

template <typename Key, typename Value, size_t N, size_t M>
struct StaticPerfectHash {
    std::array<std::pair<Key, Value>, M> table{};
    std::array<bool, M> occupied{};

    constexpr size_t probe(const Key& key, size_t i) const {
        return (hash_string(key) + i) % M;
    }

    constexpr Value get(const Key& key) const {
        for (size_t i = 0; i < M; ++i) {
            size_t idx = probe(key, i);
            if (!occupied[idx])
                break; // 不存在
            if (table[idx].first == key)
                return table[idx].second;
        }
        return Value{-1};
    }
};

// M 会影响编译速度, 实际测试, m 为 1.1 就 ok, 保险是 1.23, 快速编译就选 1.5, 最高设置为 2, 大于 2 的几乎没有收益
template <typename Key, typename Value, size_t N,
          size_t M = static_cast<std::size_t>(N * 1.5)>
consteval StaticPerfectHash<Key, Value, N, M>
make_perfect_hash(const std::array<std::pair<Key, Value>, N>& data) {
    StaticPerfectHash<Key, Value, N, M> ph{};

    for (auto [key, val] : data) {
        size_t dist = 0;
        size_t idx = hash_string(key) % M;

        while (true) {
            if (!ph.occupied[idx]) {
                ph.table[idx] = {key, val};
                ph.occupied[idx] = true;
                break;
            }
            // Robin Hood: 比较探测距离，交换
            size_t existing_idx = fnv1a_seed(ph.table[idx].first) % M;
            size_t existing_dist = (idx + M - existing_idx) % M;

            if (dist > existing_dist) {
                // 交换
                auto tmp = ph.table[idx];
                ph.table[idx] = {key, val};
                key = tmp.first;
                val = tmp.second;
                dist = existing_dist;
            }
            idx = (idx + 1) % M;
            ++dist;
            if (dist >= M) [[unlikely]]
                throw "Hash table is full, increase k!";
        }
    }
    return ph;
}

constexpr std::array<std::pair<std::string_view, int>, 6> testData6() {
    return {{
        {"key000", 0},
        {"key001", 1},
        {"key002", 2},
        {"key003", 3},
        {"key004", 4},
        {"key005", 5},
    }};
}

static constexpr auto testData = testData6();
static constexpr auto testTable = make_perfect_hash(testData);

constexpr bool testCompileTime() {
    for (size_t i = 0; i < testData.size(); i++) {
        if (testTable.get(testData[i].first) != testData[i].second)
            return false;
    }
    return true;
}

static_assert(testCompileTime(), "6 keys 编译期完美哈希测试失败");

int main() {
    for (auto [k, v] : testData) {
        int val = testTable.get(k);
        assert(val == v);
        std::cout << "测试通过: " << k << " -> " << val << "\n";
    }
    assert(testTable.get("asdasdasd") == -1);
    std::cout << "6 keys 所有测试通过。\n";
    return 0;
}
```