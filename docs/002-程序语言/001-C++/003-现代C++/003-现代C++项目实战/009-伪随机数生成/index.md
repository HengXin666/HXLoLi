# 伪随机数生成

- [伪随机数生成](https://zh.cppreference.com/w/cpp/numeric/random)

## 一、使用模板

```cpp
// 1. 定义随机数引擎生成器 (构造传参是随机数种子, 此处传入基于cpu的真随机数(开销大~))
std::mt19937 rng{std::random_device{}()};

// 2. 均匀分布: 产生在范围内均匀分布的整数
std::uniform_int_distribution<int> uni(1, 10);
// 如果需要浮点数, 请使用`uniform_real_distribution<T>`

for (int i = 0; i < 10; ++i)
    HX::print::print(uni(rng), ' ');
HX::print::print("\n");
```

`uniform_int_distribution`会控制其为均匀随机, 例如如果一个`f()`的取值为 $[0, 128]$, 我们取值是 $[0, 100]$, 那么如果是直接取模`100`, 得到的结果实际上`0 ~ 28`的概率是高一些的.

而`uniform_int_distribution`内部会保证概率均匀 (内部可能是把 $> 100$ 的数全部`continue`了, 直到生成在公平的均匀的 $[0, 100]$ 上)

## 二、XorShift32

- 这个是游戏常用的随机算法, 因为它性能高...

```cpp
struct XorShift32 {
    uint32_t a;

    explicit XorShift32(size_t seed = 0) 
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

    static constexpr uint32_t min() noexcept {
        return 1;
    }

    static constexpr uint32_t max() noexcept {
        return UINT32_MAX;
    }
};

// 使用
XorShift32 rng{std::random_device{}()};
std::uniform_int_distribution<int> uni(1, 10);

for (int i = 0; i < 10; ++i)
    HX::print::print(uni(rng), ' ');
HX::print::print("\n");
```

## 三、WangSHash

无状态依赖, 常用于并行随机

```cpp
struct WangSHash {
    uint32_t a;

    explicit WangSHash(size_t seed = 0) 
        : a(static_cast<uint32_t>(seed)) 
    {}

    using result_type = uint32_t;

    constexpr uint32_t operator()() noexcept {
        uint32_t x = a;
        x = (x ^ 61) ^ (x >> 16);
        x *= 9;
        x = x ^ (x >> 4);
        x *= 0x27d4eb2d;
        x = x ^ (x >> 15);
        return a = x;
    }

    static constexpr uint32_t min() noexcept {
        return 0;
    }

    static constexpr uint32_t max() noexcept {
        return UINT32_MAX;
    }
};
```

如:
```C++
for (std::size_t i = 0; i < 10; ++i) {
    WangSHash rng{i};
    std::uniform_int_distribution<int> uni(1, 100);
    HX::print::print(uni(rng), ' ');
}
```

而`XorShift32`, 就需要挪到`for`外面, 不然很容易出现一样的值.

```C++
HX::print::println("XorShift32:");
for (std::size_t i = 0; i < 10; ++i) {
    XorShift32 rng{i};
    std::uniform_int_distribution<int> uni(1, 100);
    HX::print::print(uni(rng), ' ');
}

HX::print::println("\nWangSHash:");
for (std::size_t i = 0; i < 10; ++i) {
    WangSHash rng{i};
    std::uniform_int_distribution<int> uni(1, 100);
    HX::print::print(uni(rng), ' ');
}
HX::print::print("\n");
```

输出:
```C++
XorShift32:
1 1 1 1 1 1 1 1 1 1 
WangSHash:
76 16 78 54 80 78 45 20 71 11 
```

> [!TIP]
> 不是`XorShift32`不能在`for`里面, 而是它容易出现一样的值 (你把`uni(1, 100)` -> `uni(1, 10000000)`) 就可以看出来`XorShift32`也是有在随机的...

## 四、洗牌

- [std::shuffle](https://zh.cppreference.com/w/cpp/algorithm/random_shuffle)

是`均匀分布+std::swap`.

```C++
#include <algorithm>
#include <iostream>
#include <iterator>
#include <random>
#include <vector>
 
int main() {
    std::vector<int> v{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
 
    std::mt19937 g(std::random_device{}());
 
    std::shuffle(v.begin(), v.end(), g);
 
    std::copy(v.begin(), v.end(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << '\n';
}
```

## 五、加权概率

例如某些游戏中的抽卡机制, 5%概率获得xxx, 是怎么实现的呢?

```cpp
// 定义概率
std::vector<float> pArr = {0.6f, 0.25f, 0.05f, 0.1f};
// 定义概率对应物品
std::vector<std::string> res = {"一星", "二星", "四星", "三星"};
std::vector<float> pArrScanned;
// 求`pArr`的元素和, 即 sum += pArr[i]; pArrScanned[i] = sum;
std::inclusive_scan(pArr.begin(), pArr.end(), std::back_inserter(pArrScanned));
HX::print::println("pArr: ", pArr);
HX::print::println("pArrScanned: ", pArrScanned);
std::mt19937 rng{std::random_device{}()};
std::uniform_real_distribution<float> unf(0.0f, 1.0f);
auto getRes = [&] () -> std::string {
    float f = unf(rng);
    // 二分, 确定概率区间
    auto it = std::lower_bound(pArrScanned.begin(), pArrScanned.end(), f);
    if (it == pArrScanned.end()) [[unlikely]] {
        return "";
    }
    return res[it - pArrScanned.begin()];
};
HX::print::println(getRes());
```