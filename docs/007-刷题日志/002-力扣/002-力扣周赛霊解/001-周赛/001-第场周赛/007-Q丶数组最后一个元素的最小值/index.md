# 3133. 数组最后一个元素的最小值
链接: [3133. 数组最后一个元素的最小值](https://leetcode.cn/problems/minimum-array-end/)

给你两个整数 n 和 x 。你需要构造一个长度为 n 的 正整数 数组 nums ，对于所有 0 <= i < n - 1 ，满足 nums[i + 1] 大于 nums[i] ，并且数组 nums 中所有元素的按位 AND 运算结果为 x 。

返回 nums[n - 1] 可能的 最小 值。

# 题解
从集合的视角看，$x$ 是每个 $nums[i]$ 的子集。换句话说，$nums[i]$ 一定是 $x$ 的超集。

例如 $x=100100$，那么 $nums[i]$ 一定在如下序列中:

$$1\underline{00}1\underline{00},1\underline{00}1\underline{01},1\underline{00}1\underline{10},1\underline{00}1\underline{11},1\underline{01}1\underline{00},1\underline{01}1\underline{01},\cdots$$

只看下划线上的数，是一个自然数序列

$$0000,0001,0010,0011,0100,0101, \cdots$$

为了让 $nums[n-1]$ 尽量小，我们应当选择 $x$ 的超集中最小的 $n$ 个数。

所以把 $x$ 的二进制中的 $0$ 视作 [空位」，往空位上填入 $n-1$，即为最小的 $nums[n-1]$

如果空位不足，往 $x$ 的前面添加前导零即可。

## 双指针

一个指针指向 $res$ 如果为空(0), 则看看 $n$ 的对应二进制位要不要填, 如果不为空则继续寻找空位:

```C++
class Solution {
public:
    long long minEnd(int n, int x) {
        long long res = x, w = 0;
        --n;
        while (n) {
            if (!((res >> w) & 1LL)) {
                res |= ((long long)n & 1LL) << w;
                n >>= 1;
            }
            ++w;
        }
        return res;
    }
};
```

时间复杂度: $O(\log{x} + \log{n})$

## lowbit优化
[1]: https://leetcode.cn/circle/discuss/CaOJ45/

上面我们需要移动到 $res$ 的`0`的位置, 因此需要逐位判断, 我们是否存在一个方法, $O(1)$ 的找到我们需要填的位置呢?

我们把 $x$ 取反, 那么 $1$ 就是我们要填的位置, 然后需要知道 这个 1 在哪一位了, 即 $lowbit$

特别地，只包含最小元素的子集，即二进制最低 1 及其后面的 0，也叫 lowbit，可以用`s & -s`算出。<sup>[[1]]</sup>

```C++
     s = 101100
    ~s = 010011
(~s)+1 = 010100 // 根据补码的定义，这就是 -s  =>  s 的最低 1 左侧取反，右侧不变
s & -s = 000100 // lowbit
```

```C++
class Solution {
public:
    long long minEnd(int n, int x) {
        n--;
        long long ans = x;
        int j = 0;
        for (long long t = ~x, lb; n >> j; t ^= lb /*去掉最低位的1*/) {
            lb = t & -t;
            // ((n >> j++) & 1) 是判断是否要填
            // ans |= lb 是给对应位填上1 (绝妙!)
            ans |= (long long) ((n >> j++) & 1) * lb;
        }
        return ans;
    }
};
```

## 还有高手: O(1)解法 利用bmi指令

看看就好orz

```C++
#include<immintrin.h>

class Solution {
public:
    __attribute__((target("bmi2")))
    long long minEnd(int n, int x) {
        return _pdep_u64(n - 1, ~uint64_t(x)) | x;
    }
};
```