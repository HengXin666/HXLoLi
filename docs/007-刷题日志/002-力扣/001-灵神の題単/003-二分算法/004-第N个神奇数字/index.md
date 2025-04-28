# 878. 第 N 个神奇数字
链接: [878. 第 N 个神奇数字](https://leetcode.cn/problems/nth-magical-number/)

一个正整数如果能被 a 或 b 整除，那么它是神奇的。

给定三个整数 n , a , b ，返回第 n 个神奇的数字。因为答案可能很大，所以返回答案 对 10^9 + 7 取模 后的值。

# 题解
## 二分 + 容斥原理

- 灵神: [二分答案+容斥原理](https://leetcode.cn/problems/nth-magical-number/solutions/1984641/er-fen-da-an-rong-chi-yuan-li-by-endless-9j34)

<p><img src="https://pic.leetcode.cn/1669032532-GjXsyF-878-2.png" alt="878-2.png"></p>

- 为什么上界是`min(a, b) * n`?
    - 看题: 被a或者b整除的第n个数, 这样肯定满足题意. 然后就是缩小即可. 

- 容斥原理的应用
    - 可以被a整除或可以被b整除 = 可以被a整除 + 可以被b整除 - 可以被a和b整除
```C++
class Solution {
    static int constexpr mod = 1e9 + 7;
    using ll = long long;
public:
    int nthMagicalNumber(int n, int a, int b) {
        ll l = 0, r = (ll) min(a, b) * n, lcm = std::lcm(a, b);
        auto fk = [&](ll x) -> bool {
            return x / a + x /b - x / lcm < n;
        };
        while (l + 1 < r) {
            ll mid = ((r - l) >> 1) + l;
            (fk(mid) ? l : r) = mid;
        }
        return r % mod;
    }
};
```