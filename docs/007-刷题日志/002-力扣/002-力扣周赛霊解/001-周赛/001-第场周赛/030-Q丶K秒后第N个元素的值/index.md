# 3179. K 秒后第 N 个元素的值
链接: [3179. K 秒后第 N 个元素的值](https://leetcode.cn/problems/find-the-n-th-value-after-k-seconds/)

给你两个整数 n 和 k。

最初，你有一个长度为 n 的整数数组 a，对所有 0 <= i <= n - 1，都有 a[i] = 1 。每过一秒，你会同时更新每个元素为其前面所有元素的和加上该元素本身。例如，一秒后，a[0] 保持不变，a[1] 变为 a[0] + a[1]，a[2] 变为 a[0] + a[1] + a[2]，以此类推。

返回 k 秒后 a[n - 1] 的值。

由于答案可能非常大，返回其对 10^9 + 7 取余 后的结果。

# 题解
## 前缀和的前缀和

实际上左边+头顶就是下一行

```C++
class Solution {
    static const int mod = 1e9 + 7;
public:
    int valueAfterKSeconds(int n, int k) {
        vector<int> f(n, 1);
        
        for (int j = 0; j < k; ++j)
            for (int i = 1; i < n; ++i)
                f[i] = (f[i] + f[i - 1]) % mod;
        
        return f[n - 1];
    }
};
```

## 杨辉三角

- 你斜着看 相当于计算的是杨辉三角第 $n+k$ 排的第 $n$ 个数，即 $$C_{n+k−1}^{n−1}=C_{n+k−1}^{k}$$

- 预处理阶乘及其逆元后，即可 $\mathcal{O}(1)$ 计算组合数。

- 计算逆元需要用到费马小定理，证明见...

```C++
const int MOD = 1'000'000'007;
const int MX = 2001;

long long q_pow(long long x, int n) {
    long long res = 1;
    for (; n > 0; n /= 2) {
        if (n % 2) {
            res = res * x % MOD;
        }
        x = x * x % MOD;
    }
    return res;
}

// 组合数模板
long long fac[MX], inv_fac[MX];

auto init = [] {
    fac[0] = 1;
    for (int i = 1; i < MX; i++) {
        fac[i] = fac[i - 1] * i % MOD;
    }
    inv_fac[MX - 1] = q_pow(fac[MX - 1], MOD - 2);
    for (int i = MX - 1; i > 0; i--) {
        inv_fac[i - 1] = inv_fac[i] * i % MOD;
    }
    return 0;
}();

long long comb(int n, int k) {
    return fac[n] * inv_fac[k] % MOD * inv_fac[n - k] % MOD;
}

class Solution {
public:
    int valueAfterKSeconds(int n, int k) {
        return comb(n + k - 1, k);
    }
};
```

$f(i, j) = f(i - 1, j) +f(i, j - 1)$ 这条递推式如何与组合数产生关系?

灵神从动态规划的角度进行了证明:

- 请见视频 [bitset 优化 DP【力扣周赛 401】](https://www.bilibili.com/video/BV1h7421R78s/)