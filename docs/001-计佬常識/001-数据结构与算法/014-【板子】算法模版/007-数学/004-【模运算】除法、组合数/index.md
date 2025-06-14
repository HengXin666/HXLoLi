# 模运算 | 模运算除法 | 组合数

# 一、模运算

常识的:

> 模运算 对 加减乘 是没有影响的:

$$
(a \pm b) \text{ \ mod \ } p = ((a \text{ \ mod \ } p) \pm (b \text{ \ mod \ } p)) \text{ \ mod \ } p \\
\\
(a \times b) \text{ \ mod \ } p = ((a \text{ \ mod \ } p) \times (b \text{ \ mod \ } p)) \text{ \ mod \ } p
$$

> [!TIP]
> 特别的, 对于 减法, 为了不会出现负数, 因此我们通常进行如下处理:
>
> $$
> (a - b) \text{ \ mod \ } p = ((a \text{ \ mod \ } p) - (b \text{ \ mod \ } p) + p) \text{ \ mod \ } p
> $$

但是对于除法, 它就不得货啦:

比如 $ \frac{27}{9} \text{ \ mod \ } 5 = 3 $, 但是 $\frac{27 \text{ \ mod \ } 5}{9 \text{ \ mod \ } 5} \text{ \ mod \ } 5 = \frac{3}{4} \text{ \ mod \ } 5$, 这甚至都不是一个整数.

这里直接放计算方法:

$$
\frac{a}{b} \text{ \ mod \ } p = a \times b^{p-2} \text{ \ mod \ } p
$$

其中, $p$ 是一个质数, 并且 $a = k_1b$, 并且 $b \not ={k_2p}$, 其中 $k_1, k_2$ 为正整数.

> [!NOTE]
> 实际上这个就是 `乘法逆元`:
>
> 在模条件下, 除以一个数 %b%, 等价于乘上另一个数 %x$. 这里的另一个数, 就是 在模 $p$ 条件下, 除以 %b% 情况下的 **乘法逆元**.

```cpp
constexpr int mod = 1e9 + 7;

template <typename T>
constexpr T qpow(T a, T b, T m) {
    T res;
    while (b) {
        if (b & 1)
            res = res * a % m;
        b >>= 1;
        a = a * a % m;
    }
    return res;
}

int a = 2233 * 64, b = 2233, res = 0;

res = ((a % mod) + (b % mod)) % mod;
res = ((a % mod) - (b % mod) + mod) % mod;
res = ((a % mod) * (b % mod)) % mod;
res = (a % mod) * (qpow(b, mod - 2, mod) % mod) % mod;
```

## 二、组合数学
### 2.1 排列数

$$
A_{n}^{a} = \frac{n!}{(n-a)!} 
$$

### 2.2 组合数

$$
C_{n}^{a} = C(a, n) = \begin{pmatrix}  
  n \\
  a
\end{pmatrix} = \frac{1}{a!} A_{n}^{a} = \frac{1}{a!} \times \frac{n!}{(n-a)!} = \frac{n!}{a!(n-a)!}
$$

预处理求解组合数, $O(n)$ 预处理, $O(1)$ 查询

```cpp
const int MOD = 1'000'000'007;
const int MX = 100'001; // 根据题目数据范围修改

long long F[MX]; // F[i] = i!
long long INV_F[MX]; // INV_F[i] = i!^-1 = qpow(i!, MOD-2)

long long qpow(long long x, int n) {
    long long res = 1;
    for (; n; n /= 2) {
        if (n % 2) {
            res = res * x % MOD;
        }
        x = x * x % MOD;
    }
    return res;
}

// C(m, n) = n! / (m! * (n - m)!)
//         = (n!) * (1 / m!) * (1 / (n - m)!)
auto init = [] {
    F[0] = 1; // 0! = 1

    // 递推求解 n!
    // n! = n * (n - 1)!
    for (int i = 1; i < MX; i++) {
        F[i] = F[i - 1] * i % MOD;
    }

    // 反推 1 / n!
    // 1 / (n - 1)! = (1 / n!) * n
    INV_F[MX - 1] = qpow(F[MX - 1], MOD - 2); // 1 / n!
                                              // a * b^{p - 2} % p
                                              // 1 * (n!)^{p - 2} % p
    for (int i = MX - 1; i; i--) {
        INV_F[i - 1] = INV_F[i] * i % MOD;
    }
    return 0;
}();

// 从 n 个数中选 m 个数的方案数
long long comb(int n, int m) {
    // C(m, n) = (n!) * (1 / m!) * (1 / (n - m)!)
    return m < 0 || m > n 
        ? 0 
        : F[n] * INV_F[m] % MOD * INV_F[n - m] % MOD;
}

class Solution {
public:
    int solve(vector<int>& nums) {
        // 预处理的逻辑写在 class 外面, 这样只会初始化一次
    }
};
```

> [!TIP]
> 核心思想:
>
> $$
> C(m, n) = \frac{n!}{a!(n-a)!} = n! \times \frac{1}{m!} \times \frac{1}{(n - m)!}
> $$
>
> 其中, $n! = n \times (n - 1)$ 递推求解,
>
> $\frac{1}{n!}$ 通过 $a \times b^{p - 2} \text{ \ mod \ } p$ 即 $(n!)^{p - 2}$ 求得.
>
> 然后 $\frac{1}{(n - 1)!} = n \times \frac{1}{n!}$ 递归求解即可~