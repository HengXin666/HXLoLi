# 参考链接
1. [oi-wiki](https://oi-wiki.org/math/number-theory/gcd/)
2. [【算法】辗转相除法求最大公约数](https://blog.csdn.net/Qiuhan_909/article/details/125941749)

# 求最大公约数: 辗转相除法

> [!TIP]
> 什么是tmd **最大公约数（Greatest Common Divisor，简称GCD）**
> > 例如: 10 与 8, 那么的 **最大公约数** 为 2
> >
> > 即 **最大**可以同时把 10 和 8 整除的数, 称为 10 与 8 的 **最大公约数**

辗转相除法，又称**欧几里德算法（Euclidean Algorithm）**，是求两个数的**最大公约数（greatest common divisor）** 的一种方法。用较大的数除以较小的数，再以除数和余数反复做除法运算，当余数为0时，取当前算式除数为最大公约数。

**过程**

- 如果我们已知两个数 $α$ 和 $b$，如何求出二者的最大公约数呢？

    不妨设 $α>b$

- 我们发现如果 $b$ 是 $α$ 的约数，那么 $b$ 就是二者的最大公约数。下面讨论不能整除的情况，即

  $α=b×q+r$, 其中 $r<b$ 。
我们通过证明可以得到 $gcd(a,b) = gcd(b,\ α \mod b)$

递归
```C++
// 写法 1
int gcd(int a, int b) {
    if (b == 0) return a;
    return gcd(b, a % b);
}

// 写法 2
int gcd(int a, int b) { return b == 0 ? a : gcd(b, a % b); }
```

迭代写法
```C++
int gcd(int a, int b) {
    while (b != 0) {
        int tmp = a;
        a = b;
        b = tmp % b;
    }
    return a;
}
```

时间复杂度: $O(N)$

# 求最小公倍数
> [!TIP]
> 什么是tmd **最小公倍数（Least Common Multiple，简称LCM）**
> > 例如: 10 与 8, 那么的 **最小公倍数** 为 40
> >
> > 即 **最小**可以同时 **被** 10 和 8 整除的数 称为 **最大公约数**

求 $LCM(a, b)$, 等价于 $\frac{a \times b}{GCD(a, b)}$, 直接记忆, 不用证明


```C++
#include <iostream>

using namespace std;

using ll = long long;

ll gcd(ll a, ll b) {
    return b == 0 ? a : gcd(b, a % b);
}

int main() {
    ll a, b;
    cin >> a >> b;
    cout << a * (b / gcd(a, b)) << '\n'; // 得 lcm 
    return 0;
}
```