# GCD 与 LCM
## 一、GCD与LCM的计算

```C++
template <typename T>
T gcd(T a, T b) {
    return b == 0 ? a : gcd(b, a % b);
}

template <typename T>
T lcm(T a, T b) {
    return a * (b / gcd(a, b));
}
```

## 二、翡翠定理

内容: 对于不全为零的任意整数 $a, b$ 记 $g = \gcd(a, b)$, 则对于任意整数 $x, y$ 都满足 $a \times x + b \times y$ 是 $g$ 的倍数. 特别的, 存在整数 $x$ 和 $y$ 满足 $a \times x + b \times y = g$

「裴蜀定理」也可以推广到多个整数的情况。对于不全为零的任意 $n$ 个整数 $a_1, a_2, \ldots, a_n$, 记这 $n$ 个数的最大公约数为 $g$, 则对于任意 $n$ 个整数 $x_1, x_2, \ldots, x_n$ 都满足 $\sum_{i=1}^n a_i \times x_i$ 是 $g$ 的倍数。

- 一个重要的推论是: 正整数 $a_1$ 到 $a_n$ 的最大公约数是 $1$ 的充分必要条件是存在 $n$ 个整数 $x_1$ 到 $x_n$ 满足 $\sum_{i=1}^n a_i \times x_i = 1$。
