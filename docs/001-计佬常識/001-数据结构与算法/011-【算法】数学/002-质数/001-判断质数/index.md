# 判断质数
## 什么是tm的质数

- **质数**: 即除了 $1$ 和 $它本身$, 不能被其他数整除的数.

> 质数是指 **大于1** 且只能被 $1$ 和 $自身$ 整除的 **正整数**, <span style="color:red"> 1 $不是$ 质数</span>

## 暴力求解 $O(N)$

```C++
bool ifFxxk(int m) { // 判断 m 是不是质数
    for (int i = 2; i < m; ++i)
        if (!(m % i))
            return false;
    return m > 1;
}
```

## 简单优化 $O(\sqrt{N})$

根据上面代码, 对于 $12$ 和 $13$ 我们如何判断他们是不是质数:

1. 如果`12 % 2 == 0`, 那么`12 % 2 * 2 == 0`
    - 如果`12 % 3 == 0`, 那么`12 % 3 * 2 == 0`

2. 如果`13 % 2 != 0`, 那么`13 % 2 * 2 != 0`
    - 如果`13 % 3 == 0`, 那么`13 % 3 * 2 == 0`

显然我们不需要循环这么多次, 因为 m 如果不能被 2 整除, 那么也一定不能被 2 * k 整除.

所以我们最多只需要循环到 `sqrt(m)` 即可.

故:

```C++
bool ifFxxk(int m) { // 判断 m 是不是质数
    int n = sqrt(m);
    for (int i = 2; i <= n; ++i)
        if (!(m % i))
            return false;
    return m > 1;
}
```

使用 $i \le \sqrt m$ 等价于 $i^2 \le m$
```C++
bool ifFxxk(int m) { // 判断 m 是不是质数
    for (int i = 2; i * i <= m; ++i)
        if (!(m % i))
            return false;
    return m > 1;
}
```