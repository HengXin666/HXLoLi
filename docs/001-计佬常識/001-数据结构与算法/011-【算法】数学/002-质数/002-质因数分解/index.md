# 质因数分解
## [2507. 使用质因数之和替换后可以取到的最小值](https://leetcode.cn/problems/smallest-value-after-replacing-with-sum-of-prime-factors/)

给你一个正整数 n 。

- 请你将 n 的值替换为 n 的 **质因数** 之和，重复这一过程。

注意，如果 n 能够被某个质因数多次整除，则在求和时，应当包含这个质因数同样次数。
返回 n 可以取到的最小值。


```C++
示例 1：
输入：n = 15
输出：5
解释：最开始，n = 15 。
15 = 3 * 5 ，所以 n 替换为 3 + 5 = 8 。
8 = 2 * 2 * 2 ，所以 n 替换为 2 + 2 + 2 = 6 。
6 = 2 * 3 ，所以 n 替换为 2 + 3 = 5 。
5 是 n 可以取到的最小值。

示例 2：
输入：n = 3
输出：3
解释：最开始，n = 3 。
3 是 n 可以取到的最小值。
```

暴力解法: 直接模拟这个过程 学习 to 0x3f ([质因数分解 最近公共祖先【力扣周赛 324】](https://www.bilibili.com/video/BV1LW4y1T7if/) (非常好的一次周赛: 指可以学习到很多东西))
```C++
class Solution {
public:
    int smallestValue(int n) {
        while (114514) {
            /* 筛选质因数 */
            int x = n, s = 0;
            for (int i = 2; i * i <= x; ++i)
                for (; x % i == 0; x /= i)
                    s += i; // i 必定是质因数
            if (x > 1)
                s += x; // 没有任何大于1的可以整除它, 那么它也是质数
            
            if (s == n)
                return n;
            n = s;
        }
    }
};
```

时间复杂度: $O(\sqrt{n})$, 最坏情况下每次循环 $n$ 更新为 $2 + \dfrac{n}{2}$, 近似看成是 $n$ 减半，那么时间复杂度为 $O\left(\sqrt n + \sqrt\dfrac{n}{2} + \sqrt\dfrac{n}{4} + \cdots \right)=O\left(\sqrt n\cdot \left(1 + \sqrt\dfrac{1}{2} + \sqrt\dfrac{1}{4} + \cdots\right)\right) = O(\sqrt n)$ (首项为 1, 公比为 $\dfrac{1}{\sqrt 2}$ 的等比数列, $q < 1$ 所以是收敛的, 看成是常数.)

## 分解质因数 - 核心模版

获取 $x$ 的所有质因数
```C++
// 有重复的, 在内层for处, 因此需要小处理一下...
set<int> getPrimeFactorArray(int x) {
    set<int> res;
    for (int i = 2; i * i <= x; ++i)
        for (; x % i == 0; x /= i)
            res.insert(i);
    if (x > 1)
        res.insert(x);
    return res;
}

vector<int> getPrimeFactorArray(int x) {
    vector<int> res;
    for (int i = 2; i * i <= x; ++i) {
        if (x % i)
            continue;
        res.push_back(i);
        for (; x % i == 0; x /= i)
            ;
    }
    if (x > 1)
        res.push_back(x);
    return res;
}
```

原理剖析:

对于一个数 $x$, 如果从 $1, 2, 3, ..., q$ 找到第一个可以整除 $x$ 的数 $q$, 显然 $q$ 必须是一个 **质数**, 因为如果它不是质数<sup>*[反证法]*</sup>, 那么在它就可以分解出一个因子, 这个因子显然也是可以整除 $x$ 的. 而 它的因子肯定比它本身小, 那必然在前面出现, 这时候它就不是第一个出现的了! (自相矛盾 => 得证: $q$ 一定是一个 **质数**)

那么, 这时候已经筛选到`第一个`**质因子** $q$, 此时 对`x /= q`就是相当于**除去**一个质因子 $q$. (由于 $x$ 可能由多个相同的质因子 $q$ 乘得, 因此需要`for (; x % i == 0; x /= i);`)

最终剩下的 $x$ 如果大于1, 说明它除了1和本身不能被任何数整除(没有其他因子), 故它也是一个素数.

> 问:
> ```C++
if (x > 1)
    res.push_back(x);
> ```
> 这个 $x$ 有没有可能是重复的质数因子呢(之前已经出现的)?

> [!TIP]
> 在这个特定的算法中，这个情况是不可能发生的。原因在于，如果 $x$ 是重复的质数因子，它在前面的迭代中已经被除去了。在每一次迭代中，都会将 $x$ 除以它的质因数，直到无法再整除为止。因此，当代码执行到这一步时, $x$ 已经不可能是之前出现过的质因数，它要么是新的质因数，要么是最后一个剩余的大于1的数（即质数）。<sup>[By GPT-4o]</sup>