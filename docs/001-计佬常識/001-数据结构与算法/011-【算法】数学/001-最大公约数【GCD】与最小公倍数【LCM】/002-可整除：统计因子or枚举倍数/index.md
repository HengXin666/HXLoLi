# 统计因子/枚举倍数
给你两个整数数组 $nums1$ 和 $nums2$，长度分别为 $n$ 和 $m$。同时给你一个正整数 $k$。

如果 $nums1[i]$ 可以被 $nums2[j] \times k$ 整除，则称数对 $(i, j)$ 为 **优质数对** ( $0 \le i \le n - 1, 0 \le j \le m - 1$ )。

返回 **优质数对** 的总数。

- 学习链接: [两种方法：统计因子/枚举倍数（Python/Java/C++/Go）](https://leetcode.cn/problems/find-the-number-of-good-pairs-ii/solutions/2790631/tong-ji-yin-zi-ge-shu-pythonjavacgo-by-e-bl3o)

## 统计因子
我们知道, 如果 $nums1[i]$ 可以被 $nums2[j] \times k$ 整除, 则有 $$nums1[i] \ \% \ (nums2[j] \times k) == 0$$

即 $nums2[j] \times k$ 是 $nums1[i]$ 的因子, 因此我们可以先统计 $nums1[i]$ 因子的个数, 再遍历 $nums2[j]$ 看它是否在 $nums1[i]$ 的因子里面.

- 我们知道统计一个数因子的算法的时间复杂度是 $O(\sqrt{n})$ 的.

类比[判断质数](../../002-质数/001-判断质数/index.md)的那个: 

- 如果一个数 $d$ 可以 **整除** $x$, 那么 $d (d \in [1, x])$ 是 $x$ 的一个因子, 但是同时 $\frac{x}{d}$ 也是 $x$ 的一个因子, 所以我们在 $O(\sqrt{n})$ 就可以一次找到所有的因子!

- 特别注意的是: 如果 $d \times d = x$ 时候, 会重复统计, 因此需要去掉一个!

代码如下, (小优化: $d \le \sqrt{x} \Longleftrightarrow d \times d \le x$ (左右同时平方嘛~))
```C++
unordered_map<int, int> cnt;

int x = 114514; // 统计 114514 的因子
for (int d = 1; d * d <= x; d++) {
    if (x % d)
        continue;
    
    cnt[d]++;
    if (d * d < x)
        cnt[x / d]++;
}
```

总的代码如下: (小优化: a[i] % (b[i] * k) <=> (a[i] / k) % (b[i]), 这样可以减少枚举范围)

```C++
class Solution {
public:
    long long numberOfPairs(
        vector<int>& nums1, vector<int>& nums2, int k) {
/*
(a[i] / k) % (b[i]) == 0

枚举 a[i] / k 的 因子
*/
        unordered_map<int, int> cnt;
        for (int it : nums1) {
            if (it % k)
                continue;
            it /= k;
            for (int i = 1; i <= sqrt(it); ++i) {
                if (it % i == 0) {
                    ++cnt[i];
                    if (i * i < it)
                        ++cnt[it / i];
                }
            }
        }
        long long res = 0;
        for (int it : nums2)
            res += cnt.count(it) ? cnt[it] : 0;
        return res;
    }
};
```

时间复杂度: $O(n\sqrt{\frac{U}{k}} + m)$ 其中 $n$ 是 $nums_1$ 的长度, $m$ 是 $nums_2$ 的长度, $U = \max(nums_1)$ 

## 枚举倍数 (调和级数枚举)

1. 先去重 $nums_1$ 和 $nums_2$ 到 $cnt_1$ 和 $cnt_2$ 中, 记 $nums_1$ 中元素最大值为 $u$.

2. 枚举 $cnt_2$ 的元素 $i$, 然后枚举 $i$ 的倍数: $i, 2i, 3i, ...$ 直到 $u$, 然后看看这些值有那些是在 $cnt_1$ 的, 累加到 $s$ 中.

```C++
class Solution {
public:
    long long numberOfPairs(
        vector<int>& nums1, vector<int>& nums2, int k) {
/*
(a[i] / k) % (b[i]) == 0

b[i] 的倍数, 使其不超过 max(a[i] / k)
*/
        unordered_map<int, int> cnt1, cnt2;
        for (int it : nums1) {
            if (it % k)
                continue;
            ++cnt1[it / k];
        }
        if (!cnt1.size())
            return 0;

        int maxx = 0;
        for (auto [i, _] : cnt1)
            if (i > maxx)
                maxx = i;

        for (int it : nums2)
            ++cnt2[it];

        long long res = 0;
        for (auto [i, v] : cnt2) {
            int s = 0;
            for (int j = i; j <= maxx; j += i) { // 枚举倍数
                s += cnt1.count(j) ? cnt1[j] : 0;
            }
            res += (long long) s * v;
        }

        return res;
    }
};
```

时间复杂度: $O(n + m + \frac{U}{k}\log{m})$, 其中 $n$ 是 $nums_1$ 的长度, $m$ 是 $nums_2$ 的长度, $U = \max(nums_1)$.

其中, $\frac{U}{k}\log{m}$ 由**调和级数**推得.

证明:

最坏情况下: $cnt_2.size = nums_2.size = m$ (全部只出现一次)

并且由于 哈希表 $cnt_2$ 肯定不能有重复的 $Key$, 那么假设 $i \in \{1, 2, 3, ..., m\}$

那么从 $i$ 枚举 $i, 2i, 3i, ..., u$ 的次数 为 $x=\left \lfloor \frac{u}{i} \right \rfloor $ 次

即枚举 $cnt_2$ 需要: $$\frac{u}{1} + \frac{u}{2} + \frac{u}{3} + ... + \frac{u}{m} \text{ 次}$$ 即 $$u(1 + \frac{1}{2} + \frac{1}{3} + ... + \frac{1}{m}) \text{ 次}$$ 即 $$u \times \sum^m_{i=1}{\frac{1}{i}} \text{ 次}$$ 然后再严谨一些? $$u \times \int{\frac{1}{x}} \text{ 次}$$ 即 $$u \times (\ln{x} + c) \text{ 次}$$ 一共有 $m$ 个, 并且 $uc$ 是常数, $u$ 实际上是我们的 $\frac{U}{k}$, 时间复杂度不理会 $log$ 的底数, 故整理得 $$\text{时间复杂度为: } O(\frac{U}{k} \log{m})$$