# 九、状压dp
## 9.1 排列型 ① 相邻无关
暴力做法是枚举所有排列, 对每个排列计算和题目有关的值, 时间复杂度（通常来说）是 $O(n!)$, 可以解决 $n ≤ 10$ 的问题.

状压 DP 可以把时间复杂度（通常来说）优化至 $O(n \times 2^n)$, 可以解决 $n ≤ 20$ 的问题.

一般有两种定义方式:

1. 定义 $f[S]$ 表示已经排列好的元素（下标）集合为 $S$ 时，和题目有关的最优值。通过枚举当前位置要填的元素（下标）来转移。

2. 定义 $f[S]$ 表示可以选的元素（下标）集合为 $S$ 时，和题目有关的最优值。通过枚举当前位置要填的元素（下标）来转移。

---

### 9.1.3 例1

题目示例:

[3376. 破解锁的最少时间 I](https://leetcode.cn/problems/minimum-time-to-break-locks-i/)

Bob 被困在了一个地窖里，他需要破解 $n$ 个锁才能逃出地窖，每一个锁都需要一定的 **能量** 才能打开。每一个锁需要的能量存放在一个数组`strength`里，其中`strength[i]`表示打开第 $i$ 个锁需要的能量。

Bob 有一把剑，它具备以下的特征:

- 一开始剑的能量为`0`。

- 剑的能量增加因子 $X$ 一开始的值为`1`。

- 每分钟，剑的能量都会增加当前的 $X$ 值。

- 打开第 $i$ 把锁，剑的能量需要到达 至少`strength[i]`。

- 打开一把锁以后，剑的能量会变回`0`, $X$ 的值会增加一个给定的值 $K$。

你的任务是打开所有 $n$ 把锁并逃出地窖，请你求出需要的 **最少** 分钟数。

请你返回 Bob 打开所有 $n$ 把锁需要的 **最少** 时间。

> 数据范围:
> - n == strength.length
> - 1 <= n <= 8
> - 1 <= K <= 10
> - 1 <= strength[i] <= $10^6$

这个是朴素版本 $O(n!)$:

```C++
class Solution {
public:
    int findMinimumTime(vector<int>& strength, int k) {
        unordered_set<int> vis;
        int n = strength.size();
        auto dfs = [&](this auto&& dfs, int i, int pw) {
            if (i < 0)
                return 0;
            int res = 1e9;
            for (int j = 0; j < n; ++j) {
                if (!vis.count(j)) {
                    vis.insert(j);
                    res = min(res, dfs(i - 1, pw + k) + ((strength[j] - 1) / pw + 1));
                    vis.erase(j);
                }
            }
            return res;
        };
        return dfs(n - 1, 1);
    }
};
```

我们可以把`vis`使用二进制压缩, 一个数字就是一个集合, 也是一个状态. (这个就是状态压缩, 把一个`unordered_set<int>`的状态, 压缩为一个`int`(二进制))

即:

```C++
class Solution {
    static constexpr int inf = 1e9;
public:
    int findMinimumTime(vector<int>& strength, int k) {
        int n = strength.size();
        vector<vector<int>> memo(
            n * k + 1, vector<int>(1 << n, inf)
        );
        auto dfs = [&](this auto&& dfs, int pw, int s) {
            if (s == (1 << n) - 1)
                return 0;
            int& res = memo[pw][s];
            if (res != inf)
                return res;
            for (int j = 0; j < n; ++j) {
                if (!(s >> j & 1)) {
                    res = min(
                        res, 
                        dfs(pw + k, s | (1 << j)) + ((strength[j] - 1) / pw + 1)
                    );
                }
            }
            return res;
        };
        return dfs(1, 0);
    }
};
```

> 然后 $pw$ 实际上可以通过 $s$ 中的`1`来计算, 可以省略一维:
> ```C++
> class Solution {
>     static constexpr int inf = 1e9;
> public:
>     int findMinimumTime(vector<int>& strength, int k) {
>         int n = strength.size();
>         vector<int> memo(1 << n, inf);
>         auto dfs = [&](this auto&& dfs, int s) {
>             if (s == (1 << n) - 1)
>                 return 0;
>             int& res = memo[s];
>             if (res != inf)
>                 return res;
>             int x = 1 + k * popcount((unsigned)s);
>             for (int j = 0; j < n; ++j) {
>                 if (!(s >> j & 1)) {
>                     res = min(
>                         res, 
>                         dfs(s | (1 << j)) + ((strength[j] - 1) / x + 1)
>                     );
>                 }
>             }
>             return res;
>         };
>         return dfs(0);
>     }
> };
> ```

---

### 9.1.3 例2

这题也很像: [1879. 两个数组最小的异或值之和](https://leetcode.cn/problems/minimum-xor-sum-of-two-arrays/)

> [!TIP]
> 此处展示的是以`0`作为递归边界; 而不是`(1 << n) - 1`.

```C++
class Solution {
    static constexpr int inf = 1e9;
public:
    int minimumXORSum(vector<int>& nums1, vector<int>& nums2) {
        int n = nums1.size();
        vector<int> memo(1 << n, inf);
        return [&](this auto&& dfs, int s) {
            if (!s)
                return 0;
            int& res = memo[s];
            if (res != inf)
                return res;
            int i = popcount((unsigned)s) - 1;
            for (int j = 0; j < n; ++j) {
                if ((s >> j) & 1) {
                    res = min(
                        res, 
                        dfs(s ^ (1 << j)) + (nums1[i] ^ nums2[j])
                    );
                }
            }
            return res;
        } ((1 << n) - 1);
    }
};
```

## 9.2 排列型 ② 相邻相关
一般定义 $f[S][i]$ 表示未选（或者已选）的集合为 $S$，且上一个填的元素（下标）为 $i$ 时，和题目有关的最优值。通过枚举当前位置要填的元素（下标）来转移。

时间复杂度（通常来说）是 $O(n^2 \times 2^n)$。

### 9.2.1 例1

- [996. 平方数组的数目](https://leetcode.cn/problems/number-of-squareful-arrays/)

如果一个数组的任意两个相邻元素之和都是 **完全平方数** ，则该数组称为 **平方数组** 。

给定一个整数数组`nums`，返回所有属于 **平方数组** 的`nums`的排列数量。

如果存在某个索引 $i$ 使得`perm1[i] != perm2[i]`，则认为两个排列`perm1`和`perm2`不同。

示例 1：

> 输入：nums = [1,17,8]
>
> 输出：2
>
> 解释：[1,8,17] 和 [17,8,1] 是有效的排列。

示例 2：

> 输入：nums = [2,2,2]
>
> 输出：1

提示：

- 1 <= nums.length <= 12
- 0 <= nums[i] <= $10^9$

---

1. 预处理完全平方数
2. 排列型 ② 相邻相关`状压dp`
3. 预处理阶乘(1~12), 把排列的部分清掉

```cpp
unordered_set<int> gpow;

int __init__ = [] {
    for (int i = 0; i <= 30'000; ++i)
        gpow.insert(i * i);
    return 0;
} ();

class Solution {
public:
    int numSquarefulPerms(vector<int>& nums) {
        const int n = nums.size();
        vector<vector<int>> memo(1 << n, vector<int>(n, -1));
        int _ = -1;
        unordered_map<int, int> cnt;
        for (int it : nums)
            ++cnt[it];
        int res = [&](this auto&& dfs, int s, int i) {
            if (!s)
                return 1;
            int& res = i == -1 ? _ : memo[s][i];
            if (res != -1)
                return res;
            res = 0;
            for (int j = 0; j < n; ++j) {
                if ((s >> j & 1) 
                    && (i == -1 || gpow.count(nums[i] + nums[j]))
                ) {
                    res += dfs(s ^ (1 << j), j);
                }
            }
            return res;
        } ((1 << n) - 1, -1);
        for (auto [_, c] : cnt) { // 1 ~ 12 的阶乘
            res /= array<int, 13>{0, 1, 2, 6, 24, 
                120, 720, 5040, 40320, 362880, 
                3628800, 39916800, 479001600
            }[c];
        }
        return res;
    }
};
```

更加通用的模版: (已经把多余的去掉了, 因此只是`模版`)

```C++
class Solution {
public:
    int numSquarefulPerms(vector<int>& nums) {
        const int n = nums.size();
        vector<vector<int>> memo(1 << n, vector<int>(n, -1));
        auto dfs = [&](this auto&& dfs, int s, int i) {
            if (!s)
                return 1; // 递归边界
            int& res = memo[s][i];
            if (res != -1)
                return res;
            res = 0;
            for (int j = 0; j < n; ++j) {
                if ((s >> j & 1) && gpow.count(nums[i] + nums[j]) /*题目条件*/) {
                    res += dfs(s ^ (1 << j), j); // 转移方程
                }
            }
            return res;
        };
        int res = 0;
        // 在外部枚举第一个数, 内部就不需要进行边界处理!
        for (int i = 0; i < n; ++i)
            res += dfs(((1 << n) - 1) ^ (1 << i), i); // 入口
        return res;
    }
};
```

学会了, 可以秒杀: [2741. 特别的排列](https://leetcode.cn/problems/special-permutations/)

## 9.3 旅行商问题（TSP）

本质上就是排列型 ②。

- 状压dfs的vis即可: [847. 访问所有节点的最短路径](https://leetcode.cn/problems/shortest-path-visiting-all-nodes/) (有状压弗洛伊德/A*解法)

## 9.4 枚举子集的子集
一般定义 $f[S]$ 表示未选（或者已选）的集合为 $S$ 时，和题目有关的最优值。通过枚举 $S$ (或者 $S$ 的 补集 $CuS$ )的子集来转移。

时间复杂度（通常来说）是 $O(3^n)$, 证明见题解(使用二项式定理做的)。

值得注意的是，枚举子集的子集还可以用「选或不选」来做，对于存在无效状态的情况，可以做到更优的时间复杂度。具体见`1349 题解`最后的写法。(另外, 似乎不太适合写记忆化?!)

前置知识:

1. 枚举子集, 对于二进制集合`1011`, 它的子集有`{1010, 1001, 1000, 0000}`

可以使用以下方法, 高效枚举:
```C++
int s = 0b1011;
for (int sub = s; sub; sub = (sub - 1) & s) {
    std::cout << sub << '\n';
}
```

2. 求补集

```C++
int u = (1 << 4) - 1; // 全集
int s = 0b1011;       // 集合 s
int CuS = s ^ u;      // s 的补集
```

3. 迭代求集合, 有一个数组`int arr[n]`, 有一个集合`int s[1 << n]`; 如果我们使用集合 $s[bit]$ 表示 $bit$ 对应的数位为 $arr$ 的索引的元素的和. 可以使用以下方法: (此处的`+`可以是任何预处理的计算, 依题目而设计)

```C++
int arr[n];
int s[1 << n];
for (int i = 0; i < n; ++i)
    for (int j = 0, bit = 1 << i; j < bit; ++j)
        // 对于新的集合`bit`, 交上集合`j`(已经求过的),
        // 再加上属于自己集合表示位的`i`对应的元素值`arr[i]`
        s[j | bit] = s[j] + arr[i];
```

---

相关题目:

- [1986. 完成任务的最少工作时间段](https://leetcode.cn/problems/minimum-number-of-work-sessions-to-finish-the-tasks/)

```C++
class Solution {
public:
    int minSessions(vector<int>& tasks, int sessionTime) {
        int n = tasks.size();
        vector<int> sum(1 << n);
        for (int i = 0; i < n; ++i)
            for (int j = 0, bit = 1 << i; j < bit; ++j)
                sum[bit | j] = sum[j] + tasks[i];
        vector<int> memo(1 << n, (int)1e9);
        return [&](this auto&& dfs, int s) {
            if (!s)
                return 0;
            int& res = memo[s];
            if (res != (int)1e9)
                return res;
            for (int sub = s; sub; sub = (sub - 1) & s) {
                if (sum[sub] <= sessionTime) { // 子集耗时 不足 sessionTime
                    res = min(
                        res,
                        dfs(s ^ sub) + 1
                    );
                }
            }
            return res;
        } ((1 << n) - 1);
    }
};
```

- [2305. 公平分发饼干](https://leetcode.cn/problems/fair-distribution-of-cookies/)

## 9.5 其他状压 DP

这里好难...摊牌了