# 3154. 到达第 K 级台阶的方案数
链接: [3154. 到达第 K 级台阶的方案数](https://leetcode.cn/problems/find-number-of-ways-to-reach-the-k-th-stair/)

给你有一个 非负 整数 k 。有一个无限长度的台阶，最低 一层编号为 0 。

- 虎老师有一个整数 jump ，一开始值为 0 。虎老师从台阶 1 开始，虎老师可以使用 任意 次操作，目标是到达第 k 级台阶。假设虎老师位于台阶 i ，一次 操作 中，虎老师可以：

- 向下走一级到 i - 1 ，但该操作 不能 连续使用，如果在台阶第 0 级也不能使用。
向上走到台阶 $i + 2^{jump}$ 处，然后 jump 变为 jump + 1 。

请你返回虎老师到达台阶 k 处的总方案数。

注意 ，虎老师可能到达台阶 k 处后，通过一些操作重新回到台阶 k 处，这视为不同的方案。

提示:
- 0 <= k <= 10^9

# 题解
## HX: py@cache

一开始是枚举可能j, 然后 dfs -> i = 1 的, 但是死活过不了样例qwq.., 然后试一试这个pass的思路, 然后题解居然过了...orz

```py
class Solution:
    def waysToReachStair(self, k: int) -> int:
        @cache
        def dfs(i: int, j: int, do: int) -> int:
            if (i == k):
                return 1 + (0 if do == 1 else dfs(i - 1, j, 1)) + dfs(i + 2 ** j, j + 1, 0)
            if (i > k + 1 or i < 0):
                return 0
            return (0 if do == 1 else dfs(i - 1, j, 1)) + dfs(i + 2 ** j, j + 1, 0)
        return dfs(1, 0, 0)
```

## 0x3f: 两种方法：记忆化搜索/组合数学
- [两种方法：记忆化搜索/组合数学（Python/Java/C++/Go）](https://leetcode.cn/problems/find-number-of-ways-to-reach-the-k-th-stair/solutions/2782792/liang-chong-fang-fa-ji-yi-hua-sou-suo-zu-j227)

### 记忆化
```C++
class Solution {
    unordered_map<long long, int> memo;

    int dfs(int i, int j, bool preDown, int k) {
        if (i > k + 1) {
            return 0;
        }
        long long p = (long long) i << 32 | j << 1 | preDown; // 用一个 long long 表示状态
        if (memo.contains(p)) { // 之前算过了
            return memo[p];
        }
        int res = i == k;
        res += dfs(i + (1 << j), j + 1, false, k); // 操作二
        if (i && !preDown) {
            res += dfs(i - 1, j, true, k); // 操作一
        }
        return memo[p] = res; // 记忆化
    };

public:
    int waysToReachStair(int k) {
        return dfs(1, 0, false, k);
    }
};
```
时间复杂度 $O(log^2k)$

### 组合数学

不会qwq..orz...

```C++
int c[31][31];
auto init = [] {
    for (int i = 0; i < 31; i++) {
        c[i][0] = c[i][i] = 1;
        for (int j = 1; j < i; j++) {
            c[i][j] = c[i - 1][j - 1] + c[i - 1][j];
        }
    }
    return 0;
}();

class Solution {
public:
    int waysToReachStair(int k) {
        int ans = 0;
        for (int j = k > 1 ? 32 - __builtin_clz(k - 1) : 0; (1 << j) - k <= j + 1; j++) {
            ans += c[j + 1][(1 << j) - k];
        }
        return ans;
    }
};
```

时间复杂度 $O(1)$