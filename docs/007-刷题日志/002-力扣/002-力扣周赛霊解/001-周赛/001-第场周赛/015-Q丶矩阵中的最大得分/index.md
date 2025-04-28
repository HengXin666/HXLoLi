# 3148. 矩阵中的最大得分
链接: [3148. 矩阵中的最大得分](https://leetcode.cn/problems/maximum-difference-score-in-a-grid/)

给你一个由 正整数 组成、大小为 m x n 的矩阵 grid。你可以从矩阵中的任一单元格移动到另一个位于正下方或正右侧的任意单元格（不必相邻）。从值为 c1 的单元格移动到值为 c2 的单元格的得分为 c2 - c1 。

你可以从 任一 单元格开始，并且必须至少移动一次。

返回你能得到的 **最大** 总得分。

提示:
- m == grid.length
- n == grid[i].length
- 2 <= m, n <= 1000
- 4 <= m * n <= 10^5
- 1 <= grid[i][j] <= 10^5

# 题解
## 我: 暴力dp
击败了 4% 的 C++...
```C++
class Solution {
    const int inf = -1e9;
public:
    int maxScore(vector<vector<int>>& g) {
        int n = g.size();
        int m = g[0].size();
        vector<vector<int>> f(n + 1, vector<int>(m + 1, inf));
        int res = inf;
        
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                for (int y = 0; y < i; ++y) {
                    f[i + 1][j + 1] = max(
                        f[i + 1][j + 1], 
                        max(f[y + 1][j + 1], 0)
                        + (g[i][j] - g[y][j]));
                }
                
                for (int x = 0; x < j; ++x) {
                    f[i + 1][j + 1] = max(
                        f[i + 1][j + 1], 
                        max(f[i + 1][x + 1], 0)
                        + (g[i][j] - g[i][x]));
                }
                
                res = max(res, f[i + 1][j + 1]);
            }
        }
        
        return res;
    }
};
```

## 0x3f 二维前缀最小值
因为 A -> B 即 $+ (C_B - C_A)$, 而 B -> D 即 $+ (C_D - C_B)$

则 A -> B -> D 有 $$+ (C_B - C_A)+ (C_D - C_B)$$ 即 $$C_D - C_A$$ 因此可以发现和过程无关, 只和`始末点`有关(类似于`重力势能`)

因此我们需要求: $[y][x] \to [i][j]$ 即 $\max(C_{[y][x]} - C_{[i][j]})$ (i != y && j != x)

可以枚举 [i][j], 然后 [y][x] 只能是: `0 <= y <= i && 0 <= x <= j && i != y && j != x` 从左上角部分转移过来的:

```C++
class Solution {
    const int inf = 1e9;
public:
    int maxScore(vector<vector<int>>& g) {
        int n = g.size();
        int m = g[0].size();
        vector<vector<int>> minArr(n + 1, vector<int>(m + 1, inf));
        int res = -inf;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                minArr[i + 1][j + 1] = min(minArr[i][j + 1], minArr[i + 1][j]);
                res = max(res, g[i][j] - minArr[i + 1][j + 1]); // 大 - 小
                minArr[i + 1][j + 1] = min(minArr[i + 1][j + 1], g[i][j]);
            }
        }
        return res;
    }
};
```
