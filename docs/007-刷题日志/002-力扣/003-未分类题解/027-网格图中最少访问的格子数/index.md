# 2617. 网格图中最少访问的格子数

第 340 场周赛 Q4 困难2582

给你一个下标从 0 开始的 m x n 整数矩阵 grid 。你一开始的位置在 左上角 格子 (0, 0) 。

当你在格子 (i, j) 的时候，你可以移动到以下格子之一：

- 满足 j < k <= grid[i][j] + j 的格子 (i, k) （向右移动），或者
- 满足 i < k <= grid[i][j] + i 的格子 (k, j) （向下移动）。

请你返回到达 右下角 格子 (m - 1, n - 1) 需要经过的最少移动格子数，如果无法到达右下角格子，请你返回 -1 。

## 示例 1：

输入：grid = [[3,4,2,1],[4,2,3,1],[2,1,0,0],[2,4,0,0]]<br>
输出：4<br>
解释：上图展示了到达右下角格子经过的 4 个格子。

## 示例 2：

输入：grid = [[3,4,2,1],[4,2,1,1],[2,1,1,0],[3,4,1,0]]<br>
输出：3<br>
解释：上图展示了到达右下角格子经过的 3 个格子。

## 示例 3：

输入：grid = [[2,1,0],[1,0,0]]<br>
输出：-1<br>
解释：无法到达右下角格子。

提示：

$
m == grid.length\\
n == grid[i].length\\
1 <= m, n <= 10^5\\
1 <= m * n <= 10^5\\
0 <= grid[i][j] < m * n\\
grid[m - 1][n - 1] == 0\\
$

# 题解
## 暴力dp

时间复杂度 $O(m*n*(m + n))$ 显然超时

```C++
class Solution {
    const int INF = 1e9; 
public:
    int minimumVisitedCells(vector<vector<int>>& grid) {
/*
在 (i, j) 可以向
(i, k) 移动 j < k <= g[i][j] + j 可以选择向右 [0, g[i][j]] 单位
(k, j) 移动 j < k <= g[i][j] + i 可以选择向右 [0, g[i][j]] 单位

定义 移动到 [i, j] 所需的最少移动格子数为 f[i][j] 初始化为 INF
    f[i][j] = min(
        for x in (0, j) && x > j - x 的 f[i][x] + 1,
        for y in (0, i) && y > i - y 的 f[y][j] + 1,
    )
超时 O(N^3)

那? dfs+回溯?! 也不对啊, 最坏时间复杂度O(n!)才有-1

果然需要堆... fk

因为 需要的是 min (for...&&, for...&&) + 1
    实际上就是tm的 min (min(for &&), min(for &&)) + 1
    所以使用一个最小堆来维护? 但是不好写qwq...
*/
        int m = grid.size();
        int n = grid[0].size();
        vector<vector<int>> f(m, vector<int>(n, INF));

        f[0][0] = 1;
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                for (int x = 0; x < j; ++x) {
                    if (grid[i][x] >= j - x) {
                        f[i][j] = min(f[i][j], f[i][x] + 1);
                    }
                }

                for (int y = 0; y < i; ++y) {
                    if (grid[y][j] >= i - y) {
                        f[i][j] = min(f[i][j], f[y][j] + 1);
                    }
                }
            }
        }

        // for (int i = 0; i < m; ++i) {
        //     for (int j = 0; j < n; ++j) {
        //         printf("%d ", f[i][j] != INF ? f[i][j] : -1);
        //     }
        //     printf("\n");
        // }

        return f[m - 1][n - 1] == INF ? -1 : f[m - 1][n - 1];
    }
};
```

## 单调队列优化dp

[两种方法：单调栈优化 DP / 贪心+最小堆（Python/Java/C++/Go/JS/Rust）](https://leetcode.cn/problems/minimum-number-of-visited-cells-in-a-grid/solutions/2216329/dan-diao-zhan-you-hua-dp-by-endlesscheng-mc50)

对于 $\min _{k=j+1}^{j+g} f[i][k]$ ，在倒序枚举 $j$ 时， $k$ 的左边界 $j+1$ 是在单调减小的，但右边界没有单调性。联想到滑动窗口最小值的做法，我们用一个 $f$ 值底小顶大的单调栈来维护 $f[i][j]$ 及其下标 $j$ 。由于 $j$ 是倒序枚举，单调栈中的下标是底大顶小的，从那么在单调栈上二分查找最大的不超过 $j+g$ 的下标 $k$ ，对应的 $f[i][k]$ 就是要计算的最小值。

对于 $\min _{k=i+1}^{i+g} f[k][j]$ 也同理，每一列都需要维护一个单调栈。

```C++
class Solution {
public:
    int minimumVisitedCells(vector<vector<int>> &grid) {
        int m = grid.size(), n = grid[0].size(), mn;
        vector<vector<pair<int, int>>> col_stacks(n); // 每列的单调栈
        vector<pair<int, int>> row_st; // 行单调栈
        for (int i = m - 1; i >= 0; i--) {
            row_st.clear();
            for (int j = n - 1; j >= 0; j--) {
                int g = grid[i][j];
                auto &col_st = col_stacks[j];
                mn = i < m - 1 || j < n - 1 ? INT_MAX : 1;
                if (g) { // 可以向右/向下跳
                    // 在单调栈上二分查找最优转移来源
                    auto it = lower_bound(row_st.begin(), row_st.end(), j + g, [](const auto &a, const int b) {
                        return a.second > b;
                    });

                    if (it < row_st.end())
                        mn = it->first + 1;
                    it = lower_bound(col_st.begin(), col_st.end(), i + g, [](const auto &a, const int b) {
                        return a.second > b;
                    });

                    if (it < col_st.end())
                        mn = min(mn, it->first + 1);
                }
                
                if (mn < INT_MAX) {
                    // 插入单调栈
                    while (!row_st.empty() && mn <= row_st.back().first) {
                        row_st.pop_back();
                    }
                    row_st.emplace_back(mn, j);

                    while (!col_st.empty() && mn <= col_st.back().first) {
                        col_st.pop_back();
                    }
                    col_st.emplace_back(mn, i);
                }
            }
        }
        return mn < INT_MAX ? mn : -1; // 最后一个算出的 mn 就是 f[0][0]
    }
};
```

## 还有很多方法
请去题解区