# 741. 摘樱桃
链接: [741. 摘樱桃](https://leetcode.cn/problems/cherry-pickup/)

给你一个 n x n 的网格 grid ，代表一块樱桃地，每个格子由以下三种数字的一种来表示：

0 表示这个格子是空的，所以你可以穿过它。
1 表示这个格子里装着一个樱桃，你可以摘到樱桃然后穿过它。
-1 表示这个格子里有荆棘，挡着你的路。
请你统计并返回：在遵守下列规则的情况下，能摘到的最多樱桃数：

- 从位置 (0, 0) 出发，最后到达 (n - 1, n - 1) ，只能向下或向右走，并且只能穿越有效的格子（即只可以穿过值为 0 或者 1 的格子）；
- 当到达 (n - 1, n - 1) 后，你要继续走，直到返回到 (0, 0) ，只能向上或向左走，并且只能穿越有效的格子；
- 当你经过一个格子且这个格子包含一个樱桃时，你将摘到樱桃并且这个格子会变成空的（值变为 0 ）；
- 如果在 (0, 0) 和 (n - 1, n - 1) 之间不存在一条可经过的路径，则无法摘到任何一个樱桃。

## 题解
### dp
#### 错误的代码
```C++
class Solution {
public:
    int cherryPickup(vector<vector<int>>& grid) {
        // bfs x / dfs x / dp
/*
首先: A -> b -> A 等价于 A -> b 再一次 A -> b
定义 f[i][j][y][x] 为第一次走到[i][j]第二次走到[y][x]的最大樱桃数量
有  f[i][j] = max(f[i - 1][j], f[i][j - 1]) + g[i][j]
    f[y][x] = max(f[y + 1][x], f[y][x + 1]) + g[y][x]
    减去第一次经过时候的
    f[i][j][y][x] -= f[i][j][n - 1][n - 1]
*/
        const int n = grid.size();
        vector<vector<vector<vector<int>>>> f(n + 1, vector<vector<vector<int>>>(n + 1, vector<vector<int>>(n + 1, vector<int>(n + 1, 0))));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                for (int y = 0; y < n; ++y) {
                    for (int x = 0; x < n; ++x) {
                        if (grid[i][j] == -1 || grid[y][x] == -1) {
                            f[i + 1][j + 1][y + 1][x + 1] = INT_MIN;
                            continue;
                        }
                            
                        f[i + 1][j + 1][y + 1][x + 1] = max(
                            max(f[i + 1][j][y + 1][x], f[i + 1][j][y][x + 1]),
                            max(f[i][j + 1][y + 1][x], f[i][j + 1][y][x + 1])
                        ) + grid[i][j] + grid[y][x];

                        if (i == y && j == x)
                            f[i + 1][j + 1][y + 1][x + 1] -= grid[i][j];
                    }
                }
            }
        }

        return max(f[n][n][n][n], 0);
    }
};
```

#### 正确的
1. [动态规划-详细解析 + 图解](https://leetcode.cn/problems/cherry-pickup/solutions/97703/dong-tai-gui-hua-xiang-xi-jie-xi-tu-jie-by-newhar)

2. [教你一步步思考 DP：从记忆化搜索到递推到空间优化！（Python/Java/C++/Go）](https://leetcode.cn/problems/cherry-pickup/solutions/2766975/jiao-ni-yi-bu-bu-si-kao-dpcong-ji-yi-hua-ruue)

```C++
class Solution {
public:
    int cherryPickup(vector<vector<int>>& grid) {
        int n = grid.size();
        vector<vector<vector<int>>> f(n * 2 - 1, vector<vector<int>>(n + 1, vector<int>(n + 1, INT_MIN)));
        f[0][1][1] = grid[0][0];
        for (int t = 1; t < n * 2 - 1; t++) {
            for (int j = max(t - n + 1, 0); j <= min(t, n - 1); j++) {
                if (grid[t - j][j] < 0) continue;
                for (int k = j; k <= min(t, n - 1); k++) {
                    if (grid[t - k][k] < 0) continue;
                    f[t][j + 1][k + 1] = max({f[t - 1][j + 1][k + 1], f[t - 1][j + 1][k], f[t - 1][j][k + 1], f[t - 1][j][k]}) +
                                         grid[t - j][j] + (k != j ? grid[t - k][k] : 0);
                }
            }
        }
        return max(f[n * 2 - 2][n][n], 0);
    }
};
```


### 费用流