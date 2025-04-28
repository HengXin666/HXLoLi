# 3122. 使矩阵满足条件的最少操作次数
链接: [3122. 使矩阵满足条件的最少操作次数](https://leetcode.cn/problems/minimum-number-of-operations-to-satisfy-conditions/)

给你一个大小为`m x n`的二维矩形`grid`。每次 **操作** 中，你可以将 **任一** 格子的值修改为 **任意** 非负整数。完成所有操作后，你需要确保每个格子`grid[i][j]`的值满足：

- 如果下面相邻格子存在的话，它们的值相等，也就是`grid[i][j] == grid[i + 1][j]`（如果存在）。
- 如果右边相邻格子存在的话，它们的值不相等，也就是`grid[i][j] != grid[i][j + 1]`（如果存在）。

请你返回需要的 **最少** 操作数目。

## 示例:
输入：grid = [[1,1,1],[0,0,0]]

输出：3

解释：

将矩阵变成 [[1,0,1],[1,0,1]] ，它满足所有要求，需要 3 次操作：

- 将 grid[1][0] 变为 1 。
- 将 grid[0][1] 变为 0 。
- 将 grid[1][2] 变为 1 。

# 题解
## [错误]: 模拟 + 贪心

```C++
class Solution {
public:
    int minimumOperations(vector<vector<int>>& grid) {
        // grid[i][j] == grid[i + 1][j]
        // grid[i][j] != grid[i][j + 1]
        // 最少操作次数
        int res = 0;
        int zuo = 0;
        for (int j = 0; j < grid[0].size(); ++j) { // g[?][j]
            map<int, int> js; // 计数
            for (int i = 0; i < grid.size(); ++i) { // g[i][j]
                ++js[grid[i][j]];
            }
            
            // 选择最多且和左边不一样的
            auto it = js.rbegin();
            if (j == 0 || it->first != zuo) {
                res += grid.size() - it->second;
            } else {
                ++it;
                res += grid.size() - it->second;
            }
            zuo = it->first;
        }
        
        return res;
    }
};
```

## 记忆化搜索
遇事不决, 直接dp! 这个都想不到就给我刷100道dp!!

贪心你需要证明它数学上的正确性, 但是动态规划不需要, 因为它就是一个暴力枚举的过程, 只不过我们把它记忆化了! (然后就可以递归转递推)

反向思考:

$$最少的操作数 = m \times n - 最大保留不变元素个数$$

使用「枚举选哪个」的思想, DFS, 然后再转化为记忆化(显然有重叠子问题)

```C++
class Solution {
public:
    int minimumOperations(vector<vector<int>>& grid) {
/*
最少的操作数 = m * n - 保留原来
定义 f[i][j] 为使得前 i 列 且 第i列是j的 符合条件的保留的最多数数
有 f[i][j] = max(j in) f[i - 1][非j] + 非j数
*/
        vector<vector<int>> f(grid[0].size() + 1, vector<int>(11));

        vector<vector<int>> gNumArr(grid[0].size() + 1, vector<int>(11));

        for (int i = 0; i < grid.size(); ++i)
            for (int j = 0; j < grid[0].size(); ++j)
                ++gNumArr[j][grid[i][j]];

        function<int(int, int)> dfs = [&](int i, int pa_num) {
            if (i < 0)
                return 0;
            
            if (f[i][pa_num])
                return f[i][pa_num];

            for (int j = 0; j < 10; ++j) {
                if (j == pa_num)
                    continue;
                
                f[i][pa_num] = max(
                    f[i][pa_num],
                    dfs(i - 1, j) + gNumArr[i][j]
                );
            }

            return f[i][pa_num];
        };

        // xbzz, 把 行和列搞反了...
        return grid.size() * grid[0].size() - dfs(grid[0].size(), 10);
    }
};
```

## 递推 + 优化

```C++
class Solution {
public:
    int minimumOperations(vector<vector<int>>& grid) {
        int m = grid.size(), n = grid[0].size();
        int f0 = 0, f1 = 0, pre = -1;
        for (int i = 0; i < n; i++) {
            int cnt[10]{};
            for (auto& row : grid) {
                cnt[row[i]]++;
            }
            int mx = -1, mx2 = 0, x = -1;
            for (int v = 0; v < 10; v++) {
                int res = (v != pre ? f0 : f1) + cnt[v]; // 保留元素 v
                if (res > mx) { // 更新最优解和次优解
                    mx2 = mx;
                    mx = res;
                    x = v;
                } else if (res > mx2) { // 更新次优解
                    mx2 = res;
                }
            }
            f0 = mx;
            f1 = mx2;
            pre = x;
        }
        return m * n - f0;
    }
};
// 来自: 灵茶山艾府
```
