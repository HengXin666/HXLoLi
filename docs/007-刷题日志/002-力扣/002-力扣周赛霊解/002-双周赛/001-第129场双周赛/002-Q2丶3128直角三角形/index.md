# 3128. 直角三角形
链接: [3128. 直角三角形](https://leetcode.cn/problems/right-triangles/)

给你一个二维 boolean 矩阵 grid 。

请你返回使用 grid 中的 3 个元素可以构建的 直角三角形 数目，且满足 3 个元素值 都 为 1 。

注意：

如果 grid 中 3 个元素满足：一个元素与另一个元素在 同一行，同时与第三个元素在 同一列 ，那么这 3 个元素称为一个 直角三角形 。这 3 个元素互相之间不需要相邻。

# 题解
## 枚举中间 + 乘法原理
假设第 $i$ 行有 $rowSum$ 个 $1$, 第 $j$ 列有 $colSum$ 个 $1$。

根据乘法原理, 以 $(i, j)$ 为直角顶点的三角形个数为 $$(rowSum - 1) \times (colSum - 1)$$

即

```C++
class Solution {
public:
    long long numberOfRightTriangles(vector<vector<int>>& g) {
        long long res = 0;
        vector<int> y(g.size());
        vector<int> x(g[0].size());

        for (int i = 0; i < g.size(); ++i) {
            for (int j = 0; j < g[i].size(); ++j) {
                if (g[i][j]) {
                    ++y[i];
                    ++x[j];
                }
            }
        }
        
        for (int i = 0; i < g.size(); ++i) {
            for (int j = 0; j < g[i].size(); ++j) {
                if (g[i][j]) {
                    res += (y[i] - 1) * (x[j] - 1);
                }
            }
        }
        
        return res;
    }
};
```

当然你也可以把`-1`给提出来:

```C++
class Solution {
public:
    long long numberOfRightTriangles(vector<vector<int>>& grid) {
        int n = grid[0].size();
        vector<int> col_sum(n, -1); // 提前减一
        for (int j = 0; j < n; j++) {
            for (auto& row : grid) {
                col_sum[j] += row[j];
            }
        }

        long long ans = 0;
        for (auto& row : grid) {
            int row_sum = accumulate(row.begin(), row.end(), 0) - 1; // 提前减一
            for (int j = 0; j < row.size(); j++) {
                if (row[j] == 1) {
                    ans += row_sum * col_sum[j];
                }
            }
        }
        return ans;
    }
};

// By 0x3f
// https://leetcode.cn/problems/right-triangles/solutions/2758892/cheng-fa-yuan-li-pythonjavacgo-by-endles-7469
```
