# 对角线枚举

- 例题: [3446. 按对角线进行矩阵排序](https://leetcode.cn/problems/sort-matrix-by-diagonals/) | 详细推导: [【模板】遍历对角线，附相似题目（Python/Java/C++/Go）](https://leetcode.cn/problems/sort-matrix-by-diagonals/solutions/3068709/mo-ban-mei-ju-dui-jiao-xian-pythonjavacg-pjxp/)

给你一个 $n \times m$ 的矩阵, 请对对角线排序.

```C++
class Solution {
public:
    vector<vector<int>> sortMatrix(vector<vector<int>>& grid) {
        // 对角线遍历
        // 怎么确定对角线呢? -> 他们 i - j 相同, 即 斜率是 -1
        // 设最右上的对角线是 k = 1
        // 则最左下的对角线是 k = n + m - 1
        // 有 k = i - j + m
        int n = grid.size(), m = grid[0].size();
        for (int k = 1; k < n + m; ++k) {
            // 确定 j 的范围 (从 i = 0, i = n - 1推导)
            // 注意 j 需要满足在 [0, m - 1] 中
            int j_min = max(0, m - k),             // i = 0
                j_max = min(m - 1, n - 1 - k + m); // i = n - 1
            vector<int> tmp;
            // 枚举对角线 已经 j 可以反推 i
            for (int j = j_min; j <= j_max; ++j)
                tmp.push_back(grid[j + k - m][j]);
            if (k < m) {
                sort(tmp.begin(), tmp.end(), [&](int a, int b) {
                    return a < b;
                });
            } else {
                sort(tmp.begin(), tmp.end(), [&](int a, int b) {
                    return a > b;
                });
            }
            for (int j = j_min, idx = 0; j <= j_max; ++j)
                grid[j + k - m][j] = tmp[idx++];
        }
        return grid;
    }
};
```

- 小练习: [2711. 对角线上不同值的数量差](https://leetcode.cn/problems/difference-of-number-of-distinct-values-on-diagonals/)

```C++
class Solution {
    using ll = uint64_t;
public:
    vector<vector<int>> differenceOfDistinctValues(
        vector<vector<int>>& grid
    ) {
        int n = grid.size(), m = grid[0].size();
        // k = i - j + m;
        for (int k = 1; k <= n + m - 1; ++k) {
            int min_j = max(0, m - k),             // i = 0
                max_j = min(m - 1, n - 1 - k + m); // i = n - 1
            int len = max_j - min_j + 1;
            vector<ll> usiro(len + 1);
            for (int j = max_j, idx = len - 1; j >= min_j; --j, --idx) {
                usiro[idx] = usiro[idx + 1] | 1ULL << grid[j + k - m][j];
            }
            ll mae = 0;
            for (int j = min_j, idx = 0; j <= max_j; ++j, ++idx) {
                int v = grid[j + k - m][j];
                grid[j + k - m][j] = abs(
                    popcount(mae) - popcount(usiro[idx + 1])
                );
                mae |= 1ULL << v;
            }
        }
        return grid;
    }
};
```
