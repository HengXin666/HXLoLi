# 二维前缀和

- [【图解】一张图秒懂二维前缀和！（Python/Java/C++/Go/JS/Rust）](https://leetcode.cn/problems/range-sum-query-2d-immutable/solutions/2667331/tu-jie-yi-zhang-tu-miao-dong-er-wei-qian-84qp/)

```C++
class NumMatrix {
    vector<vector<int>> sum;
public:
    NumMatrix(vector<vector<int>> &matrix) {
        int m = matrix.size(), n = matrix[0].size();
        sum.resize(m + 1, vector<int>(n + 1));
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                sum[i + 1][j + 1] = sum[i + 1][j] + sum[i][j + 1] - sum[i][j] + matrix[i][j];
            }
        }
    }

    // 返回左上角在 (r1,c1) 右下角在 (r2,c2) 的子矩阵元素和
    int sumRegion(int r1, int c1, int r2, int c2) {
        return sum[r2 + 1][c2 + 1] - sum[r2 + 1][c1] - sum[r1][c2 + 1] + sum[r1][c1];
    }
};
```

> 求和 $mat[i][j]$ ~ $mat[y][x]$, 就是 $sum[y + 1][x + 1] - sum[y + 1][j] - sum[i][x + 1] + sum[i][j]$ (任何时候, 大的坐标+1)