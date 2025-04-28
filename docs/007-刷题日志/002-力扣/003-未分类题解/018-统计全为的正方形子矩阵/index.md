# 1277. 统计全为 1 的正方形子矩阵
原题: [1277. 统计全为 1 的正方形子矩阵](https://leetcode.cn/problems/count-square-submatrices-with-all-ones/description/)

中等

给你一个 m * n 的矩阵，矩阵中的元素不是 0 就是 1，请你统计并返回其中完全由 1 组成的 正方形 子矩阵的个数。

## 示例 1：

输入：matrix =
```
[
  [0,1,1,1],
  [1,1,1,1],
  [0,1,1,1]
]
```
输出：15<br>
解释： <br>
边长为 1 的正方形有 10 个。<br>
边长为 2 的正方形有 4 个。<br>
边长为 3 的正方形有 1 个。<br>
正方形的总数 = 10 + 4 + 1 = 15.<br>
## 示例 2：

输入：matrix = 
```
[
  [1,0,1],
  [1,1,0],
  [1,1,0]
]
```
输出：7<br>
解释：<br>
边长为 1 的正方形有 6 个。 <br>
边长为 2 的正方形有 1 个。<br>
正方形的总数 = 6 + 1 = 7.<br>

## 提示：

$1 <= arr.length <= 300$<br>
$ 1 <= arr[0].length <= 300$ <br>
$ 0 <= arr[i][j] <= 1$ <br>

# 代码
## 1) 蜜汁越界
(dsb, 越界1小时都看不出来吗?!)
```C++
class Solution {
public:
    int daAn[305] = {0};

    int fun(int n) {
        if (daAn[n])
            return daAn[n];

        int res = 0;
        for (int i = 1; i <= n; ++i) {
            res += i * i;
        }

        daAn[n] = res;
        return daAn[n];
    }

    int countSquares(vector<vector<int>>& matrix) {
        /*
        思路: 因为寻找出最大边长 => 其子边长的正方形个数
        所以, 我们只需要 找到所有不重合最大正方形即可 (思考 5 min)
        如何寻找 所有不重合最大正方形 ?
        遍历 即可, 发现不符合的就break 然后回溯到起点(之前的就缓存置零)
        */
        const int coSize = matrix.size();
        const int size = matrix[0].size();
        int res = 0;

        for (int i = 0; i < coSize; ++i) {
            for (int j = 0; j < size; ++j) {
                // 外层为回溯
                printf("\n[res:%d] %d %d --- | ", res, i, j);
                // 记录长度
                int len = 1;
                bool tag;
                for (; len < coSize; ++len) {
                    printf("|");
                    tag = 1;
                    for (int y = i; y < i + len && y < coSize && j + len - 1 < size; ++y) {
                        printf("[y](%d, %d)_<%d>", j + len - 1, y, matrix[y][j + len - 1]);
                        if (!matrix[j + len - 1][y])
                            goto END;
                        tag = 0;
                    }

                    for (int x = j; x < j + len && x < size && i + len - 1 < coSize; ++x) {
                        printf("[x](%d, %d)_<%d>", x, i + len - 1, matrix[i + len - 1][x]);
                        if (!matrix[x][i + len - 1])
                            goto END;
                        tag = 0;
                    }

                    if (tag)
                        break;
                }

                END:
                if (len == 1)
                    break;
                res += fun(len);
                printf("END(len = %d) [清空:(%d, %d) ~ (%d, %d)]\n", len, i, j
                , i + len , j + len);

                for (int i = 0; i < coSize; ++i) {
                    for (int j = 0; j < size; ++j) {
                        printf("%d ", matrix[i][j]);
                    }
                    printf("\n");
                }

                for (int y = i; y <= i + len; ++y) {
                    for (int x = j; x <= j + len; ++x) {
                        printf("[%d][%d]_", y, x);
                        matrix[y][x] = 0;
                    }
                    printf("\n");
                }

                j = j + len - 1; // 因为有 ++j

                for (int i = 0; i < coSize; ++i) {
                    for (int j = 0; j < size; ++j) {
                        printf("%d ", matrix[i][j]);
                    }
                    printf("\n");
                }
            }
        }

        printf("return == %d\n", res);
        return res;
    }
};
```

## 2) 重构但忽视了重要的细节
(5分钟重构, 就tm解决了?!)
```C++
class Solution {
public:
    int daAn[305] = {0};

    int fun(int n) {
        if (daAn[n])
            return daAn[n];

        int res = 0;
        for (int i = 1; i <= n; ++i) {
            res += i * i;
        }

        daAn[n] = res;
        return daAn[n];
    }

    int countSquares(vector<vector<int>>& matrix) {
        /*
        思路: 因为寻找出最大边长 => 其子边长的正方形个数
        所以, 我们只需要 找到所有不重合最大正方形即可 (思考 5 min)
        如何寻找 所有不重合最大正方形 ?
        遍历 即可, 发现不符合的就break 然后回溯到起点(之前的就缓存置零)
        */
        const int coSize = matrix.size();
        const int size = matrix[0].size();
        int res = 0;

        for (int i = 0; i < coSize; ++i)
        {
            for (int j = 0; j < size; ++j)
            {
                // 外层为回溯
                int len = 0;
                while (1) 
                {
                    bool tag = 0;
                    for (int y = i; y <= i + len && j + len < size; ++y)
                    {
                        if (!matrix[y][j + len])
                            goto END;
                        tag = 1;
                    }

                    if (!tag)
                        goto END;
                    tag = 0;

                    for (int x = j; x <= j + len && i + len < coSize; ++x)
                    {
                        if (!matrix[i + len][x])
                            goto END;
                        tag = 1;
                    }

                    if (!tag)
                        goto END;

                    ++len;
                    if (i + len >= coSize || j + len >= size)
                        goto END;
                }

                END:
                for (int y = i; y < i + len; ++y)
                    for (int x = j; x < j + len; ++x)
                        matrix[y][x] = 0;

                res += fun(len);
            }
        }

        return res;
    }
};
```

无法解决共用问题, 如
```FUCK
1,1,0
1,1,0
1,1,0   预期结果: 2 + 6 == 8
```

# 题解

[221. 最大正方形](../004-最大正方形/index.md) <-状态转移方程的证明-> [力扣官方题解: 统计全为 1 的正方形子矩阵](https://leetcode.cn/problems/count-square-submatrices-with-all-ones/solutions/101706/tong-ji-quan-wei-1-de-zheng-fang-xing-zi-ju-zhen-2)

写个 `221.` 题, 然后手画一个

```
1, 1, 1, 1, 1
1, 1, 1, 1, 1
1, 1, 0, 1, 1
1, 1, 0, 1, 1
```

按照dp写出dp矩阵(拜托这个应该是很特殊的了, 包含了很多情况的, 不然你就写个别的=-=)

```
1, 1, 1, 1, 1
1, 2, 2, 2, 2
1, 2, 0, 1, 2
1, 2, 0, 1, 2
```

你就会惊奇的发现 dp[i][j] 矩阵加和 等于 完全由 1 组成的 正方形 子矩阵的个数

```C++
class Solution {
public:
    int countSquares(vector<vector<int>>& matrix) {
        const int coSize = matrix.size(), size = matrix[0].size();

        int res = 0;
        for (int i = 1; i < coSize; ++i) {
            for (int j = 1; j < size; ++j) {
                if (matrix[i][j]) {
                    matrix[i][j] = min(min(matrix[i - 1][j], matrix[i][j - 1]), matrix[i - 1][j - 1]) + 1;
                    res += matrix[i][j];
                }
            }
        }

        for (int i = 0; i < coSize; ++i) {
            res += matrix[i][0];
        }

        for (int j = 1; j < size; ++j) {
            res += matrix[0][j];
        }

        return res;
    }
};
```