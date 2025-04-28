# [NOIP2000 提高组] 方格取数
原题: [P1004 [NOIP2000 提高组] 方格取数](https://www.luogu.com.cn/problem/P1004)
## 题目背景

NOIP 2000 提高组 T4

## 题目描述

设有 $N \times N$ 的方格图 $(N \le 9)$，我们将其中的某些方格中填入正整数，而其他的方格中则放入数字 $0$。如下图所示（见样例）:

![](https://cdn.luogu.com.cn/upload/image_hosting/0bpummja.png)

某人从图的左上角的 $A$ 点出发，可以向下行走，也可以向右走，直到到达右下角的 $B$ 点。在走过的路上，他可以取走方格中的数（取走后的方格中将变为数字 $0$）。  
此人从 $A$ 点到 $B$ 点共走两次，试找出 $2$ 条这样的路径，使得取得的数之和为最大。

## 输入格式

输入的第一行为一个整数 $N$（表示 $N \times N$ 的方格图），接下来的每行有三个整数，前两个表示位置，第三个数为该位置上所放的数。一行单独的 $0$ 表示输入结束。

## 输出格式

只需输出一个整数，表示 $2$ 条路径上取得的最大的和。

## 样例 #1

### 样例输入 #1

```
8
2 3 13
2 6  6
3 5  7
4 4 14
5 2 21
5 6  4
6 3 15
7 2 14
0 0  0
```

### 样例输出 #1

```
67
```

## 提示

数据范围： $1\le N\le 9$。


## 代码

注意**不能**这样定义状态: dp[i][j][k] 表示 第k次 走到 map[i][j] 的最大加和

```C++
#include <cstdio>
#include <vector>

using namespace std;

int main() {
    int n;
    scanf("%d", &n);
    vector<vector<int>> map(n + 1, vector<int>(n + 1, 0));

    for (int a, b, c; ;) {
        scanf("%d%d%d", &a, &b, &c);
        if (a == b && b == c && c == 0)
            break;
        map[a][b] = max(c, map[a][b]);
    }

    // 从(1, 1)出发, 可以向下, 也可以向右, 走到 (n, n), 求路上的加和最大值
    // 定义 dp[i][j][k][l] 为 第1次 走到 map[i][j] 和 第2次走到 map[k][l] 的最大权值
    // 状态转移方程呢? 要么下下/下右/右下/右右
    // 对于 dp[i - 1][j][k][l] 这种你是不是sabi?! 对于 (i, j) (k, l) 的状态怎么会是 (i - 1, j) (k, l) 转移过来的呢? (k, l) 都还不存在!
    vector<vector<vector<vector<int>>>> dp(n + 1, vector<vector<vector<int>>>(n + 1, vector<vector<int>>(n + 1, vector<int>(n + 1))));

    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= n; ++j) {
            for (int k = 1; k <= n; ++k) {
                for (int l = 1; l <= n; ++l) {
                    dp[i][j][k][l] = max(
                        max(dp[i - 1][j][k - 1][l], dp[i - 1][j][k][l - 1]),
                        max(dp[i][j - 1][k - 1][l], dp[i][j - 1][k][l - 1])
                    ) + map[i][j] + map[k][l];

                    if (i == k && j == l)
                        dp[i][j][k][l] -= map[k][l]; // 重合了, 减去
                }
            }
        }
    }

    printf("%d\n", dp[n][n][n][n]);
    return 0;
}
```
