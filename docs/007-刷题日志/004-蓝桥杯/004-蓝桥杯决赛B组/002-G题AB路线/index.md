# [蓝桥杯 2023 国 B] AB 路线

链接: [#7. AB 路线（编程题）](https://dashoj.com/d/lqbproblemG/p/7) / [P9425 [蓝桥杯 2023 国 B] AB 路线](https://www.luogu.com.cn/problem/P9425)

## 题目描述

有一个由 $N \times M$ 个方格组成的迷宫，每个方格写有一个字母 `A` 或者 `B`。小蓝站在迷宫左上角的方格，目标是走到右下角的方格。他每一步可以移动到上下左右相邻的方格去。

由于特殊的原因，小蓝的路线必须先走 $K$ 个 `A` 格子、再走 $K$ 个 `B` 格子、再走 $K$ 个 `A` 格子、再走 $K$ 个 `B` 格子……如此反复交替。

请你计算小蓝最少需要走多少步，才能到达右下角方格？

注意路线经过的格子数不必一定是 $K$ 的倍数，即最后一段 `A` 或 `B` 的格子可以不满 $K$ 个。起点保证是 `A` 格子。

例如 $K = 3$ 时，以下 $3$ 种路线是合法的：

```plain
AA
AAAB
AAABBBAAABBB
```

以下 $3$ 种路线不合法：

```plain
ABABAB
ABBBAAABBB
AAABBBBBBAAA
```

## 输入格式

第一行包含三个整数 $N$, $M$ 和 $K$。

以下 $N$ 行，每行包含 $M$ 个字符（`A` 或 `B`），代表格子类型。

## 输出格式

一个整数，代表最少步数。如果无法到达右下角，输出 $-1$。

## 样例 #1

### 样例输入 #1

```
4 4 2
AAAB
ABAB
BBAB
BAAA
```

### 样例输出 #1

```
8
```

## 提示

### 样例说明

每一步方向如下：下右下右上右下下；路线序列:`AABBAABBA`。

### 评测用例规模与约定

 - 对于 $20\%$ 的数据, $1 \le N, M \le 4$。
 - 对于另 $20\%$ 的数据, $K = 1$。
 - 对于 $100\%$ 的数据, $1 \le N, M \le 1000$，$1 \le K \le 10$。
 
第十四届蓝桥杯大赛软件赛决赛 C/C++ 大学 B 组 G 题

# 题解
## BFS + 分层图

- 学习分层图: [[蓝桥杯]真题讲解：AB路线（BFS+分层图）](https://www.bilibili.com/video/BV11f42127Lf/)

为什么有 $k$ 层?

- 先看 $vis[i][j][k]$ 的定义: 小人走到 $[i][j]$ 格子, 并且是还需要走 $k$ 个格子才可以 换格子种类. (从这个定义也可以看出来, 为什么`vis[ny][nx][dk] = true`这样更新(即 为什么是更新相邻的 (dk 和 dk - 1)), 因为走格子就是按顺序的嘛)

```C++
#include <cstdio>
#include <functional>
#include <vector>
#include <queue>
#include <tuple>

using namespace std;

const int fx[4][2] = {
    {-1, 0}, {0, -1}, {1, 0}, {0, 1}
};

int main() {
    int n, m, k;
    scanf("%d %d %d", &n, &m, &k);
    vector<vector<char>> tiz(n, vector<char>(m));
    // 分层图, 标记 vis[i][j][k] 为已走过的
    vector<vector<vector<char>>> vis(n, vector<vector<char>>(m, vector<char>(k))); // The vis[i][j][k]
    for (int i = 0; i < n; ++i) {
        char str[m + 1];
        scanf("%s", str);
        for (int j = 0; j < m; ++j) {
            tiz[i][j] = str[j];
        }
    }
    // bfs
    vis[0][0][k - 1] = 1; // 还不知道为什么不写也可以过(可能是因为回不来吧?)
    // 队列元组: 上一次访问的 [y][x], 已经走了 dk 个 A或B, 总路程 mo
    queue<tuple<int, int, int, int>> pq;
    pq.push(make_tuple(0, 0, k, 0)); 
    
    while (pq.size()) {
        int i, j, dk, mo;
        tie(i, j, dk, mo) = pq.front();
        pq.pop();
        
        if (i == n - 1 && j == m - 1) { // 结束(最先到达必定是最小的)
            printf("%d\n", mo);
            return 0;
        }
        --dk, ++mo;
        for (int z = 0; z < 4; ++z) {
            int ny = i + fx[z][0], nx = j + fx[z][1];
            if (ny < 0 || nx < 0 || ny >= n || nx >= m || vis[ny][nx][dk])
                continue;
            
            // 判断 是接着走 A/B 还是换着走 (通过 dk 是否为 0 判断)
            // tiz 是 图, 判断当前走的是否符合和上一次走的
            if (dk && tiz[i][j] == tiz[ny][nx]) {
                vis[ny][nx][dk] = true;
                pq.push(make_tuple(ny, nx, dk, mo)); 
            }
            else if (!dk && tiz[i][j] != tiz[ny][nx]) {
                vis[ny][nx][dk] = true;
                pq.push(make_tuple(ny, nx, k, mo));
            }
        }
    }
    
    printf("-1\n"); // 没有路径
    return 0;
}
```
