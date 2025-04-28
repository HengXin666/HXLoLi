# [NOIP2006 提高组] 金明的预算方案

## 题目描述

金明今天很开心，家里购置的新房就要领钥匙了，新房里有一间金明自己专用的很宽敞的房间。更让他高兴的是，妈妈昨天对他说：“你的房间需要购买哪些物品，怎么布置，你说了算，只要不超过 $n$ 元钱就行”。今天一早，金明就开始做预算了，他把想买的物品分为两类：主件与附件，附件是从属于某个主件的，下表就是一些主件与附件的例子：

| 主件 | 附件 |
| :----------: | :----------: |
| 电脑 | 打印机，扫描仪 |
| 书柜 | 图书 |
| 书桌 | 台灯，文具 |
| 工作椅 | 无 |

如果要买归类为附件的物品，必须先买该附件所属的主件。每个主件可以有 $0$ 个, $1$ 个或 $2$ 个附件。每个附件对应一个主件，附件不再有从属于自己的附件。金明想买的东西很多，肯定会超过妈妈限定的 $n$ 元。于是，他把每件物品规定了一个重要度，分为 $5$ 等：用整数 $1 \sim 5$ 表示，第 $5$ 等最重要。他还从因特网上查到了每件物品的价格（都是 $10$ 元的整数倍）。他希望在不超过 $n$ 元的前提下，使每件物品的价格与重要度的乘积的总和最大。

设第 $j$ 件物品的价格为 $v_j$，重要度为 $w_j$，共选中了 $k$ 件物品，编号依次为 $j_1,j_2,\dots,j_k$，则所求的总和为：

$v_{j_1} \times w_{j_1}+v_{j_2} \times w_{j_2}+ \dots +v_{j_k} \times w_{j_k}$。

请你帮助金明设计一个满足要求的购物单。

## 输入格式

第一行有两个整数，分别表示总钱数 $n$ 和希望购买的物品个数 $m$。

第 $2$ 到第 $(m + 1)$ 行，每行三个整数，第 $(i + 1)$ 行的整数 $v_i$，$p_i$，$q_i$ 分别表示第 $i$ 件物品的价格、重要度以及它对应的的主件。如果 $q_i=0$，表示该物品本身是主件。

## 输出格式

输出一行一个整数表示答案。

## 样例 #1

### 样例输入 #1

```
1000 5
800 2 0
400 5 1
300 5 1
400 3 0
500 2 0
```

### 样例输出 #1

```
2200
```

## 提示

#### 数据规模与约定

对于全部的测试点，保证 $1 \leq n \leq 3.2 \times 10^4$，$1 \leq m \leq 60$，$0 \leq v_i \leq 10^4$，$1 \leq p_i \leq 5$，$0 \leq q_i \leq m$，答案不超过 $2 \times 10^5$。

NOIP 2006 提高组 第二题

# 样例增量
> 注意本题很坑, 也没有描述好那个 $q_i$ 是什么. 所以我偷点样例来.

### [1]
```in
100 2
10 1 2
10 1 0
```
```out
20
```

### [2]
```in
4500 12
100 3 0
400 5 0
300 5 0
1400 2 0
500 2 0
800 2 4
1400 5 4
300 5 0
1400 3 8
500 2 0
1800 4 0
440 5 10
```

```out
16700
```

# 解题
## 尝试一

分析的状态是:

1. 不装
2. 装
    
    a. 是主件 / 已经放了主件, 直接加
    
    b. 不是主件 并且 主件没有放, 尝试加主件, 再放附件  

- 然而只有30分..., 因为缺少了如果把这个主件去除和附件去除的处理...

```C++
long long dp_14_BFS_tmp(int now_okane, int now_i, int* arr, long long* DP, int& n, int& m, int*& v, int*& w, int*& baba)
{
    // 退出条件
    if (now_okane <= 0 || now_i < 0)
        return 0;

    if (DP[now_okane])
        return DP[now_okane];


    int* tmp = new int[m];
    for (int i = 0; i < m; ++i)
    {
        tmp[i] = arr[i];
    }

    // 不买
    long long res = dp_14_BFS_tmp(now_okane, now_i - 1, tmp, DP, n, m, v, w, baba);

    if (baba[now_i] == 0)
    {
        // 为主件
        if (now_okane >= v[now_i])
        {
            res = getMax(res, dp_14_BFS_tmp(now_okane - v[now_i], now_i - 1, tmp, DP, n, m, v, w, baba) + v[now_i] * w[now_i]);
        }
    }
    else
    {
        // 为附件
        if (tmp[baba[now_i] - 1] == 0)
        {
            // 需要购买主件
            if (now_okane >= v[now_i] + v[baba[now_i] - 1])
            {
                tmp[baba[now_i] - 1] = 1;
                int k = res;
                res = getMax(res, dp_14_BFS_tmp(now_okane - v[now_i] - v[baba[now_i] - 1], now_i - 1, tmp, DP, n, m, v, w, baba) + v[now_i] * w[now_i] + v[baba[now_i] - 1] * w[baba[now_i] - 1]);
                if (k == res)
                    tmp[baba[now_i] - 1] = 0;
            }
        }
    }

    delete tmp;
    DP[now_okane] = res;
    return DP[now_okane];
}

void dp_14_BFS_DP(void)
{
    int n, m;
    scanf("%d %d", &n, &m);
    n /= 10;

    int* v = new int[m];
    int* w = new int[m];
    int* baba = new int[m];
    int* tmp = new int[m];
    long long* DP = new long long[n + 1];

    for (int i = 0; i <= n; ++i)
    {
        DP[i] = 0;
    }

    for (int i = 0; i < m; ++i)
    {
        tmp[i] = 0;
        scanf("%d %d %d", &v[i], &w[i], &baba[i]);
        v[i] /= 10;
    }

    // 没有问题, 但是缺少了一种情况: 如果我换新的, 是不是需要把主件标记去掉?! 显然这样的方法是去不掉的!
    printf("%d", dp_14_BFS_tmp(n, m - 1, tmp, DP, n, m, v, w, baba) * 10);
}
```
## 尝试二

看了题解后, 又来继续尝试...
因为是状态搞错了; `可以枚举所有可能为什么还需要去记忆那个主件放了没有呢?!` 是吧...


```C++
void dp_14_03(void)
{
    int n, m;
    scanf("%d %d", &n, &m);
    n /= 10;

    vector<vector<int>> w(1 + m, vector<int>(4));
    vector<vector<int>> v(1 + m, vector<int>(4));
    vector<long long> DP(n + 1);
    for (int i = 1; i <= m; ++i)
    {
        int V, W, p;
        scanf("%d %d %d", &V, &W, &p);
        V /= 10;
        if (p)
        {
            if (v[p][1] == 0)
            {
                v[p][1] = v[p][0] + V;
                w[p][1] = w[p][0] + W * V;
            }
            else
            {
                v[p][2] = v[p][0] + V;
                w[p][2] = w[p][0] + W * V;

                v[p][3] = v[p][1] + V;
                w[p][3] = w[p][1] + W * V;
            }
        }
        else
        {
            v[i][0] = V;
            w[i][0] = V * W;
        }
    }

    // dp
    for (int i = 1; i <= m; ++i)
    {
        if (v[i].empty())
            continue;

        for (int j = n; j > 0; --j)
        {
            if (j - v[i][0] >= 0)
            {
                DP[j] = max(DP[j], DP[j - v[i][0]] + w[i][0]);

                if (v[i][1] && j - v[i][1] >= 0)
                    DP[j] = max(DP[j], DP[j - v[i][1]] + w[i][1]);

                if (v[i][2] && j - v[i][2] >= 0)
                    DP[j] = max(DP[j], DP[j - v[i][2]] + w[i][2]);

                if (v[i][3] && j - v[i][3] >= 0)
                    DP[j] = max(DP[j], DP[j - v[i][3]] + w[i][3]);
            }
        }
    }

    printf("%d", DP[n] * 10);

    // 依旧有问题!艹, 不能保证那些异步的数, 所以还是要分开啊^[1]
    /** 如:
     100 2
     10 1 2
     10 1 0
     */
}
```
> 还是有坑..., 行, ...

修改为: 即可

```C++
#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

long long getMax(long long a, long long b)
{
    return a > b ? a : b;
}

int main(void)
{
    int n, m;
    scanf("%d %d", &n, &m);
    n /= 10;

    vector<vector<int>> w(1 + m, vector<int>(4));
    vector<vector<int>> v(1 + m, vector<int>(4));
    vector<long long> DP(n + 1);
    for (int i = 1; i <= m; ++i)
    {
        int V, W, p;
        scanf("%d %d %d", &V, &W, &p);
        V /= 10;
        if (p)
        {
            if (v[p][1] == 0)
            {
                v[p][1] = v[p][0] + V;
                w[p][1] = w[p][0] + W * V;
            }
            else
            {
                v[p][2] = v[p][0] + V;
                w[p][2] = w[p][0] + W * V;

                v[p][3] = v[p][1] + V;
                w[p][3] = w[p][1] + W * V;
            }
        }
        else
        {
            v[i][0] = V;
            w[i][0] = V * W;
            if (v[i][1] != 0)
            {
                v[i][1] += v[i][0];
                w[i][1] += w[i][0];
                if (v[i][2] != 0)
                {
                    v[i][3] += v[i][1] + v[i][2];
                    w[i][3] += w[i][1] + w[i][2];
                    
                    v[i][2] += v[i][0];
                    w[i][2] += w[i][0];
                }
            }
        }
    }

    // 初始化状态
    for (int i = 1; i <= m; ++i)
    {
        if (v[i].empty())
            continue;

        for (int j = n; j > 0; --j)
        {
            if (j - v[i][0] >= 0)
            {
                DP[j] = max(DP[j], DP[j - v[i][0]] + w[i][0]);

                if (v[i][1] && j - v[i][1] >= 0)
                    DP[j] = max(DP[j], DP[j - v[i][1]] + w[i][1]);

                if (v[i][2] && j - v[i][2] >= 0)
                    DP[j] = max(DP[j], DP[j - v[i][2]] + w[i][2]);

                if (v[i][3] && j - v[i][3] >= 0)
                    DP[j] = max(DP[j], DP[j - v[i][3]] + w[i][3]);
            }
        }
    }

    printf("%lld", DP[n] * 10);
    
    return 0;
}
```

# 题解

参考:

[Erutsiom 的博客 - 题解 P1064 【金明的预算方案】](https://www.luogu.com.cn/blog/moisture2333/solution-p1064)

> 从 ①选这个东西放包里 ②不要这个东西 变成了：
>
> ①不买主件 ②买主件 ③买主件+副件1 ④买主件+副件2 ⑤买主件+副件1+副件2