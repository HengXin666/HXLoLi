# 疯狂的采药

## 题目背景

此题为纪念 LiYuxiang 而生。

## 题目描述

LiYuxiang 是个天资聪颖的孩子，他的梦想是成为世界上最伟大的医师。为此，他想拜附近最有威望的医师为师。医师为了判断他的资质，给他出了一个难题。医师把他带到一个到处都是草药的山洞里对他说：“孩子，这个山洞里有一些不同种类的草药，采每一种都需要一些时间，每一种也有它自身的价值。我会给你一段时间，在这段时间里，你可以采到一些草药。如果你是一个聪明的孩子，你应该可以让采到的草药的总价值最大。”

如果你是 LiYuxiang，你能完成这个任务吗？

此题和原题的不同点：

$1$. 每种草药可以无限制地疯狂采摘。

$2$. 药的种类眼花缭乱，采药时间好长好长啊！师傅等得菊花都谢了！

## 输入格式

输入第一行有两个整数，分别代表总共能够用来采药的时间 $t$ 和代表山洞里的草药的数目 $m$。

第 $2$ 到第 $(m + 1)$ 行，每行两个整数，第 $(i + 1)$ 行的整数 $a_i, b_i$ 分别表示采摘第 $i$ 种草药的时间和该草药的价值。

## 输出格式

输出一行，这一行只包含一个整数，表示在规定的时间内，可以采到的草药的最大总价值。

## 样例 #1

### 样例输入 #1

```
70 3
71 100
69 1
1 2
```

### 样例输出 #1

```
140
```

## 提示

#### 数据规模与约定

- 对于 $30\%$ 的数据，保证 $m \le 10^3$ 。
- 对于 $100\%$ 的数据，保证 $1 \leq m \le 10^4$，$1 \leq t \leq 10^7$，且 $1 \leq m \times t \leq 10^7$，$1 \leq a_i, b_i \leq 10^4$。

# 作答:

## 贪心
> [!TIP]
> 实际上这个是错误的, 我只是尝试使用贪心, 然后按 $价值 / 时间$ 的方式进行排序, 优先把大的全部装, 然后继续尝试
> > 意外的得到了 90 分, 注意, 下面的代码还没有开 ll

```C++
#include <stdio.h>
#include <list>

using namespace std;

typedef struct Dp_10
{
    int time;
    int okane;
    double so;
} Dp_10;

int dp_10_DP(int now_t, list<Dp_10>::iterator mono_id, list<Dp_10>& L)
{
    int res = 0;
    while (mono_id != L.end())
    {
        while (now_t >= mono_id->time)
        {
            now_t -= mono_id->time;
            res += mono_id->okane;
        }
        ++mono_id;
    }

    return res;
}

bool dp_10_sort(Dp_10 a, Dp_10 b)
{
    return a.so > b.so ? true : false;
}

int main(void)
{
    int t, m;
    list<Dp_10> L;
    scanf("%d %d", &t, &m);
    
    for (int i = 0; i < m; ++i)
    {
        Dp_10 s;
        scanf("%d %d", &s.time, &s.okane);
        //  钱 / 时间
        s.so = 1.0 * s.okane / s.time;
        L.push_front(s);
    }
    L.sort(dp_10_sort);
    
    // 先进行贪心, 按 t/m 大小排列, 然后就是 最多 第一个
    int max = dp_10_DP(t, L.begin(), L);
    printf("%d", max);
    return 0;
}
```

## 正解: 完全背包 + 状态压缩 の DP

没什么好说的, 具体细节见 [<动态规划>[题型]01背包](../../../001-计佬常識/001-数据结构与算法/007-【算法】动态规划/002-【题型】背包问题/index.md)

```C++
#include <stdio.h>

long long getMax(long long a, long long b)
{
    return a > b ? a : b;
}

int main(void)
{
    int t, m;
    scanf("%d %d", &t, &m);
    int* arr_time = new int[m];
    int* arr_money = new int[m];
    long long* DP = new long long[t + 1];   // 代表容量(时间t
    for (int i = 0; i < m; ++i)
    {
        scanf("%d %d", &arr_time[i], &arr_money[i]);
    }

    // DP 状态初始化
    DP[0] = 0;

    // 状态转移方程 DP[i][j] = max{DP[i - 1][j], DP[i][j - t[i]] + m[i]};
    for (int i = 0; i < m; ++i)
    {
        for (int j = 1; j <= t; ++j)
        {
            DP[j] = j - arr_time[i] < 0 ? DP[j] : getMax(DP[j], DP[j - arr_time[i]] + arr_money[i]);
        }
    }

    printf("%lld", DP[t]);
    return 0;
}
```