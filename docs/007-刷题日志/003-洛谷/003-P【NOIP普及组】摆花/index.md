# [NOIP2012 普及组] 摆花

## 题目描述

小明的花店新开张，为了吸引顾客，他想在花店的门口摆上一排花，共 $m$ 盆。通过调查顾客的喜好，小明列出了顾客最喜欢的 $n$ 种花，从 $1$ 到 $n$ 标号。为了在门口展出更多种花，规定第 $i$ 种花不能超过 $a_i$ 盆，摆花时同一种花放在一起，且不同种类的花需按标号的从小到大的顺序依次摆列。

试编程计算，一共有多少种不同的摆花方案。

## 输入格式

第一行包含两个正整数 $n$ 和 $m$，中间用一个空格隔开。

第二行有 $n$ 个整数，每两个整数之间用一个空格隔开，依次表示 $a_1,a_2, \cdots ,a_n$。

## 输出格式

一个整数，表示有多少种方案。注意：因为方案数可能很多，请输出方案数对 $10^6+7$ 取模的结果。

## 样例 #1

### 样例输入 #1

```
2 4
3 2
```

### 样例输出 #1

```
2
```

## 提示

【数据范围】

对于 $20\%$ 数据，有 $0<n \le 8,0<m \le 8,0 \le a_i \le 8$。

对于 $50\%$ 数据，有 $0<n \le 20,0<m \le 20,0 \le a_i \le 20$。

对于 $100\%$ 数据，有 $0<n \le 100,0<m \le 100,0 \le a_i \le 100$。

NOIP 2012 普及组 第三题

# 作答

我的思路:
朴实的BFS, 当 余为 0 则返回1, 表示有一个方法
```markmap#h300
# 第 i 种, 剩余有 m 盆
## 放 a[i] 个,
### 余 m - a[i] 盆
#### 放 a[i - 1] 个
##### 余 m - a[i] - a[i - 1] 盆
#### 放 a[i - 1] - 1 个
##### 余 m - a[i] - (a[i - 1] - 1) 盆
#### 放 a[i - 1] - 2 个
##### 余 m - a[i] - (a[i - 1] - 2) 盆
#### ...
#### 放 a[i - 1] - a[i - 1]个
##### 余 m - a[i] - (a[i - 1] - a[i - 1]) 盆

## 放 a[i] - 1 个,
### 余 m - (a[i] - 1)盆
#### 放 a[i - 1] 个
##### 余 m - a[i - 1] - a[i - 1] 盆
#### 放 a[i - 1] - 1 个
##### 余 m - a[i - 1] - (a[i - 1] - 1) 盆
#### 放 a[i - 1] - 2 个
##### 余 m - a[i - 1] - (a[i - 1] - 2) 盆
#### ...
#### 放 a[i - 1] - a[i - 1]个
##### 余 m - a[i - 1] - (a[i - 1] - a[i - 1]) 盆

## 放 a[i] - 2 个
### 余 m - (a[i] - 2)盆
#### ...

## ...
### ...
#### ...

## 放 a[i] - a[i] 个
### 余 (m - a[i]) 盆
#### ...
```

具体代码如下:

```C++
#include <stdio.h>

#define DP_12_MOD ((int)1e6 + 7)

int dp_12_BFS(int i, int now_m, const int *arr)
{
    if (now_m == 0)
        return 1;

    if (i < 0)
        return 0;

    int res = 0;
    for (int number = 0; number <= arr[i]; ++number)
    {
        res = res + dp_12_BFS(i - 1, now_m - number, arr) % DP_12_MOD;
        if (now_m - number == 0)
            break;
    }

    return res;
}

void dp_12(void)
{
    int n, m;
    scanf("%d %d", &n, &m);
    int* arr = new int[n];
    for (int i = 0; i < n; ++i)
    {
        scanf("%d", &arr[i]);
    }
  
    // BFS 得了 30 分的暴力
    int max = dp_12_BFS(n - 1, m, arr);
    printf("%d", max);
}
```

以下是修改为记忆化搜索表的形式

```C++
#include <stdio.h>

#define DP_12_MOD ((int)1e6 + 7)

int dp_12_BFS_DP(int i, int now_m, const int* arr, int **DP)
{
    if (now_m <= 0)
        if (!now_m)
            return 1;
        else
            return 0;
    else
        if (i >= 0)
            if (DP[i][now_m])
                return DP[i][now_m];
            else
                ;
        else
            return 0;
        
    int res = 0;
    for (int number = 0; number <= arr[i]; ++number)
    {
        if (now_m - number < 0)
            break;
        else if (now_m - number == 0)
        {
            ++res;
            break;
        }
        else
            res += dp_12_BFS_DP(i - 1, now_m - number, arr, DP) % DP_12_MOD;
    }

    DP[i][now_m] = res;
    return DP[i][now_m];
}

void dp_12_02_DP(void)
{
    int n, m;
    scanf("%d %d", &n, &m);
    int* arr = new int[n];
    int** DP = new int*[n];
    for (int i = 0; i < n; ++i)
    {
        DP[i] = new int[m + 1];
        for (int j = 0; j <= m; ++j)
            DP[i][j] = 0;
        scanf("%d", &arr[i]);
    }
    // BFS 记忆化 还是30, 难道还有高手?
    // 分析了下载的测试用例: (就喵1秒)就发现:
    // 错误的, 忘记加 % DP_12_MOD 于最后那个了 DP[n - 1][m] 是唯一一个没有取余的, 艹 
    int max = dp_12_BFS_DP(n - 1, m, arr, DP) % DP_12_MOD;
    printf("%d", max);
}
```
<sub>居然忘记取余了, 最后一个...</sub>