# [蓝桥杯 2023 省 B] 接龙数列

地址: [P9242 [蓝桥杯 2023 省 B] 接龙数列](https://www.luogu.com.cn/problem/P9242)

## 题目描述

对于一个长度为 $K$ 的整数数列：$A_{1},A_{2},\ldots,A_{K}$，我们称之为接龙数列当且仅当 $A_{i}$ 的首位数字恰好等于 $A_{i-1}$ 的末位数字（ $2 \leq i \leq K$ ）。

例如 $12,23,35,56,61,11$ 是接龙数列； $12,23,34,56$ 不是接龙数列，因为 $56$ 的首位数字不等于 $34$ 的末位数字。所有长度为 $1$ 的整数数列都是接龙数列。

现在给定一个长度为 $N$ 的数列 $A_{1},A_{2},\ldots,A_{N}$，请你计算最少从中删除多少 个数，可以使剩下的序列是接龙序列？

## 输入格式

第一行包含一个整数 $N$。

第二行包含 $N$ 个整数 $A_{1},A_{2},\ldots,A_{N}$。

## 输出格式

一个整数代表答案。

## 样例 #1

### 样例输入 #1

```
5
11 121 22 12 2023
```

### 样例输出 #1

```
1
```

## 提示

**【样例说明】**

删除 $22$，剩余 $11,121,12,2023$ 是接龙数列。 

**【评测用例规模与约定】**

对于 $20 \%$ 的数据， $1 \leq N \leq 20$。

对于 $50 \%$ 的数据， $1 \leq N \leq 10^4$。

对于 $100 \%$ 的数据， $1 \leq N \leq 10^{5}$，$1 \leq A_{i} \leq 10^{9}$。所有 $A_{i}$ 保证不包含前导 0。

蓝桥杯 2023 省赛 B 组 E 题。

# 题解
本题让我们求最少删除的数的个数，我们实际上可以将其转化为求该数列里的最长接龙子序列，再用 $n$ 减去该值即可。

> [!TIP]
> <b style="color:yellow">学到了! 如果求最少删除个数, 可以等价于 总长度 - 满足条件的最长子序列的长度</b>

设 $dp_{i, j}$ 表示到下标 $i$  时以 $j$ 结尾的最长接龙子序列，设 $b_{i}$ 为 $a_{i}$ 的首位， $e_{i}$ 为 $a_{i}$ 的末位，则我们可以将其分为两种情况讨论:
- 当 $e_{i}$ ≠ $j$  时，$a_{i}$ 对 $dp_{i, j}$ 没有任何贡献，故 $dp_{i, j}$ 和 $dp_{i-1, j}$ 保持一致。
- 当 $e_{i}=j$ 时， $dp_{i, j}$ 可能与 $dp_{i-1, j}$ 一致，也可能变为 $dp_{i-1, b_{i}}+1$ ，故 $dp_{i, j}$ 为 $dp_{i-1, j}$ 和 $dp_{i-1, b_{i}}+1$ 中的较大值。

综上所述，我们就得到了状态转移方程:

$
dp_{i, j}=\left\{\begin{array}{ll}
dp_{i-1, j}, & e_{i} ≠ j \\
\max \left(d p_{i-1, j}, dp_{i-1, b_{i}}+1\right), &e_{i}=j
\end{array}\right.
$

时间复杂度为 $O(n)$ ，可以通过。（一个优化：我们注意到 $dp_{i, j}$ 只和前一阶段的结果有关，因此可以用滚动数组优化 $dp$，空间复杂度变为 $O(1)$ 。)

代码

```C++
#include <iostream>
#include <string>
using namespace std;

int n,dp[10],maxn;
string a; // 为了方便取头尾，可以以字符串形式存储

signed main(){
    cin >> n;
    for(int i = 1; i <= n; i++) {
        cin >> a;
        int ln = a.length();
        dp[a[ln-1] - '0'] = max(dp[a[ln-1] - '0'], dp[a[0] - '0'] + 1);
    }
    
    for(int i = 0; i <= 9; i++) 
        maxn = max(maxn,dp[i]);
        
    cout << n - maxn;
    return 0;
}
```

## 时隔多天, 我自己写的

```C++
#include <iostream>
#include <string>
#include <vector>

using namespace std;

int main() {
    // 最小删除多少个数  使得接龙数列
    // 最小删除多少个数 == 原长 - 最长接龙数列 
    // 定义 dp[i][j] 是 i 开头 j 结尾的最长接龙序列长度
    // 对于 a---b, b --- c 字符串
    // 有 dp[a][c] = max( 自己, dp[a][b] + 1 ) 
    // 现在 把目光放到 b---c 上, 则 a 实际上是未知(任意的), 故有如下代码

    vector<vector<int>> dp(10, vector<int>(10, 0));
    int n;
    cin >> n;
    int len = 0;
    for (int i = 0; i < n; ++i) {
        string s;
        cin >> s;
        for (int j = 0; j < 10; ++j) {
            dp[j][s[s.size() - 1] - '0'] = max(dp[j][s[s.size() - 1] - '0'], dp[j][s[0] - '0'] + 1);
            len = max(len, dp[j][s[s.size() - 1] - '0']);
        }
    }
    
    cout << n - len << "\n";
    
    return 0;
}
```