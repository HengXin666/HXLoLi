# 同余定理
> 若 $$ a\mod k = b \mod k $$ 则 $$|a - b| \mod k = 0 $$

- 直接看: [分享丨模运算的世界: 当加减乘除遇上取模（模运算恒等式/费马小定理/组合数）](https://leetcode.cn/circle/discuss/mDfnkW/)

## 例题
链接: [P8649 [蓝桥杯 2017 省 B] k 倍区间](https://www.luogu.com.cn/problem/P8649)

# [蓝桥杯 2017 省 B] k 倍区间

> [!TIP]
> 本题的前置题目: [523. 连续的子数组和](https://leetcode.cn/problems/continuous-subarray-sum/) 前缀和 + 维护左、枚举右

## 题目描述

给定一个长度为 $N$ 的数列，$A_1,A_2, \cdots A_N$，如果其中一段连续的子序列 $A_i,A_{i+1}, \cdots A_j(i \le j)$ 之和是 $K$ 的倍数，我们就称这个区间 $[i,j]$ 是 $K$ 倍区间。

你能求出数列中总共有多少个 $K$ 倍区间吗？

## 输入格式

第一行包含两个整数 $N$ 和 $K$ $(1 \le N,K \le 10^5)$。

以下 $N$ 行每行包含一个整数 $A_i$ $(1 \le A_i \le 10^5)$。

## 输出格式

输出一个整数，代表 $K$ 倍区间的数目。

## 样例 #1

### 样例输入 #1

```
5 2
1  
2  
3  
4  
5
```

### 样例输出 #1

```
6
```

## 提示

时限 2 秒, 256M。蓝桥杯 2017 年第八届

# 代码
## wa
前缀和, 后朴素遍历 $O(n^2)$

```C++
#include <cstdio>
#include <vector>

using namespace std;

int main() {
    int n, k;
    scanf("%d %d", &n, &k);
    
    vector<int> arr(n);
    vector<int> sumArr(n + 1);
    for (int i = 0; i < n; ++i) {
        scanf("%d", &arr[i]);
        sumArr[i + 1] = arr[i] + sumArr[i];
    }
    
    int res = 0;
    for (int i = 1; i <= n; ++i) {
        for (int j = i; j <= n; ++j) {
            if ((sumArr[j] - sumArr[i - 1]) % k == 0)
                ++res;
        }
    }
    
    printf("%d\n", res);
    
    return 0;
}
```

## 正解
由`同余定理`得: **余数为 $i$ 的有 $arr[i]$ 个**, 那么他们可以组成区间数量为 $C^2_{arr[i]}$ (无需考虑边算边更新, 因为我们可以看成区间都是相同的元素(组合数学))

- 为什么前缀和不做差得 $[i, j]$ 区间, 而是直接可以使用 $[0, i]$ 区间 ?!
    - ok, 摊牌了, 世界未解之谜tmd

- 为什么需要`arr[0] = 1;`预处理?
    - 如果不预处理, 对于输入 [1, 1, 1], k = 2
    - 那么显然有`arr[0] = 1`, `arr[1] = 2`, 然后`res = 2`才是正解
    - $C^2_{arr[1]}=\frac{(arr[1])(arr[1] - 1)}{2}=1$ 没问题
    - 但是因为 $C^2_{arr[0]}=\frac{(arr[0])(arr[0] - 1)}{2}=\frac{1 \times (1-1)}{2}=0$ 所以需要进行初始化: `arr[0] = 1`

~~不开`long long`见祖宗~~
```C++
#include <cstdio>
#include <vector>

using namespace std;

int main() {
    int n, k;
    scanf("%d %d", &n, &k);
    
    vector<long long> arr(n);
    arr[0] = 1;
    long long sum = 0; 
    for (int i = 0; i < n; ++i) {
        long long j;
        scanf("%lld", &j);
        sum = (sum + j) % k;
        ++arr[sum];
    }
    
    long long res = 0;
    for (int i = 0; i < k; ++i) {
        res += (arr[i] * (arr[i] - 1)) / 2;
    }
    
    printf("%lld\n", res);
    
    return 0;
}
```