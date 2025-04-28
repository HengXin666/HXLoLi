# [蓝桥杯 2022 省 B] 统计子矩阵

链接: [[蓝桥杯 2022 省 B] 统计子矩阵](https://www.luogu.com.cn/problem/P8783)

## 题目描述

给定一个 $N \times M$ 的矩阵 $A$，请你统计有多少个子矩阵 (最小 $1 \times 1$, 最大 $N \times M)$ 满足子矩阵中所有数的和不超过给定的整数 $K$。

## 输入格式

第一行包含三个整数 $N, M$ 和 $K$。

之后 $N$ 行每行包含 $M$ 个整数, 代表矩阵 $A$。

## 输出格式

一个整数代表答案。

## 样例 #1

### 样例输入 #1

```
3 4 10
1 2 3 4
5 6 7 8
9 10 11 12
```

### 样例输出 #1

```
19
```

## 提示

**【样例说明】**

满足条件的子矩阵一共有 $19$，包含:

大小为 $1 \times 1$ 的有 $10$ 个。

大小为 $1 \times 2$ 的有 $3$ 个。 大小为 $1 \times 3$ 的有 $2$ 个。

大小为 $1 \times 4$ 的有 $1$ 个。

大小为 $2 \times 1$ 的有 $3$ 个。

**【评测用例规模与约定】**

对于 $30 \%$ 的数据, $N, M \leq 20$.

对于 $70 \%$ 的数据, $N, M \leq 100$.

对于 $100 \%$ 的数据, $1 \leq N, M \leq 500,0 \leq A_{i j} \leq 1000,1 \leq K \leq 2.5\times10^8$. 

蓝桥杯 2022 省赛 B 组 F 题。

# 题解
## # 1
尝试使用暴力, 除了样例其他全部wa

```C++
#include <cstdio>
#include <vector>

using namespace std;

int k;
long long res = 0;

int I, J; // 开始 

void dfs(int i, int j, int now_sum, const vector<vector<int>>& arr) {
    if (now_sum > k)
        return;
    
    ++res;
    
//    bool tag = 1;
//    if (i >= arr.size() || j >= arr[0].size())
//        tag = 0;
    
    if (j + 1 < arr[0].size()) {
        int add = 0;
        for (int y = I; y <= i; ++y)
            add += arr[y][j + 1];
        dfs(i, j + 1, now_sum + add, arr);
    }
    
    if (i + 1 < arr.size()) {
        int tmp = 0;
        for (int x = J; x <= j; ++x)
            tmp += arr[i + 1][x];
        dfs(i + 1, j, now_sum + tmp, arr);
//        add += tmp;
    }
        
    
//    if (tag) {
//        dfs(i + 1, j + 1, now_sum + add, arr);
//    }
}

int main() {
    int n, m;
    scanf("%d %d %d", &n, &m, &k);
    vector<vector<int>> arr(n, vector<int>(m));
//    vector<vector<int>> sum(n, vector<int>(m));
//    vector<vector<int>> dp(n + 1, vector<int>(m + 1, 0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            scanf("%d", &arr[i][j]);
//            sum[i][j] = arr[i][j];
        }
    }
        
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            I = i;
            J = j;
            dfs(i, j, arr[i][j], arr);
        }
    }
    
    printf("%lld\n", res);
    
    return 0;
} 
```

## # 2
看了题解后发现是二维前缀和, 然后我再暴力: 可以过 80% 但时间复杂度仍然是 $O(n*m*n*m)$

```C++
#include <cstdio>
#include <vector>

using namespace std;

int k;
long long res = 0;

int find(int i, int j, const vector<vector<int>>& arr, const vector<vector<long long>>& sum) {
    int res = 0;
    for (int y = 0; i + y < arr.size(); ++y) {
        for (int x = 0; j + x < arr[0].size(); ++x) {
            if (sum[i + y + 1][j + x + 1] - sum[i + y + 1][j] - sum[i][j + x + 1] + sum[i][j] <= k) {
                ++res;
            }
        }
    }
    return res;
}

int main() {
    int n, m;
    scanf("%d %d %d", &n, &m, &k);
    vector<vector<int>> arr(n, vector<int>(m));
    vector<vector<long long>> sum(n + 1, vector<long long>(m + 1));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            scanf("%d", &arr[i][j]);
            sum[i + 1][j + 1] = sum[i + 1][j] + sum[i][j + 1] - sum[i][j] + arr[i][j];
        }
    }
    
    // 双指针维护 
    // 单点大小:  =  sum[i + 1][j + 1] - sum[i + 1][j] - sum[i][j + 1] + sum[i][j]
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            res += find(i, j, arr, sum);
        }
    }
    
    printf("%lld\n", res);
    
    return 0;
} 
```

## 3 AC
需要使用双指针 实际上是两坐标(x1, y1)(x2, y2)指针, 来维护矩形的大小, 并且时刻保证 其 和是小于 k 的

还是不明白, 放弃了qwq...

```C++
#include <cstdio>
#include <vector>

using namespace std;

int k;
long long res = 0;

int main() {
    int n, m;
    scanf("%d %d %d", &n, &m, &k);
    vector<vector<int>> arr(n, vector<int>(m));
    vector<vector<long long>> sum(n + 1, vector<long long>(m + 1));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            scanf("%d", &arr[i][j]);
            sum[i + 1][j + 1] = sum[i + 1][j] + sum[i][j + 1] - sum[i][j] + arr[i][j];
        }
    }
    
    // 双指针维护 
    // 单点大小:  =  sum[i + 1][j + 1] - sum[i + 1][j] - sum[i][j + 1] + sum[i][j]
     for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
            for (int l = 0, r = 0; r < m; r++) {
                while (l <= r && sum[j + 1][r + 1] - sum[i][r + 1] - sum[j + 1][l] + sum[i][l] > k)
                    l++;
                res += r - l + 1; //注意细节要+1
            }
        }
    }
    
    printf("%lld\n", res);
    
    return 0;
} 
```
