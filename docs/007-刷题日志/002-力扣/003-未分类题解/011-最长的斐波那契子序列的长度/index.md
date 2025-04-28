# 873. 最长的斐波那契子序列的长度
原题链接: [873. 最长的斐波那契子序列的长度](https://leetcode.cn/problems/length-of-longest-fibonacci-subsequence/description/)

如果序列 $X_1, X_2, ..., X_n$ 满足下列条件，就说它是 斐波那契式 的：

- n >= 3
- 对于所有 `i + 2 <= n`，都有 $X_i + X_{i+1} = X_{i+2}$
给定一个严格递增的正整数数组形成序列 arr ，找到 arr 中最长的斐波那契式的子序列的长度。如果一个不存在，返回  0 。

（回想一下，子序列是从原序列 arr 中派生出来的，它从 arr 中删掉任意数量的元素（也可以不删），而不改变其余元素的顺序。例如， [3, 5, 8] 是 [3, 4, 5, 6, 7, 8] 的一个子序列）

 

## 示例 1：

输入: arr = [1,2,3,4,5,6,7,8]<br>
输出: 5<br>
解释: 最长的斐波那契式子序列为 [1,2,3,5,8] 。
## 示例 2：

输入: arr = [1,3,7,11,12,14,18]<br>
输出: 3<br>
解释: 最长的斐波那契式子序列有 [1,11,12]、[3,11,14] 以及 [7,11,18] 。
 

## 提示：

$3 <= arr.length <= 1000$<br>
$1 <= arr[i] < arr[i + 1] <= 10^9$

# 代码
## 尝试1, 纯状态转移

```C++
class Solution {
public:
    int lenLongestFibSubseq(vector<int>& arr) {
        int n = arr.size();
        
        vector<int> dp(n, 0);
        int res = 0;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < i; ++j) {
                for (int k = 0; k < j; ++k) {
                    if (arr[k] + arr[j] == arr[i]) {
                        if (dp[j] == 0) {
                            dp[j] = 2;
                            dp[k] = 1;
                        } else if (dp[k] == 0) {
                            dp[k] = 1;
                        }
                        dp[i] = max(dp[i], dp[k] + 2);
                    }
                }
            }

            if (dp[i] > res)
                res = dp[i];
        }

        return res;
    }
};
```

美好的想法, 但是总有例外的情况...

# 题解
## 暴力哈希

```C++
class Solution {
public:
    int lenLongestFibSubseq(vector<int>& arr) {
        int n = arr.size();
        unordered_set<int> a(arr.begin(), arr.end());
        // 思路: 斐波那契数列, 只要确定前面两个数, 那么后面的也确定了!
        int res = 0;
        for (int i = 1; i < n; ++i) {
            for (int j = 0; j < i; ++j) {
                // 枚举全部的前两个数的可能
                int x = arr[j];
                int y = arr[i];
                int now_len = 0;

                // 查找后面的
                while (a.find(x + y) != a.end())
                {
                    y = x + y;
                    x = y - x;
                    now_len += (!now_len ? 3 : 1);
                }

                if (now_len > res)
                    res = now_len;
            }
        }

        return res;
    }
};
```

## 动态规划
wdnmd的状态!

朴素动态规划 (削题解, 我怎么怎么知道怎么来的状态?)

不难想到使用动态规划求解，定义数组 dp[i][j] 表示**最后两个值**是 arr[i], arr[j] 的最长严格递增斐波那契子序列，那么状态转移方程为:

$dp[i][j]=dp[k][i]+1,\ (\ if\ k < 1) \&\& (arr[k] + arr[i] = arr[j])$

### 分析

当 i 确定, 任何 下标小于 i 的 j 都有可能 满足 arr[j] 是 斐波那契数列 arr[i] 的前一个数字.

因此只有当确定斐波那契子序列的最后两个数字时，才能确定整个斐波那契子序列。

#### 状态
故(二维dp), 定义 dp[j][i] 为 以 arr[j], arr[i] 为最后两个数字的斐波那契数列的最大长度.

其他的： [力扣官方题解](https://leetcode.cn/problems/length-of-longest-fibonacci-subsequence/solutions/1654336/zui-chang-de-fei-bo-na-qi-zi-xu-lie-de-c-8trz)

# 学到的
## 不能局限

## 状态依旧是唯一的方向

## 暴力