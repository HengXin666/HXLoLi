# 1043. 分隔数组以得到最大和
原题地址: [1043. 分隔数组以得到最大和](https://leetcode.cn/problems/partition-array-for-maximum-sum/description/)

给你一个整数数组 `arr`，请你将该数组分隔为长度 **最多** 为 `k` 的一些（连续）子数组。分隔完成后，每个子数组的中的所有值都会变为该子数组中的**最大值**。

返回将数组分隔变换后能够得到的元素最大和。本题所用到的测试用例会确保答案是一个 32 位整数。


### 示例 1：

输入：arr = [1,15,7,9,2,5,10], k = 3<br>
输出：84<br>
解释：数组变为 [15,15,15,9,10,10,10]
### 示例 2：

输入：arr = [1,4,1,5,7,3,6,1,9,9,3], k = 4<br>
输出：83
### 示例 3：

输入：arr = [1], k = 1<br>
输出：1
 

提示：

$1 <= arr.length <= 500$<br>
$0 <= arr[i] <= 10^9$ <br>
$1 <= k <= arr.length$

# 题解
## 我的正解:
5min左右找到了这个动态转移方程：

```C++
// dp[i] 即从 [0, i] 的变幻后的元素最大和

dp[i] = max(dp[i - 1] + arr[i], // arr[i] 为单独
            dp[i - 2] + max(arr[i], arr[i - 1]) * 2,
            ...,
            dp[i - k] + max(..., arr[i - k]) * k
            )
```

滴八嘎25min...
```C++
class Solution {
public:
    int maxSumAfterPartitioning(vector<int>& arr, int k) {
        const int len = arr.size();
        vector<int> dp(len + 1, 0);
    
        for (int i = 0; i < len; ++i) {
            int maxNum = 0;
            for (int j = 1; j <= k && i - j + 1 >= 0; ++j) {
                if (maxNum < arr[i - j + 1])
                    maxNum = arr[i - j + 1];
                dp[i + 1] = max(dp[i + 1], dp[i - j + 1] + maxNum * j);
            }
        }
    
        return dp[len];
    }
};

// 思考: 为什么要 dp[i + 1] 而不是 dp[i] 为当前?
// (主要是 dp[i + 1] 是对应 arr[i] 的)
// (因为原本的 dp[i] ~ arr[i - 1] 当 i == 0 时, arr[i - 1] 会越界!)
```