# 300. 最长递增子序列
原题: [300. 最长递增子序列](https://leetcode.cn/problems/longest-increasing-subsequence/)

中等

给你一个整数数组 `nums`，找到其中最长严格递增子序列的长度。

子序列 是由数组派生而来的序列，删除（或不删除）数组中的元素而不改变其余元素的顺序。例如，[3,6,2,7] 是数组 [0,3,1,6,2,2,7] 的子序列。

 
## 示例 1：

输入：`nums = [10,9,2,5,3,7,101,18]`<br>
输出：`4`<br>
解释：最长递增子序列是 [2,3,7,101]，因此长度为 4 。<br>
## 示例 2：
输入：`nums = [0,1,0,3,2,3]`<br>
输出：`4`<br>
## 示例 3：
输入：`nums = [7,7,7,7,7,7,7]`<br>
输出：`1`<br>
 
## 提示：

$1 <= nums.length <= 2500$ <br>
$-10^4 <= nums[i] <= 10^4$
 

## 进阶：

你能将算法的时间复杂度降低到 `O(n log(n))` 吗?

# 题解
## 直接看
[动态规划 （包含O (N log N) 解法的状态定义以及解释）](https://leetcode.cn/problems/longest-increasing-subsequence/solutions/7196/dong-tai-gui-hua-er-fen-cha-zhao-tan-xin-suan-fa-p) | 修改状态定义（同时用到了贪心算法、二分查找）| **<有机会一定要好好学学!!!!>**

## 我的代码
### 记忆化搜索 O(N^2)

```C++
class Solution {
public:
    int BFS(int now_i, vector<int>& nums, vector<int>& dp) {
        if (dp[now_i]) {
            return dp[now_i];
        }

        int res = 1; // 自己算一个长度
        for (int i = 0; i < now_i; ++i) {
            if (nums[now_i] > nums[i]) {
                res = max(res, BFS(i, nums, dp) + 1);
            }
        }

        dp[now_i] = res;
        return dp[now_i];
    }

    int lengthOfLIS(vector<int>& nums) {
        const int len = nums.size();
        vector<int> dp(len, 0);
        int res = 1;
        for (int i = len - 1; i >= 0; --i) {
            res = max(res, BFS(i, nums, dp));
        }

        return res;
    }
};
```
### dp写法

```C++
class Solution {
public:
    int lengthOfLIS(vector<int>& nums) {
        int n = nums.size();
        vector<int> dp(n, 1);
        int res = 0;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < i; ++j) {
                if (nums[j] < nums[i]) {
                    dp[i] = max(dp[j] + 1, dp[i]);
                }
            }
            if (res < dp[i])
                res = dp[i];
        }

        return res;
    }
};
```
