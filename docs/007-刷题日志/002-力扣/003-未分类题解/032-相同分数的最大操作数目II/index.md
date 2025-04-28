# 3040. 相同分数的最大操作数目 II

原题: [3040. 相同分数的最大操作数目 II](https://leetcode.cn/problems/maximum-number-of-operations-with-the-same-score-ii/)

中等


给你一个整数数组 nums ，如果 nums 至少 包含 2 个元素，你可以执行以下操作中的 任意 一个：

选择 nums 中最前面两个元素并且删除它们。<br>
选择 nums 中最后两个元素并且删除它们。<br>
选择 nums 中第一个和最后一个元素并且删除它们。<br>
一次操作的 分数 是被删除元素的和。

在确保 所有操作分数相同 的前提下，请你求出 最多 能进行多少次操作。

请你返回按照上述要求 **最多** 可以进行的操作次数。

## 示例 1：

输入：nums = [3,2,1,2,3,4]
输出：3
解释：我们执行以下操作：
- 删除前两个元素，分数为 3 + 2 = 5 ，nums = [1,2,3,4] 。
- 删除第一个元素和最后一个元素，分数为 1 + 4 = 5 ，nums = [2,3] 。
- 删除第一个元素和最后一个元素，分数为 2 + 3 = 5 ，nums = [] 。<br>
由于 nums 为空，我们无法继续进行任何操作。
## 示例 2：

输入：nums = [3,2,6,1,4]
输出：2
解释：我们执行以下操作：
- 删除前两个元素，分数为 3 + 2 = 5 ，nums = [6,1,4] 。
- 删除最后两个元素，分数为 1 + 4 = 5 ，nums = [6] 。<br>
至多进行 2 次操作。
 

提示：

$2 <= nums.length <= 2000$ <br>
$1 <= nums[i] <= 1000$ <br>

# 题解

这是一个 区间dp, 因为操作:

> 选择 nums 中最前面两个元素并且删除它们。<br>
选择 nums 中最后两个元素并且删除它们。<br>
选择 nums 中第一个和最后一个元素并且删除它们。<br>

子问题是「从两侧向内缩小的」 => 区间DP

dfs(i, j) = 操作 a[i] ... a[j] 这一段子数组(闭区间[i, j])的最多可以进行的操作数(一般题目求什么状态就定义为什么)

```C++
class Solution {
public:
    int bfs(int i, int j, int k, vector<vector<int>>& dp, vector<int>& nums) {
        if (i >= j)  // 退出条件: 元素数量 <= 1
            return 0;

        if (dp[i][j])
            return dp[i][j];
        
        if (nums[i] + nums[j] == k)
            dp[i][j] = max(dp[i][j], bfs(i + 1, j - 1, k, dp, nums) + 1);
        if (nums[i] + nums[i + 1] == k)
            dp[i][j] = max(dp[i][j], bfs(i + 2, j, k, dp, nums) + 1);
        if (nums[j - 1] + nums[j] == k)
            dp[i][j] = max(dp[i][j], bfs(i, j - 2, k, dp, nums) + 1);
        
        return dp[i][j];
    }

    int maxOperations(vector<int>& nums) {
        // dp[i][j] 区间[i, j]的操作数, 在差为t的情况下
        // dp[i][j] = dp[i + 1][j - 1], dp[i + 2][j], dp[i][j - 2] 的 max
        vector<vector<int>> dp(nums.size(), vector<int>(nums.size(), 0));

        int len = nums.size();
        int res = bfs(1, len - 2, nums[0] + nums[len - 1], dp, nums);
        res = max(res, bfs(2, len - 1, nums[0] + nums[1], dp, nums));
        res = max(res, bfs(0, len - 3, nums[len - 1] + nums[len - 2], dp, nums));

        return res + 1;
    }
};
```
