3176\. 求出最长好子序列 I
-----------------

- 链接: [3176\. 求出最长好子序列 I](https://leetcode.cn/problems/find-the-maximum-length-of-a-good-subsequence-i/description/)

给你一个整数数组 nums 和一个 **非负** 整数 k 。如果一个整数序列 seq 满足在范围下标范围 \[0, seq.length - 2\] 中存在 **不超过** k 个下标 i 满足 seq\[i\] != seq\[i + 1\] ，那么我们称这个整数序列为 **好** 序列。

请你返回 nums 中 **好**

子序列

 的最长长度

**示例 1：**

**输入：** nums = \[1,2,1,1,3\], k = 2

**输出：** 4

**解释：**

最长好子序列为 [_**1**_,_**2**_,**_1_**,_**1**_,3] 。

**示例 2：**

**输入：** nums = \[1,2,3,4,5,1\], k = 0

**输出：** 2

**解释：**

最长好子序列为 [**_1_**,2,3,4,5,**_1_**] 。

**提示：**

*   1 <= nums.length <= 500
*   `1 <= nums[i] <= 10^9`
*   0 <= k <= min(nums.length, 25)

# 题解
## HX

```C++
class Solution {
public:
    int maximumLength(vector<int>& nums, int k) {
        // f[i][k] 选择第i不超过k
        vector<vector<int>> memo(nums.size(), vector<int>(k + 1, -1));
        
        function<int(int, int)> dfs = [&](int i, int j) -> int {
            if (i < 0 || j < 0)
                return -1e8;
            
            if (memo[i][j] != -1)
                return memo[i][j];
            
            int& res = memo[i][j];
            
            res = 0;
            
            for (int it = i; it >= 0; --it) {
                res = max({
                    res, 
                    dfs(it, j - (nums[i] != nums[it])) + 1,
                });
            }
            
            return res;
        };
        
        int res = 0;
        for (int i = 0; i < nums.size(); ++i)
            res = max(res, dfs(i, k));
        
        return res;
    }
};
```

## 请看 Q4 正解