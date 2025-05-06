3177\. 求出最长好子序列 II
------------------

- 链接: [3177\. 求出最长好子序列 II](https://leetcode.cn/problems/find-the-maximum-length-of-a-good-subsequence-ii/description/)

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

*   `1 <= nums.length <= 5 * 10^3`
*   `1 <= nums[i] <= 10^9`
*   0 <= k <= min(50, nums.length)

# 题解
## 0x3f

- 见: [动态规划+优化（Python/Java/C++/Go）](https://leetcode.cn/problems/find-the-maximum-length-of-a-good-subsequence-ii/solutions/2805263/dong-tai-gui-hua-you-hua-pythonjavacgo-b-jqn2)

~~不会~~

```C++
class Solution {
public:
    int maximumLength(vector<int>& nums, int k) {
        unordered_map<int, vector<int>> fs;
        vector<int> mx(k + 2);
        for (int x : nums) {
            if (!fs.contains(x)) {
                fs[x] = vector<int>(k + 1);
            }
            auto& f = fs[x];
            for (int j = k; j >= 0; j--) {
                f[j] = max(f[j], mx[j]) + 1;
                mx[j + 1] = max(mx[j + 1], f[j]);
            }
        }
        return mx[k + 1];
    }
};
```
