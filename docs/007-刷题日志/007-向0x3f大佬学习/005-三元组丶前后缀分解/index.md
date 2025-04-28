# 前后缀分解
> 学习视频: [前后缀分解 划分型 DP【力扣周赛 368】](https://www.bilibili.com/video/BV12w411B7ia/)
## 例题
> 链接: [2909. 元素和最小的山形三元组 II](https://leetcode.cn/problems/minimum-sum-of-mountain-triplets-ii/description/)

给你一个下标从`0`开始的整数数组`nums`。

如果下标三元组`(i, j, k)`满足下述全部条件，则认为它是一个`山形三元组`：

- `i < j < k`
- `nums[i] < nums[j] 且 nums[k] < nums[j]`

请你找出`nums`中 **元素和最小** 的山形三元组，并返回其 **元素和** 。如果不存在满足条件的三元组，返回`-1`。

提示: $3 \le nums.length \le 10^5$

## 题解
显然, 直接枚举 $i,j,k$ 是 不可行的.

0x3f大佬指出: 一般这种`山形三元组`/`三元组`/`四元组`的题目, 都是枚举**中间**那个元素.

因为只需要 `nums[i] < nums[j] 且 nums[k] < nums[j]`, 而不需要`nums[i] < nums[k]`或者`nums[i] >= nums[k]`, 所以可以先预处理得出 $i, k$ 处元素的最小值

即 $$SUM_{min}=min(mae[i] + nums[j] + usiro[k]), (i + 1 = j = k -1)$$

而 $$mae[i]=min(nums[0], nums[1], ..., nums[i]), i \in [0, i]$$
同理 $$usiro[k]=min(nums[k], nums[k + 1], ..., nums[n - 1]), k \in [k, n - 1]$$

即代码:

```C++
class Solution {
public:
    int minimumSum(vector<int>& nums) {
        int n = nums.size();
        const int inf = 1e9;
        // 后缀数组: nums[i] ~ nums[n - 1] 的最小值
        vector<int> arr(n + 1, inf);
        for (int i = n - 1; i >= 0; --i) {
            arr[i] = min(arr[i + 1], nums[i]);
        }

        int res = inf;
        int maeMin = nums[0]; // i 的最小值
        // 枚举中间的数
        for (int j = 1; j < n; ++j) {
            maeMin = min(maeMin, nums[j - 1]);
            if (maeMin < nums[j] && nums[j] > arr[j + 1])
                res = min(res, maeMin + nums[j] + arr[j + 1]);
        }

        return res == inf ? -1 : res;
    }
};
```
