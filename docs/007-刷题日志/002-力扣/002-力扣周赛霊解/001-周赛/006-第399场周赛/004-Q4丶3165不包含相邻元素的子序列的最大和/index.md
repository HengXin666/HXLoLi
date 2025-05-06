# 3165. 不包含相邻元素的子序列的最大和
链接: [3165. 不包含相邻元素的子序列的最大和](https://leetcode.cn/problems/maximum-sum-of-subsequence-with-non-adjacent-elements/)

给你一个整数数组 nums 和一个二维数组 queries，其中 queries[i] = $[pos_i, x_i]$。

对于每个查询 $i$，首先将 $nums[pos_i]$ 设置为 $x_i$，然后计算查询 $i$ 的答案，该答案为 nums 中 不包含相邻元素 的 
**子序列** 的 **最大** 和。

返回所有查询的答案之和。

由于最终答案可能非常大，返回其对 10^9 + 7 取余 的结果。

子序列 是指从另一个数组中删除一些或不删除元素而不改变剩余元素顺序得到的数组。

提示:
- 1 <= nums.length <= 5 * 10^4
- -10^5 <= nums[i] <= 10^5
- 1 <= queries.length <= 5 * 10^4
- queries[i] == $[pos_i, x_i]$
- 0 <= $pos_i$ <= nums.length - 1
- -10^5 <= $x_i$ <= 10^5

# 题解
我不会

## 分治思想+线段树（Python/Java/C++/Go）

> [!TIP]
> 你应该学习到: 如果可以 **分治**, 那么就可以写成 **线段树** 的更新形式

0x3f: [分治思想+线段树（Python/Java/C++/Go）](https://leetcode.cn/problems/maximum-sum-of-subsequence-with-non-adjacent-elements/solutions/2790603/fen-zhi-si-xiang-xian-duan-shu-pythonjav-xnhz)