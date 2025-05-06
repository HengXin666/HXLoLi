# 3139. 使数组中所有元素相等的最小开销
链接: [3139. 使数组中所有元素相等的最小开销](https://leetcode.cn/problems/minimum-cost-to-equalize-array/)

给你一个整数数组 nums 和两个整数 cost1 和 cost2 。你可以执行以下 任一 操作 **任意** 次：

- 从 nums 中选择下标 i 并且将 nums[i] 增加 1 ，开销为 cost1。
- 选择 nums 中两个 不同 下标 i 和 j ，并且将 nums[i] 和 nums[j] 都 增加 1 ，开销为 cost2 。

你的目标是使数组中所有元素都 相等 ，请你返回需要的 **最小开销** 之和。

由于答案可能会很大，请你将它对 1e9 + 7 取余 后返回。

提示:
- 1 <= nums.length <= 1e5
- 1 <= nums[i] <= 1e6
- 1 <= cost1 <= 1e6
- 1 <= cost2 <= 1e6

# 题解
输入[[1,14,14,15], cost1 = 2, cost2 = 1] 得出 20; (最大值还会变) 不是我能解决的问题, 再见!

## 方法一: 值域范围比较小, 可以枚举最终的值域
> 建议看 [三分法 二分斜率【力扣周赛 396】](https://www.bilibili.com/video/BV1Nf421U7em/)

## 方法二: 贪心 + 二分斜率(三分) + 数学
[分类讨论，O(n) 做法（Python/Java/C++/Go）](https://leetcode.cn/problems/minimum-cost-to-equalize-array/solutions/2766600/fen-lei-tao-lun-on-zuo-fa-pythonjavacgo-9bsb4)