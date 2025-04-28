# 3145. 大数组元素的乘积
链接: [3145. 大数组元素的乘积](https://leetcode.cn/problems/find-products-of-elements-of-big-array/)

一个整数 x 的 强数组 指的是满足和为 x 的二的幂的最短有序数组。比方说，11 的强数组为 [1, 2, 8] 。

我们将每一个正整数 i （即1，2，3等等）的 强数组 连接得到数组 big_nums ，big_nums 开始部分为 [1, 2, 1, 2, 4, 1, 4, 2, 4, 1, 2, 4, 8, ...] 。

给你一个二维整数数组 queries ，其中 queries[i] = [fromi, toi, modi] ，你需要计算 (big_nums[fromi] * big_nums[fromi + 1] * ... * big_nums[toi]) % modi 。

请你返回一个整数数组 answer ，其中 answer[i] 是第 i 个查询的答案。

## 题解
> 不是我可以会的 (数学 + 位运算 + 数列建模并求通项公式 + 数学归纳法 + ...(不敢看)), 没事灵神已经出手, 我已经开溜~