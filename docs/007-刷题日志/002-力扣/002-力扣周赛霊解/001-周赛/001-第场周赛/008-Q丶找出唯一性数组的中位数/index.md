# 3134. 找出唯一性数组的中位数
链接: [3134. 找出唯一性数组的中位数](https://leetcode.cn/problems/find-the-median-of-the-uniqueness-array/)

给你一个整数数组 $nums$。数组 $nums$ 的 **唯一性数组** 是一个按元素从小到大排序的数组，包含了 $nums$ 的所有 **非空子数组中** 不同元素的个数。

换句话说，这是由所有`0 <= i <= j < nums.length`的`distinct(nums[i..j])`组成的递增数组。

其中，`distinct(nums[i..j])`表示从下标 $i$ 到下标 $j$ 的子数组中不同元素的数量。

返回 $nums$ 唯一性数组 的 **中位数**。

注意，数组的 **中位数** 定义为有序数组的中间元素。如果有两个中间元素，则取值较小的那个。

- $1 <= nums.length <= 10^5$
- $1 <= nums[i] <= 10^5$

# 题解
[二分答案+滑动窗口（Python/Java/C++/Go）](https://leetcode.cn/problems/find-the-median-of-the-uniqueness-array/solutions/2759114/er-fen-da-an-hua-dong-chuang-kou-pythonj-ykg9)