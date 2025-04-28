# 滑动窗口
> 学习: [滑动窗口【基础算法精讲 03】](https://www.bilibili.com/video/BV1hd4y1r7Gq/)

## 例题
链接: [209. 长度最小的子数组](https://leetcode.cn/problems/minimum-size-subarray-sum/description/)

给定一个含有 $n$ 个**正整数**的数组和一个正整数 $target$ 。

找出该数组中满足其总和大于等于 $target$ 的长度最小的 **连续** 子数组`[numsl, numsl+1, ..., numsr-1, numsr]`，并返回其长度。如果不存在符合条件的子数组，返回 `0`。

显然我们可以有一个暴力的做法:

枚举所有的 $i$ 为数组起点, 然后枚举所有的 $j$ 为数组往后的元素, 然后存储符合条件的 $resLen$, 显然这样做法的最坏的时间复杂度是 $O(n^2)$

但是, 显然会有更优的做法:

- 因为 数组中的数字都是正整数, 所以 $sum$ 是单调递增的, 那么我们可以通过一个 **窗口** 即 计算 `nums[i] ~ nums[j]` 的 $sum$
    - 当 $sum \ge target$ , 那么我们可以先 把 nums[i] 从 sum 中减去, 再看是否 $sum \ge target$, 直到 $sum < target$, 我们就继续`++j`
    - 注意: 上面的成立前提是 $sum$ 是单调递增的 (即 数组中的数字都是正整数); 不然就会导致 $sum_{i, j} < sum_{i, j + 1}$
    - 代码:
      ```C++
      class Solution {
      public:
          int minSubArrayLen(int target, vector<int>& nums) {
              // 总和大于等于 target 的长度最小的 连续子数组 的长度
              int now_sum = 0;
              int len = 1e9;
              int left = 0;
              for (int right = 0; right < nums.size(); ++right) {
                  now_sum += nums[right];
                  while (now_sum >= target) {
                      now_sum -= nums[left];
                      len = min(len, right - left + 1);
                      ++left;
                  }
              }
              return len < 1e9 ? len : 0;
          }
      };
      ```
- 小技巧: 对于 $len$ 的计算, 什么时候 需要 $right - left + 1$ 的 `+1` 什么时候不用?
    - 我们可以直接使用 **特殊值带数法**
        - 比如这一题求的是长度, 假如有一个长度为`1`的答案, 显然是只有 $left = right$ 的时候取得的答案, 显然, $left-right$ 是 为`0`的, 因此我们需要`+1`

## 练习

- [【题单】滑动窗口](https://leetcode.cn/circle/discuss/0viNMK/)