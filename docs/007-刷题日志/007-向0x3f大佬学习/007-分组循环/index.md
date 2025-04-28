# 小技巧: 分组循环
题目: [3105. 最长的严格递增或递减子数组](https://leetcode.cn/problems/longest-strictly-increasing-or-strictly-decreasing-subarray/description/)

给你一个整数数组`nums`。

返回数组`nums`中 **严格递增** 或 **严格递减** 的最长非空**子数组**的长度

## 解题
### 暴力
枚举可能的左端点, 然后分别计算 $O(n^2)$

```C++
class Solution {
public:
    int longestMonotonicSubarray(vector<int>& nums) {
        int addLen = 1, jianLen = 1;
        
        for (int i = 0; i < nums.size(); ++i) {
            for (int j = 1; i + j < nums.size(); ++j) {
                if (nums[i + j - 1] < nums[i + j]) {
                    addLen = max(addLen, j + 1);
                }
                else
                    break;
            }
        }
        
        for (int i = 0; i < nums.size(); ++i) {
            for (int j = 1; i + j < nums.size(); ++j) {
                if (nums[i + j - 1] > nums[i + j]) {
                    jianLen = max(jianLen, j + 1);
                }
                else
                    break;
            }
        }
        
        return max(jianLen, addLen);
    }
};
```

### 分组循环
首先, 我们要思考到:
- 如果我们计算了 [1, 2, 3] 是一个递增的子数组,.
- 那么 还需要计算 [2, 3] 吗?
    - 显然是不用的, 因为长度必然小于 前者.

因此我们可以使用一个[滑动窗口](../003-滑动窗口丶/index.md), 记录起点, 然后向右拓展.

但是需要分别处理两种情况, 十分麻烦!

于是`03xf`大佬告诉我们, 一个小技巧: [分组循环]

- 重点关注`tag == (nums[right] > nums[right - 1])`这行, 它保证了情况和之前的一样!

考虑以下数组:
```C++
      4
    3   3
  2       2
1
```
有 [1,2,3,4] 与 [4,3,2] 两种, O(n) 就可以遍历完

```C++
class Solution {
public:
    int longestMonotonicSubarray(vector<int>& nums) {
        // 一种 O(n) 的算法
        int res = 1;
        for (int right = 1; right < nums.size(); ++right) {
            int left = right - 1;
            if (nums[right] == nums[right - 1])
                continue;

            bool tag = nums[right] > nums[right - 1]; // 递增
            ++right; // 当前情况是满足的
            while (right < nums.size()
            && nums[right] != nums[right - 1]
            && tag == (nums[right] > nums[right - 1]))
                ++right;
            
            res = max(res, right - left);
            --right;
        }
        return res;
    }
};
```
