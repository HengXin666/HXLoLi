# 3202. 找出有效子序列的最大长度 II
给你一个整数数组 nums 和一个 正 整数 k 。

nums 的一个 子序列 sub 的长度为 x ，如果其满足以下条件，则称其为 有效子序列 ：

- `(sub[0] + sub[1]) % k == (sub[1] + sub[2]) % k == ... == (sub[x - 2] + sub[x - 1]) % k`

返回 nums 的 最长有效子序列 的长度。

## 我的

```C++
class Solution {
public:
    int maximumLength(vector<int>& nums, int k) {
        int n = nums.size();
        vector<unordered_map<int, int>> dp(n);

        int maxLength = 0;
        for (int i = 1; i < n; ++i) {
            for (int j = 0; j < i; ++j) {
                int remainder = (nums[j] + nums[i]) % k;
                if (dp[j].count(remainder)) {
                    dp[i][remainder] = max(dp[i][remainder], dp[j][remainder] + 1);
                } else {
                    dp[i][remainder] = 2; // (nums[j], nums[i])
                }
                maxLength = max(maxLength, dp[i][remainder]);
            }
        }
        
        return maxLength;
    }
};
```

## 灵神

- 学习: [模运算的世界：当加减乘除遇上取模（模运算恒等式/费马小定理）](https://leetcode.cn/circle/discuss/mDfnkW/)

题目要求: $$(a+b)\mod k=(b+c) \mod k$$ 根据前置知识中的恒等式，移项得 $$(a+b-(b+c)) \mod k = 0$$ 化简得: $$(a-c) \mod k = 0$$

(取模运算的四则运算性质类似于乘法/除法)

因此, 我们如果把数列每一项都`%=k`, 那么原问题等价于:

- 寻找一个最长的子序列，满足子序列奇数项都相同，偶数项都相同。

那么可以定义:

- $f[y][x]$ 为最后两项模为 $y, x$ 的子序列的长度。

即:

```C++
class Solution {
public:
    int maximumLength(vector<int>& nums, int k) {
        vector<vector<int>> f(k, vector<int>(k));
        int res = 0;
        for (int i : nums) {
            i %= k;
            for (int j = 0; j < k; ++j)
                res = max(res, f[i][j] = f[j][i] + 1);
        }
        return res;
    }
};
```

- 问: 如何理解这个递推？它和记忆化搜索的区别是什么？

- 答: 对比二者的计算顺序。如果用记忆化搜索来做，需要单独计算「最左（或者最右）两项模 $k$ 分别为和 $y$ 的子序列」的长度，这是「单线程」，必须查找下一个元素的位置。而递推的计算顺序是，（假设我们先遍历到了元素2，然后遍历到了元素4，两个元素属于不同的子序列）一会计算一下「最后两项模 $k$ 分别为 $y$ 和 2 的子序列」，一会又计算一下「最后两项模 $k$ 分别为 $y$ 和 4 的子序列」，这是「多线程」，没有查找元素位置的过程，遇到谁就处理谁。