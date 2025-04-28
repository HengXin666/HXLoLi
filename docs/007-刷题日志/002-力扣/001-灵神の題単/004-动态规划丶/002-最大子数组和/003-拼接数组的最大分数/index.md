# 2321. 拼接数组的最大分数
链接: [2321. 拼接数组的最大分数](https://leetcode.cn/problems/maximum-score-of-spliced-array/)

困难`1791` 第 299 场周赛 Q3

给你两个下标从 0 开始的整数数组 nums1 和 nums2 ，长度都是 n 。

你可以选择两个整数 left 和 right ，其中 0 <= left <= right < n ，接着 交换 两个子数组 nums1[left...right] 和 nums2[left...right] 。

例如，设 nums1 = [1,2,3,4,5] 和 nums2 = [11,12,13,14,15] ，整数选择 left = 1 和 right = 2，那么 nums1 会变为 [1,12,13,4,5] 而 nums2 会变为 [11,2,3,14,15] 。
你可以选择执行上述操作 一次 或不执行任何操作。

数组的 分数 取 sum(nums1) 和 sum(nums2) 中的最大值，其中 sum(arr) 是数组 arr 中所有元素之和。

返回 可能的最大分数 。

子数组 是数组中连续的一个元素序列。arr[left...right] 表示子数组包含 nums 中下标 left 和 right 之间的元素（含 下标 left 和 right 对应元素）。

# 题解
## 基于提示的状态机dp
我看提示的: 假设 DP 是 DP（pos， state）。POS 是您当前所处的位置。state 是 {0,1,2} 之一，其中 0 表示取数组 A，1 表示取子数组 B，2 表示再次取数组 A。我们需要谨慎处理过渡。


```C++
class Solution {
public:
    int maximumsSplicedArray(
        vector<int>& nums1, vector<int>& nums2
        ) {
        int n = nums1.size();
        vector<vector<int>> sumArr1(n + 1, vector<int>(3)), 
                            sumArr2(n + 1, vector<int>(3));
        for (int i = 0; i < n; ++i) {
            sumArr1[i + 1][0] = sumArr1[i][0] + nums1[i];
            sumArr1[i + 1][1] = max(sumArr1[i][0] + nums1[i],
                                    sumArr1[i][1] + nums2[i]);
            sumArr1[i + 1][2] = max(sumArr1[i][2] + nums1[i],
                                    sumArr1[i][1] + nums2[i]);

            sumArr2[i + 1][0] = sumArr2[i][0] + nums2[i];
            sumArr2[i + 1][1] = max(sumArr2[i][0] + nums2[i],
                                    sumArr2[i][1] + nums1[i]);
            sumArr2[i + 1][2] = max(sumArr2[i][2] + nums2[i],
                                    sumArr2[i][1] + nums1[i]);
        }
        
        return max({
            sumArr1[n][0],
            sumArr1[n][1],
            sumArr1[n][2], 
            sumArr2[n][0],
            sumArr2[n][1],
            sumArr2[n][2],
        });
    }
};
```

## 灵神的妙转化
- [转换成最大子数组和（Python/Java/C++/Go）](https://leetcode.cn/problems/maximum-score-of-spliced-array/solutions/1626030/by-endlesscheng-fm8l)

设 $s_1 = \sum_i{nums_1[i]}$ 交换 $[L, R]$ 范围内的数有 $$\sum_i{nums_1'[i]} = s_1 - (nums_1[L] + nums_1[L + 1] + ... + nums_1[R]) + (nums_2[L] + nums_2[L + 1] + ... + nums_2[R])$$ 即可以化为 $$\sum_i{nums_1'[i]} = s_1 + (nums_2[L] - nums_1[L]) + ... + (nums_2[R] - nums_1[R])$$ 设 $diff[i] = nums_2[i] - nums_1[i]$, 上式变为 $$s_1 + diff[L] + ... + diff[R]$$ 为了最大化上式，我们需要最大化 $diff$ 数组和 (允许数组为空), $nums_2$ 同理.

```C++
class Solution {
    int solve(vector<int> &nums1, vector<int> &nums2) {
        int s1 = 0, maxSum = 0;
        for (int i = 0, s = 0; i < nums1.size(); ++i) {
            s1 += nums1[i];
            s = max(s + nums2[i] - nums1[i], 0);
            // 改为 s = max(s, 0) + nums2[i] - nums1[i]; 也一样
            maxSum = max(maxSum, s);
        }
        return s1 + maxSum;
    }

public:
    int maximumsSplicedArray(vector<int> &nums1, vector<int> &nums2) {
        return max(solve(nums1, nums2), solve(nums2, nums1));
    }
};
```
