# 3162. 优质数对的总数 I
链接: [3162. 优质数对的总数 I](https://leetcode.cn/problems/find-the-number-of-good-pairs-i/)

给你两个整数数组 nums1 和 nums2，长度分别为 n 和 m。同时给你一个正整数 k。

如果 nums1[i] 可以被 nums2[j] * k 整除，则称数对 (i, j) 为 优质数对（0 <= i <= n - 1, 0 <= j <= m - 1）。

返回 优质数对 的总数。

# 题解
## 暴力

```C++
class Solution {
public:
    int numberOfPairs(vector<int>& nums1, vector<int>& nums2, int k) {
        int res = 0;
        for (int i = 0; i < nums1.size(); ++i) {
            for (int j = 0; j < nums2.size(); ++j)
            if (nums1[i] % (nums2[j] * k) == 0)
                ++res;
        }
        return res;
    }
};
```

## 非暴力
见`Q3`