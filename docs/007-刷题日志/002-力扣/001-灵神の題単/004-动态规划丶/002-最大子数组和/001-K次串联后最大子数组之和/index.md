# 1191. K 次串联后最大子数组之和
链接: [1191. K 次串联后最大子数组之和](https://leetcode.cn/problems/k-concatenation-maximum-sum/)

中等`1748` 
第 154 场周赛 Q3

给定一个整数数组 arr 和一个整数 k ，通过重复 k 次来修改数组。

例如，如果`arr = [1, 2] ， k = 3`，那么修改后的数组将是`[1, 2, 1, 2, 1, 2]`。

返回修改后的数组中的最大的子数组之和。注意，子数组长度可以是 0，在这种情况下它的总和也是 0。

由于 结果可能会很大，需要返回的`1e9 + 7`的 模 。

# 题解

1. 最大子数组和 < 0；说明整个数组全为负数，返回0；

2. 最大子数组和 >= 0；

	1. 如果数组和小于0；串联多个数组时，中间的数组相加对最终的贡献都是小于0的，所以只能看相邻的两个数组之间，可能有一个子数组的和为正，这个子数组的最大和为 = 前一个数组的最大后缀和 + 后一个数组的最大前缀和；此时，最终结果 = max（最大子数组和, 最大前缀和+最大后缀和）
    
    2. 此时，最大子数组和>=0; 数组和>=0; <span style="color:red">中间的数组都会贡献增加值</span>，为 (k-2)*sum；<span style="color:red">第一个数组贡献最大后缀和，最后一个数组贡献最大前缀和</span>；结果为 max_post + (k-2)*sum + max_pre

```C++
class Solution {
    static constexpr int mod = 1e9 + 7;
    using ll = long long;
public:
    int kConcatenationMaxSum(vector<int>& arr, int k) {
        ll res = 0, n = arr.size(), tmp = 0, sum = 0;
        for (int i = 0; i < n; ++i) {
            sum += arr[i];
            tmp = max(0LL, tmp) + arr[i];
            res = max(res, tmp);
        }
        if (k == 1)
            return res % mod;
        ll res2 = res;
        for (int i = 0; i < n; ++i) {
            tmp = max(0LL, tmp) + arr[i];
            res2 = max(res2, tmp);
        }
        if (k == 2)
            return res2 % mod;
        return sum > 0 
        ? (res2 + (k - 2) * sum) % mod
        : res2;
    }
};
```