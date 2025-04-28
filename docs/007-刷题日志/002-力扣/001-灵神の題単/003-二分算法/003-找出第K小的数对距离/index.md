# 719. 找出第 K 小的数对距离
链接: [719. 找出第 K 小的数对距离](https://leetcode.cn/problems/find-k-th-smallest-pair-distance/)

数对 (a,b) 由整数 a 和 b 组成，其数对距离定义为 a 和 b 的绝对差值。

给你一个整数数组 nums 和一个整数 k ，数对由 nums[i] 和 nums[j] 组成且满足 0 <= i < j < nums.length 。返回 所有数对距离中 第 k 小的数对距离。

# 题解
## 二分 + 滑动窗口

- 排序后, index 为 [l, r] 的数的距离绝对不会超过 nums[r] - nums[l], 那么对于固定的 l, 那么有 cnt = r - l - 1 个数对。

- 二分距离, 然后统计数对数量即可

```C++
class Solution {
public:
    int smallestDistancePair(vector<int>& nums, int k) {
        // 二分: 统计距离 <= x 的个数
        sort(nums.begin(), nums.end());
        int l = -1, r = nums.back() - nums[0] + 1, n = nums.size();
        auto fk = [&](int x) -> bool {
            int cnt = 0;
            for (int i = 0, j = 0; i < n; ++i) {
                while (j < n && nums[j] - nums[i] <= x)
                    ++j;
                cnt += j - i - 1;
            }
            return cnt < k;
        };
        while (l + 1 < r) {
            int mid = l + ((r - l) >> 1);
            (fk(mid) ? l : r) = mid;
        }
        return r;
    }
};
```
