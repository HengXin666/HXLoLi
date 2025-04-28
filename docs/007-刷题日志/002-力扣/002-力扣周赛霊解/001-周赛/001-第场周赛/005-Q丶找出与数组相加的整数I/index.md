# 3131. 找出与数组相加的整数 I
链接: [3131. 找出与数组相加的整数 I](https://leetcode.cn/problems/find-the-integer-added-to-array-i/)

给你两个长度相等的数组 nums1 和 nums2。

数组 nums1 中的每个元素都与变量 x 所表示的整数相加。如果 x 为负数，则表现为元素值的减少。

在与 x 相加后，nums1 和 nums2 相等 。当两个数组中包含相同的整数，并且这些整数出现的频次相同时，两个数组 相等 。

返回整数 x 。

# 题解

```C++
class Solution {
public:
    int addedInteger(vector<int>& nums1, vector<int>& nums2) {
        sort(nums1.begin(), nums1.end());
        sort(nums2.begin(), nums2.end());
        return nums2[0] - nums1[0];
    }
};
```

```C++
class Solution {
public:
    int addedInteger(vector<int>& nums1, vector<int>& nums2) {
        int min1 = 1e9, min2 = 1e9;
        for (int i = 0; i < nums1.size(); ++i) {
            min1 = min(min1, nums1[i]);
            min2 = min(min2, nums2[i]);
        }
        return min2 - min1;
    }
};
```