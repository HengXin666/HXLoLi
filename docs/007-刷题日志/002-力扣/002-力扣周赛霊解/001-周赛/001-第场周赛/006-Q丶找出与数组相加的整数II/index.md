# 3132. 找出与数组相加的整数 II
链接: [3132. 找出与数组相加的整数 II](https://leetcode.cn/problems/find-the-integer-added-to-array-ii/)

给你两个整数数组 nums1 和 nums2。

从 nums1 中移除两个元素，并且所有其他元素都与变量 x 所表示的整数相加。如果 x 为负数，则表现为元素值的减少。

执行上述操作后，nums1 和 nums2 相等 。当两个数组中包含相同的整数，并且这些整数出现的频次相同时，两个数组 相等 。

返回能够实现数组相等的 最小 整数 x 。

- `3 <= nums1.length <= 200`
- `nums2.length == nums1.length - 2`
- `0 <= nums1[i], nums2[i] <= 1000`
- 测试用例以这样的方式生成：存在一个整数 x，nums1 中的每个元素都与 x 相加后，再移除两个元素，nums1 可以与 nums2 相等。

# 题解
## 暴力
数据范围告诉我们可以暴力! 枚举删除的数, 并且保证其合法:

```C++
class Solution {
public:
    int minimumAddedInteger(
        vector<int>& nums1, vector<int>& nums2) {
        int x = 1e9;
        // nums2[i] - nums1[i] = x
        // 暴力!
        sort(nums1.begin(), nums1.end());
        sort(nums2.begin(), nums2.end());
        for (int i = 0; i < nums1.size(); ++i) { // 删除 i
            for (int j = i + 1; j < nums1.size(); ++j) { // 删除 j
                int tmp_x = -1e9;
                for (int k = 0, py = 0; k < nums2.size(); ++k) {
                    if (k + py == i)
                        ++py;
                    if (k + py == j)
                        ++py;
                    
                    if (tmp_x != (int)-1e9 && tmp_x != nums2[k] - nums1[k + py])
                        goto A;
                    
                    tmp_x = nums2[k] - nums1[k + py];
                }
                x = min(x, tmp_x);
                A:
                ;
            }
        }
        return x;
    }
};
```

## 排序+枚举

我们可以通过排序后确认需要保留的是谁`nums1[0~2]`(因为题目保证可以得出答案), 然后问题就变成判断`nums2`的合法性, 即判断 $nums2$ 是否为 $nums1[i] + diff$ 的子序列([392. 判断子序列](https://leetcode.cn/problems/is-subsequence/))

```C++
class Solution {
public:
    int minimumAddedInteger(vector<int>& nums1, vector<int>& nums2) {
        ranges::sort(nums1);
        ranges::sort(nums2);
        // 枚举保留 nums1[2] 或者 nums1[1] 或者 nums1[0]
        // 倒着枚举是因为 nums1[i] 越大答案越小，第一个满足的就是答案
        for (int i = 2; i; i--) {
            int diff = nums2[0] - nums1[i];
            // 在 {nums1[i] + diff} 中找子序列 nums2
            int j = 0;
            for (int k = i; k < nums1.size(); k++) {
                if (nums2[j] == nums1[k] + diff && ++j == nums2.size()) {
                    // nums2 是 {nums1[i] + diff} 的子序列
                    return diff;
                }
            }
        }
        // 题目保证答案一定存在
        return nums2[0] - nums1[0];
    }
};
```
