# 3107. 使数组中位数等于 K 的最少操作数
链接: [3107. 使数组中位数等于 K 的最少操作数](https://leetcode.cn/problems/minimum-operations-to-make-median-of-array-equal-to-k/)

中等 第392场周赛 Q3

---

给你一个整数数组 nums 和一个 非负 整数 k 。

一次操作中，你可以选择任一下标 i ，然后将 nums[i] 加 1 或者减 1 。

请你返回将 nums 中位数 变为 k 所需要的 最少 操作次数。

一个数组的 中位数 指的是数组按 非递减 顺序排序后最中间的元素。如果数组长度为偶数，我们选择中间两个数的较大值为中位数。

## 示例 1：

输入：nums = [2,5,6,8,5], k = 4

输出：2

解释：我们将 nums[1] 和 nums[4] 减 1 得到 [2, 4, 6, 8, 4] 。现在数组的中位数等于 k 。所以答案为 2 。

## 示例 2：

输入：nums = [2,5,6,8,5], k = 7

输出：3

解释：我们将 nums[1] 增加 1 两次，并且将 nums[2] 增加 1 一次，得到 [2, 7, 7, 8, 5] 。结果数组的中位数等于 k 。所以答案为 3 。

## 示例 3：

输入：nums = [1,2,3,4,5,6], k = 4

输出：0

解释：数组中位数已经等于 k 了，所以不需要进行任何操作。

## 提示：
$
1 <= nums.length <= 2 * 10^5\\
1 <= nums[i] <= 10^9\\
1 <= k <= 10^9\\
$

# 题解
## 我是sb

我喵了一眼题目, 就以为是[[中位数贪心]货仓寻址问题](../../../../001-计佬常識/001-数据结构与算法/012-【算法】贪心/001-【中位数贪心】货仓寻址问题/index.md), 然后就按照这个的思路写了, 但是! 这里我分类讨论了: 奇数 的 三种情况 与 偶数的三种情况, 再加上 前缀和 求的for循环写成`i = 1`开始, 直接爆炸! 调了1个小时, 最好以 6 wa 的牛逼历史在最后3min内通过!艹

```C++
class Solution {
        int erfen2(int t, const vector<int>& nums) { // < k
            int l = 0, r = nums.size() - 1;
            while (l <= r) {
                int mid = ((r - l) >> 1) + l;
                if (nums[mid] <= t) {
                    l = mid + 1;
                } else {
                    r = mid - 1;
                }
            }
            
            return r;
        }
    
        int erfen(int t, const vector<int>& nums) { // >= k
            int l = 0, r = nums.size() - 1;
            while (l <= r) {
                int mid = ((r - l) >> 1) + l;
                if (nums[mid] < t) {
                    l = mid + 1;
                } else {
                    r = mid - 1;
                }
            }
            
            return l;
        }
    
public:
    long long minOperationsToMakeMedianK(vector<int>& nums, int a) {
        sort(nums.begin(), nums.end());
        int n = nums.size();
        long long k = a;
        vector<long long> sumArr(n + 2);
        for (int i = 0; i < n; ++i)
            sumArr[i + 1] = sumArr[i] + nums[i]; // 前缀和
        long long res = 0; // 操作次数
        if (n & 1) { // 奇数
            if (k <= nums[n / 2]) {
                int i = erfen(k, nums);
                
                res = (sumArr[n / 2 + 1] - sumArr[i]) - k * (n / 2 - i + 1);
            }
            else {
                int i = erfen2(k, nums);
                res = k * (i - n / 2 + 1) - (sumArr[i + 1] - sumArr[n / 2]);
            }
        } else { // 偶数
            if (k > nums[n / 2]) { // 大大大
                int i = erfen2(k, nums);
                res = k * (i - n / 2 + 1) - (sumArr[i + 1] - sumArr[n / 2]);
            } else if (k < nums[n / 2 - 1]) { // 小小小
                int i = erfen(k, nums);
                res = (sumArr[n / 2] - sumArr[i]) - k * (n / 2 - i);
                // if (nums[n / 2 - 1] == nums[n / 2]) { // 俩个中位数相等
                    res += nums[n / 2] - k;
                // }
            } else { // 且 k == 俩个中位数相等
                res += abs(max(nums[n/2], nums[n/2 - 1]) - k);
            }
        }
        return res;
    }
};
```

## 正解

实际上只需要以中位数为起点, 计算差即可, 感觉这样比Q2还简单, 艹

```C++
class Solution {
public:
    long long minOperationsToMakeMedianK(
        vector<int>& nums, int k) {
        sort(nums.begin(), nums.end());
        int z = nums.size() / 2;

        long long res = 0;
        for (int i = z; i < nums.size(); ++i) {
            if (k > nums[i]) {
                res += k - nums[i]; 
            }
        }

        for (int i = z; i >= 0; --i) {
            if (k < nums[i]) {
                res += nums[i] - k;
            }
        }

        return res;
    }
};
```