# 3171. 找到按位或最接近 K 的子数组
链接: [3171. 找到按位或最接近 K 的子数组](https://leetcode.cn/problems/find-subarray-with-bitwise-or-closest-to-k/)

给你一个数组 nums 和一个整数 k 。你需要找到 nums 的一个 子数组 ，满足子数组中所有元素按位或运算 OR 的值与 k 的 绝对差 尽可能 小 。换言之，你需要选择一个子数组`nums[l..r]`满足 `|k - (nums[l] OR nums[l + 1] ... OR nums[r])|` 最小。

请你返回 最小 的绝对差值。

子数组 是数组中连续的 非空 元素序列。

提示:
- 1 <= nums.length <= 10^5
- 1 <= nums[i] <= 10^9
- 1 <= k <= 10^9

# 题解
## 0x3f: 利用 OR 的性质
见: [利用 OR 的性质（Python/Java/C++/Go）](https://leetcode.cn/problems/find-subarray-with-bitwise-or-closest-to-k/solutions/2798206/li-yong-and-de-xing-zhi-pythonjavacgo-by-gg4d) | [logTrick【力扣周赛 400】](https://www.bilibili.com/video/BV1Qx4y1E7zj/)

首先我们想思考一个暴力的做法: 遍历出所有的`nums[l..r], (0 <= l <= r <= nums.length)`, 得到`|k - num_lr|`的所有可能的结果.

考虑以下结果:

```C++
/*
for (int i = 0; i < n; ++i) {
	for (int j = i, num = 0; j < n; ++j) {
    	num |= nums[j]; // 得到 num_lr 所有可能的结果
    }
} */

for (int i = 0; i < n; ++i) {
	for (ini j = i - 1; j >= 0; --j) {
    	nums[j] |= nums[i]; // 得到 num_lr 所有可能的结果
    }
}
```

对于位运算, 我们知道: `nums[j] OR nums[i] >= nums[j]`

从集合的角度来看, 相当于`集合A ∪ 集合B`, 如果 $B ⊆ A$, 那么 $A ∪ B = A$, 所以后续的循环都不会改变元素值，可以退出内层循环。

```C++
class Solution {
public:
    int minimumDifference(vector<int>& nums, int k) {
        int ans = INT_MAX;
        for (int i = 0; i < nums.size(); i++) {
            int x = nums[i];
            ans = min(ans, abs(x - k));
            for (int j = i - 1; j >= 0 && (nums[j] | x) != nums[j]; j--) {
                nums[j] |= x;
                ans = min(ans, abs(nums[j] - k));
            }
        }
        return ans;
    }
};
```

## 思考
1. 把 OR 换成 AND 怎么做？[1521. 找到最接近目标值的函数值](https://leetcode.cn/problems/find-a-value-of-a-mysterious-function-closest-to-target/)
2. 把 OR 换成 GCD 怎么做？
3. 把 OR 换成 LCM 怎么做？

## 更加通用的模版 ?

之前的模版只能求得可以, 但不知道是哪个 $[l, r]$ 点, 以下代码不知道是不是更通用的qwq

```C++
class Solution {
public:
    int minimumDifference(vector<int>& arr, int target) {
        int ans = abs(arr[0] - target);
        vector<int> valid = {arr[0]};
        for (int num: arr) {
            vector<int> validNew = {num};
            ans = min(ans, abs(num - target));
            for (int prev: valid) {
                validNew.push_back(prev | num);
                ans = min(ans, abs((prev | num) - target));
            }
            validNew.erase(unique(validNew.begin(), validNew.end()), validNew.end());
            valid = validNew;
        }
        return ans;
    }
};
```
