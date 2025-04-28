# 2411. 按位或最大的最小子数组长度
链接: [2411. 按位或最大的最小子数组长度](https://leetcode.cn/problems/smallest-subarrays-with-maximum-bitwise-or/)

第 87 场双周赛 Q3 `1938`

给你一个长度为 n 下标从 0 开始的数组 nums ，数组中所有数字均为非负整数。对于 0 到 n - 1 之间的每一个下标 i ，你需要找出 nums 中一个 最小 非空子数组，它的起始位置为 i （包含这个位置），同时有 最大 的 按位或运算值 。

- 换言之，令 $B_{ij}$ 表示子数组`nums[i...j]`的按位或运算的结果，你需要找到一个起始位置为 i 的最小子数组，这个子数组的按位或运算的结果等于 $max(B_{ik})$ ，其中 `i <= k <= n - 1`。

一个数组的按位或运算值是这个数组里所有数字按位或运算的结果。

请你返回一个大小为 n 的整数数组 answer，其中 answer[i]是开始位置为 i ，按位或运算结果最大，且 最短 子数组的长度。

子数组 是数组里一段连续非空元素组成的序列。

# 题解
## 暴力 $O(n^2)$

我偷灵神已经优化过的了: (因为我写的居然有八嘎!?)

```C++
class Solution {
public:
    vector<int> smallestSubarrays(vector<int> &nums) {
        int n = nums.size();
        vector<int> ans(n);
        for (int i = 0; i < n; ++i) {
            ans[i] = 1;
            for (int j = i - 1; j >= 0 && (nums[j] | nums[i]) != nums[j]; --j) {
                nums[j] |= nums[i];
                ans[j] = i - j + 1;
            }
        }
        return ans;
    }
};
```

有BUG: (会wa!)
```C++
class Solution {
public:
    vector<int> smallestSubarrays(vector<int>& nums) {
        int maxx = 0;
        for (int& it : nums)
            maxx |= it;
        
        vector<int> res(nums.size());

        for (int i = 0; i < nums.size(); ++i) {
            int now = 0, j = i;
            bool tag = 0;
            for (; j < nums.size(); ++j) {
                now |= nums[j];
                if (now == maxx) {
                    tag = 1;
                    break;
                }
            }

            res[i] = j - i + tag;
        }
        
        return res;
    }
};
```


## 二进制滑动窗口 $O(n)$

倒着来, 如果 当前 nums[i] 的二进制位可以顶替 之前 nums[j] (i < j) 的二进制位, 那么就更新它, 最后我们只就得到最前的出现的1, 并且保证它是最大的, int32 保证我们只需要遍历至多32次, 实际上时间复杂度应该为 $O(N \times log(max(nums)))$ 才标准...

```C++
class Solution {
public:
    vector<int> smallestSubarrays(vector<int>& nums) {
        int maxx = 0, oneCot = 0;
        for (int& it : nums)
            maxx |= it;
        
        vector<int> tmpArr(33, -1);

        for (int i = 0; maxx; maxx >>= 1, ++i) {
            if (maxx & 1) {
                ++oneCot;
            }
        }

        vector<int> res(nums.size());

        for (int i = nums.size() - 1; i >= 0; --i) {
            int it = nums[i], j = 0, right = i;
            for (; it || j < oneCot; it >>= 1, ++j) {
                if (it & 1) {
                    tmpArr[j] = i;
                }

                right = max(right, tmpArr[j]);
            }

            res[i] = right - i + 1;
        }

        return res;
    }
};
```

## 通用秒杀模版 By 0x3f
[利用或运算的性质 + 通用模板（Python/Java/C++/Go）](https://leetcode.cn/problems/smallest-subarrays-with-maximum-bitwise-or/solutions/1830911/by-endlesscheng-zai1)

该模板可以做到

- 求出所有子数组的按位或的结果，以及值等于该结果的子数组的个数。
- 求按位或结果等于任意给定数字的子数组的最短长度/最长长度。

```C++
class Solution {
public:
    vector<int> smallestSubarrays(vector<int> &nums) {
        int n = nums.size();
        vector<int> ans(n);
        vector<pair<int, int>> ors; // 按位或的值 + 对应子数组的右端点的最小值
        for (int i = n - 1; i >= 0; --i) {
            ors.emplace_back(0, i);
            ors[0].first |= nums[i];
            int k = 0;
            for (int j = 1; j < ors.size(); ++j) {
                ors[j].first |= nums[i];
                if (ors[k].first == ors[j].first)
                    ors[k].second = ors[j].second; // 合并相同值，下标取最小的
                else
                  ors[++k] = ors[j];
            }
            ors.resize(k + 1);
            // 本题只用到了 ors[0]，如果题目改成任意给定数字，可以在 ors 中查找
            ans[i] = ors[0].second - i + 1;
        }
        return ans;
    }
};
```

orz, tql, 有空再看qwq..