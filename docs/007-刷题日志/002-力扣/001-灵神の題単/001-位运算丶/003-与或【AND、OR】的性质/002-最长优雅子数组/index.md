# 2401. 最长优雅子数组
链接: [2401. 最长优雅子数组](https://leetcode.cn/problems/longest-nice-subarray/)


第 309 场周赛 Q3 `1750`

给你一个由 正 整数组成的数组 nums 。

如果 nums 的子数组中位于 不同 位置的每对元素按位 与（AND）运算的结果等于 0 ，则称该子数组为 优雅 子数组。

返回 最长 的优雅子数组的长度。

子数组 是数组中的一个 连续 部分。

注意：长度为 1 的子数组始终视作优雅子数组。

# 题解
## 我的

暴力判断 + 滑动窗口

```C++
class Solution {
public:
    int longestNiceSubarray(vector<int>& nums) {
        // o(n)
        int res = 0;
        const int n = nums.size();
        for (int l = 0, r = 0; r < n; ++r) {  
            A:
            for (int i = l; i < r; ++i) {
                if (nums[i] & nums[r]) {
                    ++l;
                    goto A;
                }
            }

            res = max(res, r - l + 1);
        }

        return res;
    }
};
```

## 暴力
因为最多只能用 30 个元素 (1, 2, 4, 8, 16, ..., 2^30), 因此可以暴力:

小技巧: 
- 判断
    - nums[l] & nums[r] == 0,
    - nums[l + 1] & nums[r] == 0,
    - ...,
    - nums[r - 1] & nums[r] == 0;
- 等价于判断
    - ( nums[l] | nums[l + 1] | ... | nums[r - 1] ) & nums[r] == 0

```C++
class Solution {
public:
    int longestNiceSubarray(vector<int>& nums) {
        int res = 0;
        const int n = nums.size();
        for (int i = 0; i < n; ++i) {
            int j = i;
            int tmp = 0;
            while (j >= 0 && !(nums[j] & tmp)) {
                tmp |= nums[j--];
            }
            res = max(res, i - j);
        }

        return res;
    }
};
```

## 滑动窗口

`&^|`都出现了...

*削寄窍:*

当 $x \& y = 0$ 时, 有 $k = x | y$, 而 $x = k \wedge y, y = k \wedge x$

即:

```C++
因为 010101 & 101000 == 0

  010101 (x)
| 101000 (y)
---------
  111101 (k)

  111101 (k)
^ 101000 (y)
---------
  010101 (x)
```


```C++
class Solution {
public:
    int longestNiceSubarray(vector<int>& nums) {
        int res = 0;
        const int n = nums.size();
        for (int l = 0, r = 0, tmp = 0; r < n; ++r) {
            while (tmp & nums[r])
                tmp ^= nums[l++];
    
            tmp |= nums[r];

            res = max(res, r - l + 1);
        }

        return res;
    }
};
```
