# 3201. 找出有效子序列的最大长度 I
链接: [3201. 找出有效子序列的最大长度 I](https://leetcode.cn/problems/find-the-maximum-length-of-valid-subsequence-i/)

给你一个整数数组 nums。

nums 的子序列 sub 的长度为 x ，如果其满足以下条件，则称其为 有效子序列：

- `(sub[0] + sub[1]) % 2 == (sub[1] + sub[2]) % 2 == ... == (sub[x - 2] + sub[x - 1]) % 2`

返回 nums 的 最长的有效子序列 的长度。

一个 子序列 指的是从原数组中删除一些元素（也可以不删除任何元素），剩余元素保持原来顺序组成的新数组。

- 2 <= nums.length <= 2 * 10^5
- 1 <= nums[i] <= 10^7

# 题解
## 暴力可能

```C++
class Solution {
public:
    int maximumLength(vector<int>& nums) {
        int n = nums.size(), res = 0, len = 0;
        // 双, 双
        for (int i = 0, mae = -1; i < n; ++i) {   
            if (mae != -1) {
                if (nums[i] % 2 == 0) {
                    mae = i;
                    ++len;
                }
            } else
                if (nums[i] % 2 == 0)
                    mae = i;
        }
        res = max(res, len);
        len = 0;

        // 单, 单
        for (int i = 0, mae = -1; i < n; ++i) {
            if (mae != -1) {
                if (nums[i] % 2 == 1) {
                    mae = i;
                    ++len;
                }
            } else
                if (nums[i] % 2 == 1)
                    mae = i;
        }
        res = max(res, len);len = 0;

        // 单, 双
        for (int i = 0, mae = -1, tag = 1; i < n; ++i) {
            if (mae != -1) {
                if (nums[i] % 2 == tag) {
                    mae = i;
                    ++len;
                    tag = !tag;
                }
            } else {
                if (nums[i] % 2 == tag) {
                mae = i;
                tag = !tag;
            }
            }
        }
        res = max(res, len);len = 0;

        // 双, 单
        for (int i = 0, mae = -1, tag = 0; i < n; ++i) {
            if (mae != -1) {
                if (nums[i] % 2 == tag) {
                    mae = i;
                    ++len;
                    tag = !tag;
                }
            } else {
                if (nums[i] % 2 == tag) {
                mae = i;
                tag = !tag;
            }
            }
        }
        res = max(res, len);

        return res + 1;
    }
};
```

## dp
见Q3题解