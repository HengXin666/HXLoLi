# 2680. 最大或值

链接: [2680. 最大或值](https://leetcode.cn/problems/maximum-or/)


第 104 场双周赛 Q3 `1912`

给你一个下标从 0 开始长度为 n 的整数数组 nums 和一个整数 k 。每一次操作中，你可以选择一个数并将它乘 2 。

你最多可以进行 k 次操作，请你返回 `nums[0] | nums[1] | ... | nums[n - 1]` 的最大值。

a | b 表示两个整数 a 和 b 的 按位或 运算。

## 题解
### 我的超时
1. 我思考到了 需要把`k`集中到一个数上移动, 但是我不确定是哪一个, 所以我打算把全部比特位长度最大的全部尝试一次, 显然 $O(n^2)$ 原点爆炸!

```C++
class Solution {
public:
    long long maximumOr(vector<int>& nums, int k) {
        // 可以选择任何一个数进行 nums[i] << 1 的运算, 并且可以多次
        // 选择最大的数然后 << ?
        unordered_multiset<int> arr; // 可疑的数字
        int maxH = 0;    // 最高位的长度
        for (int& it : nums) {
            if (int tmp_len = 32-__builtin_clz(it); tmp_len >= maxH) {
                if (tmp_len > maxH) {
                    arr.clear();
                    maxH = tmp_len;
                }
                arr.insert(it);
            }
        }

        long long res = 0; // 除去最高位的数的 |
        for (int& it : nums)
            if (!arr.count(it))
                res |= it;

        long long resMax = res;
        for (auto it = arr.begin(); it != arr.end(); ++it) {
            long long tmp = res | ((long long)(*it) << k);
            for (auto jt = arr.begin(); jt != arr.end(); ++jt) {
                if (it != jt) {
                    tmp |= *jt;
                }
            }
            resMax = max(resMax, tmp);
        }

        return resMax;
    }
};

/*
12 - 01100 (需要: 移动后使得最少的1重合)
09 - 01001

实际上要求最大, 就是贪心的, 把最高位为最左边的数<<k 即可,
对于是谁, 就一一尝试得了

11000 - 12 * 2 == 24
01001 - 9
|
11101 - 24 + 9 =

10010
01100
11110
*/
```

## AC
我就看了一下题目的提示2:
> 计算前缀 or 和后缀 or 并对每个元素执行 k 次运算，并最大化答案。

wdf!! 前后缀分解直接秒了!! 原来题目就是一个贪心?!!? (前后缀我怎么没有想到?!!?!?!?!?)

```C++
class Solution {
public:
    long long maximumOr(vector<int>& nums, int k) {
        // 可以选择任何一个数进行 nums[i] << 1 的运算, 并且可以多次
        // 选择最大的数然后 << ?
        
        // 前后缀分解!
        const int n = nums.size();
        vector<int> usiro(n + 1);
        for (int i = n - 1; i >= 0; --i) {
            usiro[i] = usiro[i + 1] | nums[i];
        }

        long long res = 0;
        for (int i = 0, mae = 0; i < n; ++i) {
            res = max(res, mae | usiro[i + 1] | ((long long)nums[i]) << k);
            mae |= nums[i];
        }

        return res;
    }
};
```
