# 1829. 每个查询的最大异或值
链接: [1829. 每个查询的最大异或值](https://leetcode.cn/problems/maximum-xor-for-each-query/)


第 50 场双周赛 Q3 `1523`

给你一个 有序 数组 `nums` ，它由 `n` 个非负整数组成，同时给你一个整数 `maximumBit` 。你需要执行以下查询 `n` 次:

1. 找到一个非负整数 $k < 2^{maximumBit}$，使得 `nums[0] XOR nums[1] XOR ... XOR nums[nums.length-1] XOR k` 的结果 **最大化** 。`k` 是第 `i` 个查询的答案。

2. 从当前数组 `nums` 删除 **最后** 一个元素。

请你返回一个数组 `answer` ，其中 `answer[i]`是第 `i` 个查询的结果。

# 题解
## 我的

时间复杂度: $O(n \times maximumBit)$

```C++
class Solution {
public:
    vector<int> getMaximumXor(
        vector<int>& nums, int maximumBit) {
        int sumNum = 0;
        for (int& it : nums)
            sumNum ^= it;
        
        vector<int> res(nums.size());
        // 使得异或最大
/*
00110 - sumNum
11001 - i

只需找到一个小于 2^m 的数, 使得异或上的结果 部分全部为 1
即 这个被找到的二进制数长度为 maximumBit
*/
        for (int i = 0; i < res.size(); ++i) {
            // 遍历sumNum的低位
            int tmp = sumNum, len = maximumBit, now_bit_w = 0;
            while (now_bit_w < len) {
                if (!(tmp & 1)) {
                    res[i] |= 1 << now_bit_w;
                }
                ++now_bit_w;
                tmp >>= 1;
            }

            sumNum ^= *nums.rbegin();
            nums.pop_back();
        }

        return res;
    }
};
```

## 优化

1. 改变遍历方式, 可以节省一次 $O(n)$

2. 对于 上面`while`遍历位求解`k`的环节, 我们可以有 $O(1)$ 的做法
    - 首先`c = (1 >> maximumBit) - 1`得到低位的掩码(0b11...11 (maximumBit个1))
    - 然后通过`sumNum & c`可以得到需要改变的低`maximumBit`位,
    - 然后`c - (sumNum & c)`即可! `1变0, 0变1`
```C++
class Solution {
public:
    vector<int> getMaximumXor(
        vector<int>& nums, int maximumBit) {
        int sumNum = 0;
        
        const int n = nums.size() - 1;
        vector<int> res(n + 1);
        for (int i = n, c = (1 << maximumBit) - 1; i >= 0; --i) {
            sumNum ^= nums[n - i];
            res[i] = c - (sumNum & c);
        }

        return move(res);
    }
};
```
