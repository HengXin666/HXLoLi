# 2275. 按位与结果大于零的最长组合
链接: [2275. 按位与结果大于零的最长组合](https://leetcode.cn/problems/largest-combination-with-bitwise-and-greater-than-zero/)

第 293 场周赛 Q3 `1642`

对数组 nums 执行 按位与 相当于对数组 nums 中的所有整数执行 按位与 。

- 例如，对 nums = [1, 5, 3] 来说，按位与等于 1 & 5 & 3 = 1 。
- 同样，对 nums = [7] 而言，按位与等于 7 。

给你一个正整数数组 candidates 。计算 candidates 中的数字每种组合下 按位与 的结果。 candidates 中的每个数字在每种组合中只能使用 一次 。

返回按位与结果大于 0 的 最长 组合的长度。

# 题解
## 拆位
实际上就是问对于每一位的 1 的数量最多:

```C++
1000
1001
0101
1010

// 输出是 3
只看 最左边那列, 你数一下 1 的个数
```
有点脑筋急转弯? 实际上是拆位!

```C++
class Solution {
public:
    int largestCombination(vector<int>& candidates) {
        // 最长子序列 & 后非 0, 求bit某一位&后为1最长
        int res = 0;
        // 10^7 ~ 一千万元整 ~ 2^24=16777216
        for (int i = 0; i < 24; ++i) {
            int len = 0;
            for (int it : candidates)
                len += it >> i & 1;
            res = max(res, len);
        }
        return res;
    }
};
```
