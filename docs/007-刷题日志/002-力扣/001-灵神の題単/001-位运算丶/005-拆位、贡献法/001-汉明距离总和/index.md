# 477. 汉明距离总和
链接: [477. 汉明距离总和](https://leetcode.cn/problems/total-hamming-distance/)

两个整数的 汉明距离 指的是这两个数字的二进制数对应位不同的数量。

给你一个整数数组 nums，请你计算并返回 nums 中任意两个数之间 汉明距离的总和 。

数据范围: `nums.size() <= 10^4`

# 题解
## 暴力统计

必需使用库函数!
```C++
class Solution {
public:
    int totalHammingDistance(vector<int>& nums) {
        int res = 0;
        for (int i = 0; i < nums.size(); ++i)
            for (int j = i + 1; j < nums.size(); ++j)
                res += __builtin_popcount(nums[i] ^ nums[j]);
        return res;
    }
};
```

## 拆位

我们可以对每一位进行讨论, 因为统计的是每一位的数, 而不是每个数!

显然对于第i位的不同的个数就是 $cnt_0 \times cnt_1$

如图: <img src="https://pic.leetcode-cn.com/1622166123-MiinFf-image.png" alt="image.png">

```C++
auto optimize = []() { // 这个快读可以提速!
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);
    return 0;
}();

class Solution {
public:
    int totalHammingDistance(vector<int>& nums) {
        int res = 0;
        // 乘法原理: 拆位
        for (int i = 0; i < 31; ++i) {
            int cnt = 0;
            for (int it : nums)
                cnt += (it >> i) & 1;
            res += cnt * (nums.size() - cnt);
        }
        return res;
    }
};
```
