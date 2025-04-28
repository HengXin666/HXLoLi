# 137. 只出现一次的数字 II
链接: [137. 只出现一次的数字 II](https://leetcode.cn/problems/single-number-ii/)

给你一个整数数组 nums ，除某个元素仅出现 一次 外，其余每个元素都恰出现 三次 。请你找出并返回那个只出现了一次的元素。

你必须设计并实现线性时间复杂度的算法且使用常数级空间来解决此问题。

# 题解
## 统计1的个数

学习直达: [带你一步步推导出位运算公式！（Python/Java/C++/Go/JS/Rust）](https://leetcode.cn/problems/single-number-ii/solutions/2482832/dai-ni-yi-bu-bu-tui-dao-chu-wei-yun-suan-wnwy)

```C++
class Solution {
public:
    int singleNumber(vector<int>& nums) {
        // 统计1的个数
        int res = 0;
        for (int i = 0; i < 32; ++i) {
            int cnt = 0;
            for (int it : nums)
                cnt += it >> i & 1;
            res |= (cnt % 3) << i;
        }
        return res;
    }
};
```

上面这个是一位一位的, 而使用位运算可以`并行`, 只需要使用位运算写出对3取模的代码就OK

```C++
class Solution {
public:
    int singleNumber(vector<int> &nums) {
        int a = 0, b = 0;
        for (int x: nums) {
            int tmp_a = a;
            a = (a ^ x) & (a | b);
            b = (b ^ x) & ~tmp_a;
        }
        return b;
    }
}; // By 0x3f
```

```C++
class Solution {
public:
    int singleNumber(vector<int> &nums) {
        int a = 0, b = 0;
        for (int x: nums) {
            b = (b ^ x) & ~a;
            a = (a ^ x) & ~b;
        }
        return b;
    }
}; // By 0x3f
```