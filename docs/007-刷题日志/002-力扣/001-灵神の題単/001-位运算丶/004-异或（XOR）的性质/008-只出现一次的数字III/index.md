# 260. 只出现一次的数字 III
链接: [260. 只出现一次的数字 III](https://leetcode.cn/problems/single-number-iii/)

给你一个整数数组 nums，其中恰好有两个元素只出现一次，其余所有元素均出现两次。 找出只出现一次的那两个元素。你可以按 任意顺序 返回答案。

你必须设计并实现线性时间复杂度的算法且仅使用常量额外空间来解决此问题。

# 题解
## 分组
> 脑筋急转弯: 如果 a, b 只出现一次, 那么他们绝对是不同的数, 即某一位上是不同的, 那么`^`后会变成1(相同的为0了), 所以我们求出`lowbit`然后分组, 就变成对这两组进行`^`了

```C++
class Solution {
public:
    vector<int> singleNumber(vector<int>& nums) {
        vector<int> res(2);
        unsigned int xor_all = 0;
        for (int it : nums)
            xor_all ^= it;

        unsigned int lowbit = xor_all & -xor_all;
        for (int it : nums)
            res[!(it & lowbit)] ^= it;

        return res;
    }
};
```

学习: [【图解】一张图秒懂！转换成 136！（Python/Java/C++/Go/JS/Rust）](https://leetcode.cn/problems/single-number-iii/solutions/2484352/tu-jie-yi-zhang-tu-miao-dong-zhuan-huan-np9d2)