# 191. 位1的个数
链接: [191. 位1的个数](https://leetcode.cn/problems/number-of-1-bits/)

编写一个函数，输入是一个无符号整数（以二进制串的形式），返回其二进制表达式中 **设置位**(`set bit`指在某数的二进制表示中值为 1 的二进制位。) 的个数（也被称为汉明重量）。

例如:

```C++
输入：n = 128
输出：1
解释：输入的二进制串 10000000 中，共有 1 个设置位。
```

# 代码
## 朴素-统计1的个数

```C++
class Solution {
public:
    int hammingWeight(int n) {
        int res = 0;
        while (n) {
            if (n & 1)
                ++res;
            n >>= 1;
        }
        return res;
    }
};
```

## 优化-快速统计1

对于 $n \& (n - 1)$ 这个式子, 它恰好可以把 n 的最低位的 $1$ 去掉.

时间复杂度: $O(count(1))$ 即 $n$ 的 $1$ 的个数.

```C++
class Solution {
public:
    int hammingWeight(int n) {
// 1010
// 1001
// &
// 1000
        int res = 0;
        while (n) {
            ++res;
            n &= n - 1;
        }
        return res;
    }
};
```

## 最优化-合并计数器的策略
参考链接: [【转】对于任意的非负整数，统计其二进制展开中数位1的总数](https://blog.csdn.net/weixin_30587927/article/details/101093369)

实际上还是二分的思想，把一位一位计数变成二分的计数，使 $O(logn)$ 变成了 $O(loglogn)$。

```C++
int BitCount4(unsigned int n) {
    n = (n & 0x55555555) + ((n >> 1) & 0x55555555); 
    n = (n & 0x33333333) + ((n >> 2) & 0x33333333); 
    n = (n & 0x0f0f0f0f) + ((n >> 4) & 0x0f0f0f0f); 
    n = (n & 0x00ff00ff) + ((n >> 8) & 0x00ff00ff); 
    n = (n & 0x0000ffff) + ((n >> 16) & 0x0000ffff); 
    return n; 
}
```