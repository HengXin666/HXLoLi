# 461. 汉明距离
链接: [461. 汉明距离](https://leetcode.cn/problems/hamming-distance/)

两个整数之间的 汉明距离 指的是这两个数字对应二进制位不同的位置的数目。

给你两个整数 x 和 y，计算并返回它们之间的汉明距离。

# 题解
## 朴素版本

模拟判断每一位, `!=`号, 可以使用`^`代替

```C++
class Solution {
public:
    int hammingDistance(int x, int y) {
        int res = 0;
        while (x || y) {
            if ((x & 1) ^ (y & 1))
                ++res;
            x >>= 1;
            y >>= 1;
        }
        return res;
    }
};
```

## 朴素版本的优化版: $Brian Kernighan$ 算法

因为`^`就是使得`不同的位`变为`1`的, 而相同的位就会变为`0`, 故此直接统计即可.

```C++
class Solution {
public:
    int hammingDistance(int x, int y) {
        x ^= y;
        y = 0;
        while (x) {
            x &= x - 1;
            ++y;
        }
        return y;
    }
};
```

## 库函数

直接就是 $O(1)$ 的, 具体见[2595. 奇偶位数](../002-奇偶位数/index.md)/[191. 位1的个数](../005-位的个数/index.md)有推导

```C++
class Solution {
public:
    int hammingDistance(int x, int y) {
        return __builtin_popcount(x ^ y);
    }
};
```