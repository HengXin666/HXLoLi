# 342. 4的幂
链接: [342. 4的幂](https://leetcode.cn/problems/power-of-four/)

给定一个整数，写一个函数来判断它是否是 4 的幂次方。如果是，返回 true ；否则，返回 false 。

整数 n 是 4 的幂次方需满足：存在整数 x 使得 n == $4^x$

# 题解
## 0. 二的幂 && 单独查找一的位置

```C++
class Solution {
public:
    bool isPowerOfFour(int n) {
        if (n <= 0 || (n & (n - 1)))
            return false;
        
        int res = 0;
        while (n) {
            if (n & 1)
                return !(res & 1);
            ++res;
            n >>= 1;
        }
        return 0;
    }
};
```
而查询一的位置可以使用`位掩码`来达到一个 $O(1)$ 的时间复杂度

## 1. 二的幂 && 位掩码

```C++
class Solution {
public:
    // 0101 -> 1 + 4 == 5 (因为 n > 0 由短路求值, 所以不用担心)
    bool isPowerOfFour(int n) {
        const int wym = 0x55'55'55'55;
        return n > 0 && !(n & (n - 1)) && (n & wym);
    }
};
```

## 2. 二的幂 && 模

如果 $n$ 是 4 的幂，那么它可以表示成 $4^x$ 的形式，我们可以发现它除以 3 的余数一定为 1，即:

$$4x≡(3+1)x≡1x≡1( mod  3)$$

如果 $n$ 是 2 的幂却不是 4 的幂，那么它可以表示成 $4^x \times 2$ 的形式，此时它除以 3 的余数一定为 2。

因此我们可以通过 $n$ 除以 3 的余数是否为 1 来判断 $n$ 是否是 4 的幂。

```C++
class Solution {
public:
    bool isPowerOfFour(int n) {
        return n > 0 && !(n & (n - 1)) && n % 3 == 1;
    }
};
```