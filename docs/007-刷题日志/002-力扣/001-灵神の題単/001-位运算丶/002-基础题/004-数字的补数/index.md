# 476. 数字的补数

- 链接: [476. 数字的补数](https://leetcode.cn/problems/number-complement/)
- 基本类似: [1009. 十进制整数的反码](https://leetcode.cn/problems/complement-of-base-10-integer/)

---

对整数的二进制表示取反（0 变 1 ，1 变 0）后，再转换为十进制表示，可以得到这个整数的补数。

例如，整数 5 的二进制表示是 "101" ，取反后得到 "010" ，再转回十进制表示得到补数 2 。

给你一个整数 num ，输出它的补数。

# 代码
## 我的

```C++
class Solution {
public:
    int findComplement(int num) {
        int res = 0;
        int w = 0; // 当前是第几位
        while (num) {
            if (!(num & 1)) { // 如果当前为 0, 那么我们就是 1
                res |= 1 << w;
            }
            ++w;
            num >>= 1;
        }
        return res;
    }
};
```

## $O(1)$ ?

```C++
class Solution {
public:
    int findComplement(int num) {
        int t = num;
        t |= t >> 1;
        t |= t >> 2;
        t |= t >> 4;
        t |= t >> 8;
        t |= t >> 16;
        return ~num & t;
    }
};
```
