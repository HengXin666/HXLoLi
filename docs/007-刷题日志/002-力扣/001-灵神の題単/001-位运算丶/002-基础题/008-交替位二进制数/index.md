# 693. 交替位二进制数

链接: [693. 交替位二进制数](https://leetcode.cn/problems/binary-number-with-alternating-bits/)

给定一个正整数，检查它的二进制表示是否总是 0、1 交替出现：换句话说，就是二进制表示中相邻两位的数字永不相同。

## 题解
### 模拟

```C++
class Solution {
public:
    bool hasAlternatingBits(int x) {
        bool mae = !(x & 1);
        while (x) {
            if (!((x & 1) ^ mae))
                return false;
            mae = x & 1;
            x >>= 1;
        }

        return true;
    }
};
```

### 位运算

```C++
class Solution {
public:
    bool hasAlternatingBits(int x) {
        long a = x ^ (x >> 1);
        return !(a & (a + 1));
    }
};

/*
解析:
1010101
 >> 1
0101010
^
0000000

100
>> 1
010
^
001 no
*/
```
