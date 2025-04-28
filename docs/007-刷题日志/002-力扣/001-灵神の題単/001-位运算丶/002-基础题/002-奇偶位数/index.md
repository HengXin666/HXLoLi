# 2595. 奇偶位数

链接: [2595. 奇偶位数](https://leetcode.cn/problems/number-of-even-and-odd-bits/)

给你一个 **正** 整数`n`。

用`even`表示在`n`的二进制形式（下标从`0`开始）中值为`1`的偶数下标的个数。

用`odd`表示在`n`的二进制形式（下标从`0`开始）中值为`1`的奇数下标的个数。

返回整数数组`answer`，其中`answer = [even, odd]`。

# 题解
## 1. 基本操作 $O(logn)$

```C++
class Solution {
public:
    vector<int> evenOddBit(int n) {
        vector<int> res(2);
        int w = 0;
        while (n) {
            if (n & 1) {
                ++res[w % 2];
            }
            ++w;
            n >>= 1;
        }
        return res;
    }
}; // By Heng_Xin
```

```C++
class Solution {
public:
    vector<int> evenOddBit(int n) {
        vector<int> ans(2);
        for (int i = 0; n; i ^= 1, n >>= 1)
            ans[i] += n & 1;
        return ans;
    }
};

// 作者：灵茶山艾府
// 链接：https://leetcode.cn/problems/number-of-even-and-odd-bits/solutions/2177848/er-jin-zhi-ji-ben-cao-zuo-pythonjavacgo-o82o2/
// 来源：力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```

## 2. 位掩码 + 库函数
利用位掩码`0x55555555`（二进制的 $010101⋯010101$ ），取出偶数下标比特和奇数下标比特，分别用库函数统计 $1$ 的个数。

本题由于 $n$ 范围比较小，取 0x5555 作为位掩码。

```C++
class Solution {
public:
    vector<int> evenOddBit(int n) {
        const int MASK = 0x5555;
        return {__builtin_popcount(n & MASK), __builtin_popcount(n & (MASK >> 1))};
    }
};

// 作者：灵茶山艾府
// 链接：https://leetcode.cn/problems/number-of-even-and-odd-bits/solutions/2177848/er-jin-zhi-ji-ben-cao-zuo-pythonjavacgo-o82o2/
// 来源：力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```

注: [C/C++中__builtin_popcount()的使用及原理](https://blog.csdn.net/github_38148039/article/details/109598368) (函数功能: 返回输入数据中，二进制中‘1’的个数。)