# 1486. 数组异或操作
链接: [1486. 数组异或操作](https://leetcode.cn/problems/xor-operation-in-an-array/)

---

给你两个整数，n 和 start 。

数组`nums`定义为：`nums[i] = start + 2*i`（下标从 0 开始）且 `n == nums.length`。

请返回`nums`中所有元素按位异或（XOR）后得到的结果。

# 题解
## 1. 模拟

```C++
class Solution {
public:
    int xorOperation(int n, int start) {
        int res = 0;
        for (int i = 0; i < n; ++i) {
            res ^= start + (i * 2);
        }

        return res;
    }
};
```

## 2. 数学
记 $\oplus$ 为异或运算，异或运算满足以下性质:

$
x \oplus x=0 \\
{x \oplus y=y \oplus x \quad(\text { 交换律 }) ；} \\
{(x \oplus y) \oplus z=x \oplus(y \oplus z) （ \text { 结合律 }) ；} \\
{x \oplus y \oplus y=x （ \text { 自反性 } ） ；}
$

---

$
\forall i \in Z \text { ，有 } 4 i \oplus(4 i+1) \oplus(4 i+2) \oplus(4 i+3)=0 \text { 。 }
$

---

[异或-图解推导版本(小明做数学)](https://leetcode.cn/problems/xor-operation-in-an-array/solutions/761845/xiao-ming-zuo-shu-xue-by-xiaohu9527-0pu7)(省流版本)

下面是力扣官方题解:

在本题中，我们需要计算 $\operatorname{start} \oplus(  start  +2 i) \oplus(  start  +4 i) \oplus \cdots \oplus(  start  +2(n-1))$ 。
观察公式可以知道，这些数的奇偶性质相同，因此它们的二进制表示中的最低位或者均为 1 ，或者均为 0 。于是我们可以把参与运算的数的二进制位的最低位提取出来单独处理。当且仅当 $start$ 为奇数，且 $n$ 也为奇数时，结果的二进制位的最低位才为 1 。

此时我们可以将公式转化为 $(s \oplus(s+1) \oplus(s+2) \oplus \cdots \oplus(s+n-1)) \times 2+e$ ，其中 $s=\left\lfloor\frac{\text { start }}{2}\right\rfloor$ ， $e$ 表示运算结果的最低位。即我们单独处理最低位，而舍去最低位后的数列恰成为一串连续的整数。

这样我们可以描述一个函数 $\operatorname{sum} \operatorname{Xor}(x)$ ，表示 $0 \oplus 1 \oplus 2 \oplus \cdots \oplus x$ 。利用异或运算的性质 5 ，我们可以将计算该函数的复杂度降低到 $O(1)$ ，因为以 $4i$ 为开头的连续四个整数异或的结果为 0 ，所以 $\operatorname{sumXor}(x)$ 可以被表示为:

$$sumXor(x)=\begin{cases} x,& x=4k,k\in Z\\ (x-1) \oplus x,& x=4k+1,k\in Z\\ (x-2) \oplus (x-1) \oplus x,& x=4k+2,k\in Z\\ (x-3) \oplus (x-2) \oplus (x-1) \oplus x,& x=4k+3,k\in Z\\ \end{cases}$$

可以进一步化简为:

$$sumXor(x)= \begin{cases} x,& x=4k,k\in Z\\ 1,& x=4k+1,k\in Z\\ x+1,& x=4k+2,k\in Z\\ 0,& x=4k+3,k \in Z\\ \end{cases}$$

这样最后的结果即可表示为 $(sumXor(s−1)⊕sumXor(s+n−1))×2+e$。

```C++
class Solution {
public:
    int sumXor(int x) {
        if (x % 4 == 0) {
            return x;
        }
        if (x % 4 == 1) {
            return 1;
        }
        if (x % 4 == 2) {
            return x + 1;
        }
        return 0;
    }

    int xorOperation(int n, int start) {
        int s = start >> 1, e = n & start & 1;
        int ret = sumXor(s - 1) ^ sumXor(s + n - 1);
        return ret << 1 | e;
    }
};
```
