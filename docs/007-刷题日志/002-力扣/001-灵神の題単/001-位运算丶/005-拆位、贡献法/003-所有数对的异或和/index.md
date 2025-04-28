# 2425. 所有数对的异或和
链接: [2425. 所有数对的异或和](https://leetcode.cn/problems/bitwise-xor-of-all-pairings/)

第 88 场双周赛 Q3 `1622`

给你两个下标从 0 开始的数组 nums1 和 nums2 ，两个数组都只包含非负整数。请你求出另外一个数组 nums3 ，包含 nums1 和 nums2 中 所有数对 的异或和（nums1 中每个整数都跟 nums2 中每个整数 恰好 匹配一次）。

请你返回 nums3 中所有整数的 异或和 。

# 题解
## 位运算数学化简(贡献法?)

对于朴素的 $O(n^2)$ 算法: (枚举每一个数对然后异或和)

```C++
class Solution {
public:
    int xorAllNums(vector<int>& nums1, vector<int>& nums2) {
        int res = 0;
        for (int i = 0; i < nums1.size(); ++i)
            for (int j = 0; j < nums2.size(); ++j)
                res ^= nums1[i] ^ nums2[j];
        return res;
    }
};
```

有: $$res = (A[0] \wedge B[0]) \wedge (A[0] \wedge B[1]) \wedge \dots \wedge (A[0] \wedge B[B.size -1]) \wedge \\ (A[1] \wedge B[0]) \wedge (A[1] \wedge B[1]) \wedge \dots (A[1] \wedge B[B.size - 1]) \wedge \\ \dots \wedge \\ (A[A.size - 1] \wedge B[0]) \wedge (A[A.size - 1] \wedge B[1]) \wedge \dots (A[A.size - 1] \wedge B[B.size - 1]) \\ \ \\ = (\bigoplus\limits_{i=0}^{A.size - 1}\underbrace{A[i] \wedge A[i] \wedge \dots \wedge A[i]}_{B.siez个A[i]}) \wedge (\bigoplus\limits_{i=0}^{B.size - 1}\underbrace{B[i] \wedge B[i] \wedge \dots \wedge B[i]}_{A.siez个B[i]})$$

(上面实际上就是朴素算法的展开式), 我们由异或的性质: $a \wedge a = 0$ 可以得知 $$res = [(B.size \& 1) \times (\bigoplus\limits_{i=0}^{A.size - 1}A[i])] \wedge [(A.size \& 1) \times (\bigoplus\limits_{i=0}^{B.size - 1}B[i])]$$

注: $\times (A.size \& 1)$ 只是为了表达奇偶性, 当其为`偶`时( $a \wedge a = 0$ ) 可以不计算后面的异或和.

```C++
class Solution {
public:
    int xorAllNums(vector<int>& nums1, vector<int>& nums2) {
        int res = 0;
        if (nums1.size() & 1)
            for (int i : nums2)
                res ^= i;
        if (nums2.size() & 1)
            for (int j : nums1)
                res ^= j;
        return res;
    }
};
```
