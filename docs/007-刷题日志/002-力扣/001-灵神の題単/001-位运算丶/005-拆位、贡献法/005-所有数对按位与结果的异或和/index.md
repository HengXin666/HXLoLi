# 1835. 所有数对按位与结果的异或和
链接: [1835. 所有数对按位与结果的异或和](https://leetcode.cn/problems/find-xor-sum-of-all-pairs-bitwise-and/)

第 237 场周赛 Q4 `1825`

列表的 异或和（XOR sum）指对所有元素进行按位 XOR 运算的结果。如果列表中仅有一个元素，那么其 异或和 就等于该元素。

- 例如，[1,2,3,4] 的 异或和 等于 1 XOR 2 XOR 3 XOR 4 = 4 ，而 [3] 的 异或和 等于 3 。

给你两个下标 从 0 开始 计数的数组 arr1 和 arr2 ，两数组均由非负整数组成。

根据每个 (i, j) 数对，构造一个由 arr1[i] AND arr2[j]（按位 AND 运算）结果组成的列表。其中 `0 <= i < arr1.length` 且 `0 <= j < arr2.length` 。

返回上述列表的 **异或和** 。

# 题解
## 拆位

本质实际上是 `(a & b) ^ (a & c) == a & (b ^ c)`

```C++
class Solution {
public:
    int getXORSum(vector<int>& arr1, vector<int>& arr2) {
        int res = 0;
        // 预处理arr2的每一位的1的数量
        vector<int> arr2bit(31);
        for (int it : arr2) {
            for (int i = 0; i < 31; ++i)
                arr2bit[i] += it >> i & 1;
        }
        int bit = 0;
        for (int i = 0; i < 31; ++i)
            bit |= (arr2bit[i] & 1) * (1 << i);

        for (int it : arr1) {
            res ^= it & bit;
        }
        return res;
    }
};
```
即:
```C++
int xorArr(const vector<int>& arr) {
    int res = 0;
    for (int it : arr)
        res ^= it;
    return res;
}

class Solution {
public:
    int getXORSum(vector<int>& arr1, vector<int>& arr2) {
        // (a & b) ^ (a & c) == a & (b ^ c)
        // (a_0 & (xor b_arr)) ^ (a_1 & (xor b_arr)) ^ ..
        // (xor b_arr) & (a_0 ^ a_1 ^ .. ^ a_n)
        // (xor a_arr) & (xor b_arr)
        return xorArr(arr1) & xorArr(arr2);
    }
};
```