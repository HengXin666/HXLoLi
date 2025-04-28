# 1017. 负二进制转换
链接: [1017. 负二进制转换](https://leetcode.cn/problems/convert-to-base-2/)


第 130 场周赛 Q2 `1698`

给你一个整数 n ，以二进制字符串的形式返回该整数的 负二进制（base -2）表示。

注意，除非字符串就是 "0"，否则返回的字符串中不能含有前导零。

示例 1：
- 输入：n = 2
- 输出："110"
- 解释： $(-2)^2 + (-2)^1 = 2$

示例 2：
- 输入：n = 3
- 输出："111"
- 解释： $(-2)^2 + (-2)^1 + (-2)^0 = 3$

示例 3：
- 输入：n = 4
- 输出："100"
- 解释： $(-2)^2 = 4$

提示：

0 <= n <= $10^9$

# 题解
## 模拟

```C++
class Solution {
public:
    string baseNeg2(int n) {
        if (n == 0) {
            return "0";
        }
        vector<int> bits(32);
        for (int i = 0; i < 32 && n != 0; i++) {
            if (n & 1) {
                bits[i]++;
                if (i & 1) {
                    bits[i + 1]++;
                }
            }
            n >>= 1;
        }
        int carry = 0;
        for (int i = 0; i < 32; i++) {
            int val = carry + bits[i];
            bits[i] = val & 1;
            carry = (val - bits[i]) / (-2);
        }
        int pos = 31;
        string res;
        while (pos >= 0 && bits[pos] == 0) {
            pos--;
        }
        while (pos >= 0) {
            res.push_back(bits[pos] + '0');
            pos--;
        }
        return res;
    }
};
```

## 短除法

适用于任何非0进制转换:
```C++
class Solution {
    // 需要保证 k != 0
    string baseK(int N, int K) {
        if (!N) 
            return "0";
        string res;
        while (N) {
            int r = N % K; // 获取余数
            if (r < 0) // 保证余数为正数
                r = (r + abs(K)) % abs(K);
            res = std::to_string(r) + res; // 前加就无需翻转
            N = (N - r) / K; // 使得其整除, 而不是向零取整
        }
        return move(res);
    }

public:
    string baseNeg2(int x) {
        return move(baseK(x, -2));
    }
};
```

## 数学

```C++
class Solution {
public:
    string baseNeg2(int n) {
        int val = 0x55555555 ^ (0x55555555 - n);
        if (val == 0) {
            return "0";
        }
        string res;
        while (val) {
            res.push_back('0' + (val & 1));
            val >>= 1;
        }
        reverse(res.begin(), res.end());
        return res;
    }
};
```
