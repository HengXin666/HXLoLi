# 338. 比特位计数
链接: [338. 比特位计数](https://leetcode.cn/problems/counting-bits/)

给你一个整数 n ，对于 0 <= i <= n 中的每个 i ，计算其二进制表示中 1 的个数 ，返回一个长度为 n + 1 的数组 ans 作为答案。

是否有 $O(n)$ 解法?

# 题解
## $O(nlogn)$

```C++
class Solution {
public:
    vector<int> countBits(int n) {
        vector<int> res(n + 1);
        for (int i = 1; i <= n; ++i) {
            int w = 0, tmp = i;
            while (tmp) {
                ++w;
                tmp &= tmp - 1; // 快速得到下一个一的位置, 最坏情况下是 1111 1111 这样, 所以需要 logn
            }
            res[i] = w; 
        }
        return res;
    }
};
```

## 动态规划 $O(n)$ 解法

```C++
class Solution {
public:
    vector<int> countBits(int n) {
        vector<int> res(n + 1);
        // 通过观察, 有 状态转移方程
        // 奇数 = 偶数(奇数 - 1) 的 1 的个数 + 1
        // 偶数 = (偶数 >> 1) 的 1 的个数 (就是把最低为去掉罢了)
        for (int i = 1; i <= n; ++i) {
            res[i] = i & 1 ? res[i - 1] + 1 : res[i >> 1];
        }
        return res;
    }
};
```