# 3158. 求出出现两次数字的 XOR 值
链接: [3158. 求出出现两次数字的 XOR 值](https://leetcode.cn/problems/find-the-xor-of-numbers-which-appear-twice/)

给你一个数组 nums ，数组中的数字 要么 出现一次，要么 出现两次。

请你返回数组中所有出现两次数字的按位 XOR 值，如果没有数字出现过两次，返回 0 。

# 题解
## 暴力哈希

```C++
class Solution {
public:
    int duplicateNumbersXOR(vector<int>& nums) {
        int res = 0;
        map<int, int> hsh;
        for (int i : nums)
            hsh[i]++;
        for (auto [a, b] : hsh)
            if (b == 2)
                    res ^= a;
        return res;
    }
};
```

## 0x3f: 位运算集合

```C++
class Solution {
public:
    int duplicateNumbersXOR(vector<int>& nums) { // nums[i] <= 50
        int ans = 0;
        long long vis = 0;
        for (int x : nums) {
            if (vis >> x & 1) { // 已经在集合 -> 第二次出现 -> 加入答案
                ans ^= x;
            } else {
                vis |= 1LL << x; // 第一次出现: 加入集合
            }
        }
        return ans;
    }
};
```
