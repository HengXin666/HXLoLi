# 3146. 两个字符串的排列差
链接: [3146. 两个字符串的排列差](https://leetcode.cn/problems/permutation-difference-between-two-strings/)

给你两个字符串 s 和 t，每个字符串中的字符都不重复，且 t 是 s 的一个排列。

排列差 定义为 s 和 t 中每个字符在两个字符串中位置的绝对差值之和。

返回 s 和 t 之间的 排列差 。

# 题解
## HX 暴力
```C++
class Solution {
public:
    int findPermutationDifference(string s, string t) {
        int res = 0;
        for (int i = 0; i < s.size(); ++i) {
            for (int j = 0; j < s.size(); ++j) {
                if (t[j] == s[i]) {
                    res += abs(i - j);
                    break;
                }
            }
        }
        return res;
    }
};
```

## 0x3f 哈希表 O(n)

```C++
class Solution {
public:
    int findPermutationDifference(string s, string t) {
        int pos[26];
        for (int i = 0; i < s.size(); i++) {
            pos[s[i] - 'a'] = i;
        }
        int ans = 0;
        for (int i = 0; i < t.size(); i++) {
            ans += abs(i - pos[t[i] - 'a']);
        }
        return ans;
    }
};
```
