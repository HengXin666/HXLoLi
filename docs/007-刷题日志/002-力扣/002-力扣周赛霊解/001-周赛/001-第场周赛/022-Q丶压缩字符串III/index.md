# 3163. 压缩字符串 III
链接: [3163. 压缩字符串 III](https://leetcode.cn/problems/string-compression-iii/)

给你一个字符串 word，请你使用以下算法进行压缩：

- 从空字符串 comp 开始。当 word 不为空 时，执行以下操作：
    - 移除 word 的最长单字符前缀，该前缀由单一字符 c 重复多次组成，且该前缀长度 **最多** 为 9 。
    - 将前缀的长度和字符 c 追加到 comp 。

返回字符串 comp .

示例:

```C++
输入：word = "aaaaaaaaaaaaaabb"
输出："9a5a2b"
```

提示:
- 1 <= word.length <= 2 * 10^5
- word 仅由小写英文字母组成。

# 题解
## 模拟

```C++
class Solution {
public:
    string compressedString(string s) {
        string res;
        int i = 0, l = 0;
        for (; i < s.size(); ++i) {
            if (i - l < 9 && s[l] == s[i]) {
                
            } else {
                res += to_string(i - l) + s[l];
                l = i;
            }
        }
        
        if (i != l) {
            res += to_string(i - l)  + s[l];
        }
        
        return res;
    }
};
```

## 0x3f: 模拟

```C++
class Solution {
public:
    string compressedString(string word) {
        string t;
        int i0 = -1;
        for (int i = 0; i < word.length(); i++) {
            char c = word[i];
            if (i + 1 == word.length() || c != word[i + 1]) {
                int k = i - i0;
                for (int j = 0; j < k / 9; j++) {
                    t += '9';
                    t += c;
                }
                if (k % 9) {
                    t += '0' + (k % 9);
                    t += c;
                }
                i0 = i;
            }
        }
        return t;
    }
};
```
