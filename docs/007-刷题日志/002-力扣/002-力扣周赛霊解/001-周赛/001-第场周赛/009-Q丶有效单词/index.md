# 3136. 有效单词
链接: [3136. 有效单词](https://leetcode.cn/problems/valid-word/)

有效单词 需要满足以下几个条件：

- 至少 包含 3 个字符。
- 由数字 0-9 和英文大小写字母组成。（不必包含所有这类字符。）
- 至少 包含一个 元音字母 。
- 至少 包含一个 辅音字母 。

给你一个字符串 word 。如果 word 是一个有效单词，则返回 true ，否则返回 false 。

注意：

- 'a'、'e'、'i'、'o'、'u' 及其大写形式都属于 元音字母 。
- 英文中的 辅音字母 是指那些除元音字母之外的字母。

# 题解
## 关于我比赛抽风wa了2次の件

我是傻逼! 早上起来人都嘛了! 不应该晚睡的qwq

```C++
class Solution {
public:
    bool isValid(string s) {
        if (s.size() < 3)
            return 0;
        
        int y = 0, f = 0;
        
        for (char c : s) {
            if (c >= '0' && c <= '9') // 第2次 wa: 忘记写等于号
                continue;
            
            if (c >= 'A' && c <= 'Z')
                c = c - 'A' + 'a';
            
            if (c == 'a' || c =='e' || c == 'i' || c == 'o' || c == 'u') // 第1次 wa: 把o写成了0
                ++y;
            else if (c >= 'a' && c <= 'z')
                ++f;
            else
                return 0;
        }
        return y && f;
    }
};
```

## 那个男人!

```C++
class Solution {
public:
    bool isValid(string word) {
        if (word.length() < 3) {
            return false;
        }
        bool f[2]{};
        for (char c : word) {
            if (isalpha(c)) { // 是字母
                c = tolower(c);
                f[c == 'a' || c == 'e' || c == 'i' || c == 'o' || c == 'u'] = true;
            } else if (!isdigit(c)) { // 不是数字 (那就是特殊符号! 再见!)
                return false;
            }
        }
        return f[0] && f[1];
    }
};
// By 0x3f
```