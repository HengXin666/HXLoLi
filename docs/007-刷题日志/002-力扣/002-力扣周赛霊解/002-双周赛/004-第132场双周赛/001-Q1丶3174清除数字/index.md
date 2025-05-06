3174\. 清除数字
-----------

给你一个字符串 s 。

你的任务是重复以下操作删除 **所有** 数字字符：

*   删除 **第一个数字字符** 以及它左边 **最近** 的 **非数字** 字符。

请你返回删除所有数字字符以后剩下的字符串。

**示例 1：**

**输入：** s = "abc"

**输出：**"abc"

**解释：**

字符串中没有数字。

**示例 2：**

**输入：** s = "cb34"

**输出：**""

**解释：**

一开始，我们对 s\[2\] 执行操作，s 变为 "c4" 。

然后对 s\[1\] 执行操作，s 变为 "" 。

**提示：**

*   1 <= s.length <= 100
*   s 只包含小写英文字母和数字字符。
*   输入保证所有数字都可以按以上操作被删除。

[https://leetcode.cn/problems/clear-digits/description/](https://leetcode.cn/problems/clear-digits/description/)

# 题解
## HX

```C++
class Solution {
public:
    string clearDigits(string s) {
        string res = "";
        for (int i = 0; i < s.size(); ++i) {
            if (s[i] >= '0' && s[i] <= '9') {
                if (res.size())
                    res.pop_back();
            }
            else
                res += s[i];
        }
        return res;
    }
};
```

## 0x3f: 用栈维护
```C++
class Solution {
public:
    string clearDigits(string s) {
        string st;
        for (char c : s) {
            if (isdigit(c)) {
                st.pop_back();
            } else {
                st += c;
            }
        }
        return st;
    }
};
```
