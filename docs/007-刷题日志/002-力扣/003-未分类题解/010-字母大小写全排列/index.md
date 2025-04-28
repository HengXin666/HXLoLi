# 784. 字母大小写全排列
原题: [784. 字母大小写全排列](https://leetcode.cn/problems/letter-case-permutation/)

第 72 场周赛 Q1 中等 1342

给定一个字符串 s ，通过将字符串 s 中的每个字母转变大小写，我们可以获得一个新的字符串。

返回 所有可能得到的字符串集合 。以 任意顺序 返回输出。

```
示例 1：

输入：s = "a1b2"
输出：["a1b2", "a1B2", "A1b2", "A1B2"]
示例 2:

输入: s = "3z4"
输出: ["3z4","3Z4"]
```

提示:

$
1 <= s.length <= 12\\
s 由小写英文字母、大写英文字母和数字组成
$

# 题解
## 学到了一种快速大小写互换的写法!

字母大小写转换规则，例如 `char c` ：

- 互相转(大转小，小转大)， 异或32， `c ^ 32`

- 大转小，小不变，或 32, `c | 32`

- 小转大，大不变，与非32, `c & ~32`

即可以写成 `异或一个空格`:

```C++
charArray[index] ^= ' ';
```

```C++
class Solution {
public:
    vector<string> letterCasePermutation(string s) {
        vector<string> res;
        string str;
        // 选择大或者选择小
        function<void(int)> dfs = [&](int i) {
            if (i == s.size()) {
                res.push_back(str);
                return;
            }

            // 如果是字母
            if (!(s[i] >= '0' && s[i] <= '9')) {
                // 大写
                str.push_back(s[i] ^ ' '); // 快速大小写互换
                dfs(i + 1);
                str.pop_back(); // 回溯
                // 小写
            }
            str.push_back(s[i]);
            dfs(i + 1);
            str.pop_back(); // 回溯 (如果这里不回溯, 会影响后面的, str 是引用!)
        };
        dfs(0);

        return res;
    }
};
```
