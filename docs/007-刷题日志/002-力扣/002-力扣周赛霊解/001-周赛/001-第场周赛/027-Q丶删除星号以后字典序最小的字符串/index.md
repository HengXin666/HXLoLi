# 3170. 删除星号以后字典序最小的字符串
链接: [3170. 删除星号以后字典序最小的字符串](https://leetcode.cn/problems/lexicographically-minimum-string-after-removing-stars/)

给你一个字符串 s 。它可能包含任意数量的 '\*' 字符。你的任务是删除所有的 '*' 字符。

当字符串还存在至少一个 '*' 字符时，你可以执行以下操作：

- 删除最左边的 '*' 字符，同时删除该星号字符左边一个字典序 最小 的字符。如果有多个字典序最小的字符，你可以删除它们中的任意一个。

请你返回删除所有 '*' 字符以后，剩余字符连接而成的 字典序最小 的字符串。

# 题解
## 贪心

```C++
class Solution {
public:
    string clearStars(string s) {
        // 贪心, 如果有 *, 并且有多个 相同字典序的, 删除最近的
        vector<int> pq[26]; // 记录每个字母的最近的索引
        for (int i = 0; i < s.size(); ++i) {
            if (s[i] == '*') {
                for (int j = 0; j < 26; ++j) {
                    if (pq[j].size()) {
                        pq[j].pop_back();
                        break;
                    }
                }
            } else {
                pq[s[i] - 'a'].push_back(i);
            }
        }

        // 输出答案
        priority_queue<tuple<int, int>, std::vector<tuple<int, int>>, std::greater<tuple<int, int>>> qq;
        for (int i = 0; i < 26; ++i) {
            for (int it : pq[i]) {
                qq.push({it, i});
            }
        }
        
        string res;
        
        while (qq.size()) {
            auto [_, c] = qq.top();
            qq.pop();
            res += (char)(c + 'a');
        }
        
        return res;
    }
};
```

实际上, 使用一个`bool`数组记录是否删除即可, 或者使用`*`替换掉之前的, 然后构建res就不理会`*`

## 0x3f 位运算优化

用一个二进制数 mask 记录哪些字母对应的列表是非空的，那么 mask 尾零的个数即为最小的字母。


```C++
class Solution {
public:
    string clearStars(string s) {
        int n = s.length(), mask = 0;
        stack<int> st[26];
        for (int i = 0; i < n; i++) {
            if (s[i] != '*') {
                st[s[i] - 'a'].push(i);
                mask |= 1 << (s[i] - 'a');
            } else {
                int k = __builtin_ctz(mask);
                auto& p = st[k];
                s[p.top()] = '*';
                p.pop();
                if (p.empty()) {
                    mask ^= 1 << k;
                }
            }
        }
        s.erase(remove(s.begin(), s.end(), '*'), s.end());
        return s;
    }
};
```
