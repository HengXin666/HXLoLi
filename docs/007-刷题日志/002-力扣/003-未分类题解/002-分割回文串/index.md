# 131. 分割回文串
中等

原题: [131. 分割回文串](https://leetcode.cn/problems/palindrome-partitioning/description/)

给你一个字符串 s，请你将 s 分割成一些子串，使每个子串都是 
回文串。返回 s 所有可能的分割方案。

 
```
示例 1：

输入：s = "aab"
输出：[["a","a","b"],["aa","b"]]

示例 2：

输入：s = "a"
输出：[["a"]]
```

提示：

$
1 <= s.length <= 16\\
s 仅由小写英文字母组成\\
$

# 代码
## 1.0

```C++
class Solution {
    set<vector<string>> res;
    bool ifhwStr(const string& s) {
        int i = 0;
        int j = s.size() - 1;
        while (i < j) {
            if (s[i++] != s[j--])
                return 0;
        }
        return 1;
    }

    void dfs(vector<string>& tmp) {
        this->res.insert(tmp);
        if (tmp.size() == 1) {
            return;
        }
        // 假设原来有逗号, 现在是连接或者不连接
        vector<string> cache;
        for (int i = 1; i < tmp.size(); ++i) {
            // 如果连接 i 和 i - 1, 看他们是不是回文, 如果是则连接
            string str = tmp[i - 1] + tmp[i];
            if (ifhwStr(str)) {
                // 删除逗号
                cache.push_back(str);
                vector<string> ccc = cache;
                for (int j = i + 1; j < tmp.size(); ++j)
                    ccc.push_back(tmp[j]);
                dfs(ccc);
                cache.pop_back();
            }
            cache.push_back(tmp[i - 1]);
        }
    }

public:
    vector<vector<string>> partition(string s) {
        // 分或不分
        this->res = set<vector<string>>();
        vector<string> tmp;
        for (char& it : s) {
            string sss;
            sss.push_back(it);
            tmp.push_back(move(sss));
        }
        dfs(tmp);
        return vector<vector<string>>(this->res.begin(), this->res.end());
    }
};
```

## AC代码

为什么03xf大佬说这个是`答案的视角（枚举子串结束位置）`啊? 我认为是逗号选或者不选这样理解のに...

- 值得注意的是:<span style="color:red">`string&& std::string::substr(开始索引, 长度)`</span>

```C++
class Solution {
    bool ifhwStr(const string& s, int l, int r) {
        while (l < r) {
            if (s[l++] != s[r--])
                return 0;
        }
        return 1;
    }

public:
    vector<vector<string>> partition(string s) {
        /*
选或者不选这个逗号
aabb -> (0) -> [0, 0](回文) + dfs(0 + 1) -> [1, 1](回文) + dfs(1 + 1) -> [2, 2](回文) + dfs(2 + 1) -> [3, 3](回文) + dfs(3 + 1) -> 4 >= len 结束 {"a", "a", "b", "b"}
                                        -> [1, 2](回文) x             ->[2, 3](回文) + dfs(3 + 1) -> 4 >= len 结束 {"a", "a", "bb"}
            -> [0, 1](回文) + dfs(1 + 1) -> [2, 2](回文) + dfs(2 + 1) -> ...
            -> [0, 2](回文) x
            -> [0, 3](回文) x
for (int x = start; x < n; ++x) {
    // 选
    if ( (start, x) 是回文 ) {
        // 加入
        dfs(x + 1);
        // 回溯
    }
}
        */
        vector<vector<string>> res;
        vector<string> arr;
        function<void(int)> dfs = [&](int start) {
            if (start >= s.size()) {
                res.push_back(arr);
                return;
            }

            for (int x = start; x < s.size(); ++x) {
                if (ifhwStr(s, start, x)) {
                    // 我艹, substr的第二个参数是 长度 而不是索引!
                    arr.push_back(s.substr(start, x - start + 1));
                    dfs(x + 1);
                    arr.pop_back();
                }
            }
        };

        dfs(0);
        return res;
    }
};
```
