# 2564. 子字符串异或查询
链接: [2564. 子字符串异或查询](https://leetcode.cn/problems/substring-xor-queries/)

第 332 场周赛 Q3 `1959`

给你一个 二进制字符串 s 和一个整数数组 queries ，其中 queries[i] = [firsti, secondi] 。

对于第 i 个查询，找到 s 的 最短子字符串 ，它对应的 十进制值 val 与 firsti 按位异或 得到 secondi ，换言之，val ^ firsti == secondi 。

第 i 个查询的答案是子字符串 [lefti, righti] 的两个端点（下标从 0 开始），如果不存在这样的子字符串，则答案为 [-1, -1] 。如果有多个答案，请你选择 lefti 最小的一个。

请你返回一个数组 ans ，其中 ans[i] = [lefti, righti] 是第 i 个查询的答案。

子字符串 是一个字符串中一段连续非空的字符序列。

# 题解
## 预处理

因为int的二进制长度小于31, 显然可以使用 预处理出所有可能的字符串, 然后使用哈希映射: 小技巧: 既然都映射了, 不如直接使用值, 而不是还是使用字符串(写法二(超时))

```C++
class Solution {
public:
    vector<vector<int>> substringXorQueries(
        string s, vector<vector<int>>& queries) {
        // 预处理所有长度小于 30 的子字符串(不要前导0)
        unordered_map<int, pair<int, int>> hash;
        const int n = s.size();
        for (int i = 0; i < n; ++i) {
            int k = 0;     
            for (int r = 0, x = 0; i + r < n && x < 31; ++r, ++x) {
                k = (k << 1) | (s[i + r] & 1);
                if (!hash.count(k))
                    hash[k] = {i, i + r};
                
                if (s[i] == '0')
                    break;
            }
        }

        vector<vector<int>> res(queries.size(), vector<int>(2));
        for (int i = 0; i < res.size(); ++i) {
            int tmp = queries[i][0] ^ queries[i][1];
            auto [L, R] = hash.count(tmp) ? hash[tmp] : make_pair(-1, -1);
            res[i][0] = L;
            res[i][1] = R;
        }
        // v = q[0] ^ q[1]
        return res;
    }
};
```

```C++
class Solution {
public:
    vector<vector<int>> substringXorQueries(
        string s, vector<vector<int>>& queries) {
        // 预处理所有长度小于 30 的子字符串(不要前导0)
        unordered_map<string, pair<int, int>> hash;
        const int n = s.size();
        for (int i = 0; i < n; ++i) {            
            string tmp;
            for (int r = 0, x = 0; i + r < n && x < 31; ++r, ++x) {
                tmp.push_back(s[i + r]);
                if (!hash.count(tmp))
                    hash[tmp] = {i, i + r};
                
                if (s[i] == '0')
                    break;
            }
        }

        vector<vector<int>> res(queries.size(), vector<int>(2));
        for (int i = 0; i < res.size(); ++i) {
            int tmp = queries[i][0] ^ queries[i][1];
            string tmpStr;
            do {
                tmpStr = (char)((tmp & 1) + '0') + tmpStr;
            } while (tmp >>= 1);
            auto [L, R] = hash.count(tmpStr) ? hash[tmpStr] : make_pair(-1, -1);
            res[i][0] = L;
            res[i][1] = R;
        }
        // v = q[0] ^ q[1]
        return res;
    }
};
```

## 方法二: 字典树

我不会这种遍历出子串的字典树写法, 亦或者我可能需要一个模版!

```C++
class Solution {
    struct Trie {
        Trie* arr[2] = { nullptr };
        vector<int> index;
        void insert(string& s, int j) {
            Trie* t = this;
            index.push_back(j);
            for (char& c : s) {
                if (!t->arr[c - '0']) {
                    t->arr[c - '0'] = new Trie();
                }
                t = t->arr[c - '0'];
            }
            t->index.push_back(j);
        }
        void search(string& s, int i, int len, vector<vector<int>> &ans) {
            Trie* t = this;
            int k = 0;
            for (int j = 0; j < len; ++j) {
                char c = s[i+j];
                if (!t->arr[c - '0']) {
                    return;
                }
                t = t->arr[c - '0'];
                for (int x : t->index) {
                    ans[x][0] = i;
                    ans[x][1] = i + j;
                }
                t->index.resize(0);
            }
            return;
        }
    };
public:
    vector<vector<int>> substringXorQueries(string s, vector<vector<int>>& queries) {
        int n = queries.size(), m = s.size();
        vector<vector<int>> ans(n, vector<int>{-1, -1});

        Trie trie;
        for (int i = 0; i < n; ++i) {
            int x = queries[i][0] ^ queries[i][1];
            string t;
            if (x == 0) {
                t = "0";
            } else {
                while (x) {
                    t.push_back('0' + (x & 1));
                    x /= 2;
                }
                reverse(t.begin(), t.end());
            }
            trie.insert(t, i);
        }
        for (int i = 0; i < m; ++i) {
            trie.search(s, i, min(30, m - i), ans);
        }
        return ans;
    }
};

// https://leetcode.cn/problems/substring-xor-queries/solutions/2187149/c-zi-dian-shu-by-yanguilai-w9cn
```
