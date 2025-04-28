# 3160. 所有球里面不同颜色的数目
链接: [3160. 所有球里面不同颜色的数目](https://leetcode.cn/problems/find-the-number-of-distinct-colors-among-the-balls/)

给你一个整数 limit 和一个大小为 n x 2 的二维数组 queries 。

总共有 limit + 1 个球，每个球的编号为 [0, limit] 中一个 互不相同 的数字。一开始，所有球都没有颜色。queries 中每次操作的格式为 [x, y] ，你需要将球 x 染上颜色 y 。每次操作之后，你需要求出所有球中 不同 颜色的数目。

请你返回一个长度为 n 的数组 result ，其中 result[i] 是第 i 次操作以后不同颜色的数目。

注意 ，没有染色的球不算作一种颜色。

提示:
- 1 <= limit <= 10^9
- 1 <= n == queries.length <= 10^5
- queries[i].length == 2
- 0 <= queries[i][0] <= limit
- 1 <= queries[i][1] <= 10^9

# 题解
## 双哈希模拟

wa了2次, 总是忘记一些东西...

```C++
class Solution {
public:
    vector<int> queryResults(int lm, vector<vector<int>>& qu) {
        vector<int> res(qu.size());
        unordered_map<int, int> col; // 球 - 颜色
        unordered_map<int, int> cnt; // 颜色 - 数量
        int now = 0; // 当前颜色的数量
        for (int i = 0; i < res.size(); ++i) {
            // [0] -> [1]色
            if (!col[qu[i][0]]) { // 每涂色, 不存在
                col[qu[i][0]] = qu[i][1]; // 给色
                if (!(cnt[qu[i][1]]++)) // 新颜色
                    ++now;
            } else {
                // 减色数量
                if (!(--cnt[col[qu[i][0]]]))
                    --now;
                if (!(cnt[qu[i][1]]++))
                    ++now;
                col[qu[i][0]] = qu[i][1]; // 给色
            }
            res[i] = now;
        }
        
        return res;
    }
};
```

下面是简洁写法:

## 0x3f: 哈希模拟

```C++
class Solution {
public:
    vector<int> queryResults(int, vector<vector<int>>& queries) {
        vector<int> ans;
        unordered_map<int, int> color, cnt;
        for (auto& q : queries) {
            int x = q[0], y = q[1];
            if (auto it = color.find(x); it != color.end()) { // 修改已存在球的颜色
                int c = it->second;
                if (--cnt[c] == 0) {
                    cnt.erase(c);
                }
            }
            color[x] = y;
            cnt[y]++;
            ans.push_back(cnt.size());
        }
        return ans;
    }
};
```
