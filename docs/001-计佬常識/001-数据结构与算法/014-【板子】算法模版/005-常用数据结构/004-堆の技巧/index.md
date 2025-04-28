# 堆の技巧

> [!TIP]
> 技巧, `std::priority_queue`默认的大根堆, 可以通过`存负数`, 使其变为小根堆, 只需要获取的时候还原即可`int x = -pq.top();`. 这样很方便, 对于`tuple<int, int, int>`中对第一个升序、第二个降序、第三个升序这种, 只需要对取数的时候加个负号即可.

## 一、贪心模拟

- [1882. 使用服务器处理任务](https://leetcode.cn/problems/process-tasks-using-servers/)

```cpp
class Solution {
public:
    vector<int> assignTasks(vector<int>& servers, vector<int>& tasks) {
        int n = servers.size(),
            m = tasks.size();
        // j 秒开始 处理任务 j, 结束时间是 j + tasks[j]
        // 权重最小 的 ser 处理它, 如果有多个空闲的, 则选下标最小的
        // 权(小) - 下标(小)
        priority_queue<tuple<int, int>> pq;

        // 结束时间(小), [权(小) - 下标(小)]
        priority_queue<tuple<int, int, int>> timepPq;

        for (int i = 0; i < n; ++i) {
            pq.push({-servers[i], -i});
        }

        vector<int> res(m);
        int nowTime = 0;
        for (int i = 0; i < m; ++nowTime) {
            // 如果为空, 则时间快进到下一个服务器结束
            // || 如果任务结束, 则添加回空闲队列
            if (!pq.size() || timepPq.size() && nowTime >= -get<0>(timepPq.top())) {
                nowTime = get<0>(timepPq.top());
                while (timepPq.size() && get<0>(timepPq.top()) == nowTime) {
                    auto [_, serW, serIdx] = timepPq.top();
                    timepPq.pop();
                    pq.push({serW, serIdx});
                }
                nowTime *= -1;
            }
            // 取服务器
            while (i < m && i <= nowTime && pq.size()) {
                auto [serW, serIdx] = pq.top();
                pq.pop();
                timepPq.push({-(nowTime + tasks[i]), serW, serIdx});
                res[i] = -serIdx;
                ++i;
            }
        }
        return res;
    }
};
```

- [1353. 最多可以参加的会议数目](https://leetcode.cn/problems/maximum-number-of-events-that-can-be-attended/)

贪心, 找最早结束的

```cpp
class Solution {
    inline static constexpr int maxx = 1e5 + 1;
public:
    int maxEvents(vector<vector<int>>& events) {
        vector<vector<int>> time(maxx);
        for (auto& e : events) {
            time[e[0]].push_back(e[1]);
        }

        // 最早结束的
        priority_queue<int> pq; // (小)
        int res = 0;
        for (int i = 0; i < maxx; ++i) {
            // 开始时间相同, 但是我要最早结束的
            for (auto it : time[i])
                pq.push(-it);
            
            // 会议已经结束了
            while (pq.size() && -pq.top() < i)
                pq.pop();
            
            // 第 i 时刻, 只能去一个会议
            if (pq.size()) {
                pq.pop();
                ++res;
            }
        }
        return res;
    }
};
```

## 二、重排元素
### 2.1 贪心+构造 [767. 重构字符串](https://leetcode.cn/problems/reorganize-string/)

给定一个字符串 $s$, 检查是否能重新排布其中的字母，使得两相邻的字符不同。

返回 $s$ 的任意可能的重新排列。若不可行，返回空字符串`""`.

1. 优秀解法: 时间复杂度 $O(n + |\sum|)$

- [贪心+构造，排序/不排序两种写法，附相似题目（Python/Java/C++/Go/JS/Rust）](https://leetcode.cn/problems/reorganize-string/solutions/2779462/tan-xin-gou-zao-pai-xu-bu-pai-xu-liang-c-h9jg)

其核心思想是: 挑选出出现次数最多的元素, 然后 **间隔填入**. 当然其中有一些需要讨论的.

```C++
class Solution {
public:
    string reorganizeString(string s) {
        int n = s.length();
        int count[26]{}, m = 0;
        char mch;
        for (char ch : s) {
            if (++count[ch - 'a'] > m) {
                m = count[ch - 'a'];
                mch = ch;
            }
        }
        if (m > n - m + 1) {
            return "";
        }

        string ans(n, 0);
        int i = 0;
        for (; m--; i += 2) {
            ans[i] = mch; // 先填出现次数最多的字母
        }
        count[mch - 'a'] = 0;

        // 再填其它字母
        for (int j = 0; j < 26; j++) {
            int cnt = count[j];
            while (cnt--) {
                if (i >= n) {
                    i = 1; // 填完偶数填奇数
                }
                ans[i] = 'a' + j;
                i += 2;
            }
        }
        return ans;
    }
};
```

2. 朴素解法: 每一次填写, 使用最多的和第二多的

```C++
class Solution {
public:
    string reorganizeString(string s) {
        // cnt - c
        priority_queue<tuple<int, char>> pq;
        {
            int cnt[26]{};
            for (char c : s)
                ++cnt[c - 'a'];
            for (int i = 0; i < 26; ++i)
                if (cnt[i])
                    pq.push({cnt[i], i + 'a'});
        }
        string res;
        // 使用最多 + 第二多的, 最后看剩下的
        while (pq.size() >= 2) {
            auto [cnt1, c1] = pq.top();
            pq.pop();
            auto [cnt2, c2] = pq.top();
            pq.pop();
            res += c1;
            res += c2;
            if (--cnt1)
                pq.push({cnt1, c1});
            if (--cnt2)
                pq.push({cnt2, c2});
        }
        if (pq.size() && get<0>(pq.top()) > 1)
            return "";
        return res + (pq.size() ? string{} + get<1>(pq.top()) : "");
    }
};
```

### 2.2 不能连续3个相同的字母

- [1405. 最长快乐字符串](https://leetcode.cn/problems/longest-happy-string/)

给你三个整数 $a$, $b$, $c$, 请你返回 **任意一个** 满足下列全部条件的字符串 $s$:

- $s$ 是一个尽可能长的快乐字符串。
- $s$ 中 **最多** 有 $a$ 个字母 'a'、 $b$ 个字母 'b'、 $c$ 个字母 'c' 。
- $s$ 中只含有 'a'、'b'、'c' 三种字母。

如果不存在这样的字符串 $s$, 请返回一个空字符串`""`。

```C++
class Solution {
public:
    string longestDiverseString(int a, int b, int c) {
        priority_queue<tuple<int, char>> pq;
        if (a)
            pq.push({a, 'a'});
        if (b)
            pq.push({b, 'b'});
        if (c)
            pq.push({c, 'c'});

        string res;
        while (pq.size()) {
            auto [c, v] = pq.top();
            pq.pop();
            int n = res.size();
            // 如果最多的不是连续两次出现, 就继续填写
            // 否则就使用第二多的将其分割
            if (n >= 2 && res[n - 1] == v && res[n - 2] == v) {
                if (!pq.size())
                    break;
                auto [c2, v2] = pq.top();
                pq.pop();
                res += v2;
                if (--c2)
                    pq.push({c2, v2});
                pq.push({c, v});
            } else {
                res += v;
                if (--c)
                    pq.push({c, v});
            }
        }
        return res;
    }
};
```

> [!TIP]
> 不能一次填两个, 然后用第二多的分割.
>
> 这样会导致重复, 比如 `b:4, c:5` -> `ccb ccb bbc`
>
> 也不能每次取出两个:
> 
> ```C++
> 比如: a:5, b:2
> 如果每次拿出来两个
> -> aabbaa
> 
> 但最长的情况是
> -> aabaaba
> ```
