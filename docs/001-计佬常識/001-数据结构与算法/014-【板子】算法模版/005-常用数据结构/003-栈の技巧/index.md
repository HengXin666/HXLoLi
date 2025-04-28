# 栈
没见过的东西, 你又怎么会呢? 当场发明? 你怎么不发顶刊呢?

## 1. 最小栈

- [155. 最小栈](https://leetcode.cn/problems/min-stack/) [Hot 100]

$O(1)$ 时间, 设计一个支持 push ，pop ，top 操作，并能在 **常数时间** 内检索到 **最小元素** 的栈

```C++
class MinStack {
    // {val, 前缀最小值}
    stack<tuple<int, int>> st;
public:
    MinStack() 
        : st() 
    {
        st.push({0, INT_MAX}); // 哨兵
    }
    
    void push(int val) {
        st.push({val, min(val, get<1>(st.top()))});
    }
    
    void pop() {
        st.pop();
    }
    
    int top() {
        return get<0>(st.top());
    }
    
    int getMin() {
        return get<1>(st.top());
    }
};
```

## 2. 字典序最小栈

- [2434. 使用机器人打印字典序最小的字符串](https://leetcode.cn/problems/using-a-robot-to-print-the-lexicographically-smallest-string/)

给你一个字符串 $s$ 和一个机器人，机器人当前有一个空字符串 $t$。执行以下操作之一，直到 $s$ 和 $t$ 都变成空字符串:

- 删除字符串 $s$ 的 **第一个** 字符，并将该字符给机器人。机器人把这个字符添加到 $t$ 的尾部。

- 删除字符串 $t$ 的 **最后一个** 字符，并将该字符给机器人。机器人将该字符写到纸上。

请你返回纸上能写出的 **字典序最小** 的字符串。

> [!TIP]
> 贪心地思考，为了让字典序最小，在遍历 $s$ 的过程中，如果栈顶字符 $≤$ 后续字符（未入栈）的最小值，那么应该出栈并加到答案末尾，否则应当继续遍历，取到比栈顶字符小的那个字符，这样才能保证字典序最小

```C++
class Solution {
public:
    string robotWithString(string s) {
        int cnt[26]{0}, minn = 0;
        for (char c : s)
            ++cnt[c - 'a'];
        stack<char> st;
        string res;
        for (char c : s) {
            --cnt[c - 'a'];
            while (minn < 25 && !cnt[minn])
                ++minn;
            st.push(c);
            while (st.size() && st.top() - 'a' <= minn) {
                res += st.top();
                st.pop();
            }
        }
        return res;
    }
};
```

## 3. 分层栈

- [895. 最大频率栈](https://leetcode.cn/problems/maximum-frequency-stack/)

设计一个类似堆栈的数据结构，将元素推入堆栈，并从堆栈中弹出 **出现频率** 最高的元素。

实现 FreqStack 类:

- `FreqStack()`构造一个空的堆栈。

- `void push(int val)`将一个整数 val 压入栈顶。

- `int pop()`删除并返回堆栈中出现频率最高的元素。

如果出现频率最高的元素不只一个，则移除并返回最接近栈顶的元素。

```C++
class FreqStack {
    unordered_map<int, stack<int>> st;
    unordered_map<int, int> cnt;
    int maxx = 0;
public:
    FreqStack()
        : st()
        , cnt()
    {}
    
    void push(int val) {
        int x = ++cnt[val];
        maxx = max(maxx, x);
        st[x].push(val);
    }
    
    int pop() {
    BEGIN:
        auto& v = st[maxx];
        if (!v.size()) {
            --maxx;
            goto BEGIN;
        }
        int res = v.top();
        --cnt[res];
        v.pop();
        return res;
    }
};
```

> [!TIP]
> 此处可以换`unordered_map<int, stack<int>>`为`vector<stack<int>>`, 因为它是连续的, 中间不会断层.

## 4. 邻项消除
> [2751. 机器人碰撞](https://leetcode.cn/problems/robot-collisions/)


```C++
class Solution {
public:
    vector<int> survivedRobotsHealths(
        vector<int>& positions, 
        vector<int>& healths, 
        string directions // LR
    ) {
        int n = positions.size();
        vector<int> st;
        vector<int> id(n);
        iota(id.begin(), id.end(), 0);
        sort(id.begin(), id.end(), [&](int i, int j) {
            return positions[i] < positions[j];
        });
        for (int i : id) {
            auto& h = healths[i];
            auto d = directions[i];
        BEGIN:
            if (st.size() && directions[st.back()] == 'R' && d == 'L') {
                if (healths[st.back()] < h) {
                    healths[st.back()] = 0;
                    st.pop_back();
                    --h;
                    goto BEGIN; // 连续消除
                } else if (healths[st.back()] > h) {
                    --healths[st.back()];
                    h = 0;
                } else {
                    healths[st.back()] = 0;
                    h = 0;
                    st.pop_back();
                }
            } else {
                st.push_back(i);
            }
        }
        healths.erase(remove(healths.begin(), healths.end(), 0), healths.end());
        return healths;
    }
};
```

> [!TIP]
> `.erase(remove(` 是[擦除移除手法](https://en.wikipedia.org/wiki/Erase%E2%80%93remove_idiom), 即先使用`双指针`把元素移动到尾部, 然后返回移动后的非0值的位置, 然后就使用`erase`删除 [removeResIt, end()] 的内容.
>
> 其中`remove`可能的实现为:
> ```C++
> template <class ForwardIt, class T = typename std::iterator_traits<ForwardIt>::value_type>
> ForwardIt remove(ForwardIt first, ForwardIt last, const T& value) {
>     first = std::find(first, last, value);
>     if (first != last)
>         for (ForwardIt i = first; ++i != last;)
>             if (!(*i == value))
>                 *first++ = std::move(*i);
>     return first;
> }
> ```

## 5. 合法括号字符串
> 大部分题目可以不用栈, 而是用一个数字记录嵌套深度

### 5.1 匹配合法的范围

- [678. 有效的括号字符串](https://leetcode.cn/problems/valid-parenthesis-string/)

> 记录当前未匹配左括号数量的范围即可。只需遍历一遍，不需要使用栈。
>
>这道题是普通括号匹配的变体。回忆一下一般的括号匹配（即没有'*'这个特殊符号），一般用栈来做，栈用来存储左括号，每当遇到右括号时出栈。实际上，这个栈不是必须的，只需要用一个变量记录当前未匹配左括号的数量即可。
>
>从这个角度来看，这道题就会简单很多，加入'*'号后，未匹配左括号的数量从一个值变成了一个范围，所以只需要转变思路，用两个变量来记录这个范围的上下界即可。

```C++
class Solution {
public:
    bool checkValidString(string s) {
        // 未匹配的 '(' 的范围 [l, r]
        int l = 0, // 下限 
            r = 0; // 上限
        for (char c : s) {
            if (c == '(') {
                ++l, ++r;
            } else if (c == ')') {
                l = max(0, l - 1);
                if (--r < 0)
                    return false;   // 上限 < 0 说明: ")"比"(*"多
            } else { // *
                l = max(0, l - 1);  // 算 * 为 ) 或者 ""
                ++r;                // 算 * 为 (
            }
        }
        return l <= 0; // 下限为0, 说明可以匹配完美
    }
};
```

- [2116. 判断一个括号字符串是否有效](https://leetcode.cn/problems/check-if-a-parentheses-string-can-be-valid/)

本题的区别是, `*`不能表示为一个`空字符串`.

- 括号匹配通法: [一次遍历（Python/Java/C++/C/Go/JS/Rust）](https://leetcode.cn/problems/check-if-a-parentheses-string-can-be-valid/solutions/1178043/zheng-fan-liang-ci-bian-li-by-endlessche-z8ac), 使用一个变量 $c$ 维护未匹配的左括号的数量. `c += 2 * (v == '(') - 1;`, 具体请看链接...

```C++
class Solution {
public:
    bool canBeValid(string s, string locked) {
        int l = 0, r = 0, n = s.size();
        if (n & 1)
            return false;
        for (int i = 0; i < n; ++i) {
            if (locked[i] == '0') {
                l = max(++r & 1, l - 1);
            } else if (s[i] == '(') {
                ++l, ++r;
            } else { // s[i] == ')'
                if (--r < 0)
                    return false;
                l = max(0, l - 1);
            }
        }
        return l == 0;
    }
};
```


### 5.2 匹配最长合法的方案

- [32. 最长有效括号](https://leetcode.cn/problems/longest-valid-parentheses/)

使用栈记录索引, 当出栈的时候, 结算长度. 特别的: 添加一个哨兵: `-1`以及出栈后为空时候`i`.

```C++
class Solution {
public:
    int longestValidParentheses(string s) {
        int res = 0;
        stack<int> st;
        st.push(-1);
        for (int i = 0; i < s.size(); ++i) {
            if (s[i] == ')') {
                st.pop();
                if (st.size()) {
                    res = max(res, i - st.top());
                } else {
                    st.push(i);
                }
            } else {
                st.push(i);
            }
        }
        return res;
    }
};
```
