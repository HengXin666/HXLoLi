# 3106. 满足距离约束且字典序最小的字符串
链接: [3106. 满足距离约束且字典序最小的字符串](https://leetcode.cn/problems/lexicographically-smallest-string-after-operations-with-constraint/description/)

中等 第392场周赛Q2

--- 
给你一个字符串`s`和一个整数`k`。

定义函数`distance(s1, s2)`，用于衡量两个长度为`n`的字符串`s1`和`s2`之间的距离，即：

字符`'a'`到`'z'`按 循环 顺序排列，对于区间`[0, n - 1]`中的`i`，计算所有「 $s_1[i]$ 和 $s_2[i]$ 之间 最小距离」的 和 。
例如，`distance("ab", "cd") == 4`，且`distance("a", "z") == 1`。

你可以对字符串`s`执行 任意次 操作。在每次操作中，可以将`s`中的一个字母 改变 为 **任意** 其他小写英文字母。

返回一个字符串，表示在执行一些操作后你可以得到的 **字典序最小** 的字符串`t`，且满足`distance(s, t) <= k`。

## 示例 1：

```
输入：s = "zbbz", k = 3
输出："aaaz"
解释：在这个例子中，可以执行以下操作：
将 s[0] 改为 'a' ，s 变为 "abbz" 。
将 s[1] 改为 'a' ，s 变为 "aabz" 。
将 s[2] 改为 'a' ，s 变为 "aaaz" 。
"zbbz" 和 "aaaz" 之间的距离等于 k = 3 。
可以证明 "aaaz" 是在任意次操作后能够得到的字典序最小的字符串。
因此，答案是 "aaaz" 。
```

## 示例 2：
```
输入：s = "xaxcd", k = 4
输出："aawcd"
解释：在这个例子中，可以执行以下操作：
将 s[0] 改为 'a' ，s 变为 "aaxcd" 。
将 s[2] 改为 'w' ，s 变为 "aawcd" 。
"xaxcd" 和 "aawcd" 之间的距离等于 k = 4 。
可以证明 "aawcd" 是在任意次操作后能够得到的字典序最小的字符串。
因此，答案是 "aawcd" 。
```
## 示例 3：
```
输入：s = "lol", k = 0
输出："lol"
解释：在这个例子中，k = 0，更改任何字符都会使得距离大于 0 。
因此，答案是 "lol" 。
```

## 提示：
$
1 <= s.length <= 100\\
0 <= k <= 2000\\
s 只包含小写英文字母。
$

# 题解
## 我的AC

```C++
class Solution {
    // 计算 字符 a 与 b 的 distance
    int op(char a, char b) { // a -> res[i], b -> s[i]
        int res = 0;
        a -= 'a';
        b -= 'a';
        return min(abs(a - b), // 一时间不知道怎么优化
                   min(
                    a + abs(26 - b), // 实际上等价于 26 - abs(a - b)
                    b + abs(26 - a)
                   )); 
    }
public:
    string getSmallestString(string s, int k) {
        // 贪心的, 改 从前开始的a即可
        string res = s;
        int now_k = 0;
        for (int i = 0; i < s.size(); ++i) {
            // 尝试将当前res[i]改为`a`
            for (int j = 0; j < 26; ++j) {
                int tmp = op(s[i], 'a' + j);
                if (tmp + now_k <= k) {
                    res[i] = 'a' + j;
                    now_k += tmp;
                    break;
                }
            }
            
            if (now_k == k)
                break;
        }
        
        return res;
    }
};
```

## 03xf更优化

```C++
class Solution {
public:
    string getSmallestString(string s, int k) {
        for (int i = 0; i < s.length(); i++) {
            // a --- ? --- 'z'
            // |<- ->|
            // s[i]-'a'
            //       |<-  ->|
            //       'z' - s[i]
            // +1 是 'z' 与 'a' 之间的距离
            int dis = min(s[i] - 'a', 'z' - s[i] + 1);
            if (dis > k) {
                s[i] -= k;
                break;
            }
            s[i] = 'a';
            k -= dis;
        }
        return s;
    }
};

// 作者：灵茶山艾府
// 链接：https://leetcode.cn/problems/lexicographically-smallest-string-after-operations-with-constraint/solutions/2727203/tan-xin-pythonjavacgo-by-endlesscheng-vzgo/
// 来源：力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```