# 1143. 最长公共子序列
# LCR 095. 最长公共子序列
原题链接: [LCR 095. 最长公共子序列](https://leetcode.cn/problems/qjnOS7/description/) / [1143. 最长公共子序列](https://leetcode.cn/problems/longest-common-subsequence/description/)

给定两个字符串 text1 和 text2，返回这两个字符串的最长 公共子序列 的长度。如果不存在 公共子序列 ，返回 0 。

一个字符串的 子序列 是指这样一个新的字符串：它是由原字符串在不改变字符的相对顺序的情况下删除某些字符（也可以不删除任何字符）后组成的新字符串。

例如，"ace" 是 "abcde" 的子序列，但 "aec" 不是 "abcde" 的子序列。
两个字符串的 公共子序列 是这两个字符串所共同拥有的子序列。

 

## 示例 1：

输入：text1 = "abcde", text2 = "ace"<br>
输出：3<br>
解释：最长公共子序列是 "ace" ，它的长度为 3 。
## 示例 2：

输入：text1 = "abc", text2 = "abc"<br>
输出：3<br>
解释：最长公共子序列是 "abc" ，它的长度为 3 。
## 示例 3：

输入：text1 = "abc", text2 = "def"<br>
输出：0<br>
解释：两个字符串没有公共子序列，返回 0 。
 

## 提示：

$1 <= text1.length, text2.length <= 1000$<br>
text1 和 text2 仅由小写英文字符组成。

# 我的破烂代码
## V2 尝试记忆化
```C++
class Solution {
public:
    int BFS(int now_i, int now_j /*text2*/, string& text1, string& text2, vector<int>& dp) {
        if (now_i < 0 || now_j < 0)
            return 0;

        if (dp[now_i])
            return dp[now_i];

        int res = 0;
        for (int i = text2.size() - 1; i >= 0; --i) {
            if (text1[now_i] == text2[i]) {
                for (int j = text1.size() - 1; j >= 0; --j)
                    res = max(res, BFS(j - 1, i - 1, text1, text2, dp) + 1);
            }
        }
        dp[now_i] = res;
        return res;
    }

    int longestCommonSubsequence(string text1, string text2) {
        // 选择一个较短的作为处理串
        if (text1.size() > text2.size()) {
            text1.swap(text2);
        }
        int len = text1.size();
        // set<char> tmp(text1.begin(), text1.end());
        // for (auto& it : text2) {
        //     if (tmp.find(it) == tmp.end()) {
        //         it = 0;
        //     }
        // }
        vector<int> dp(len, 0); // dp[i] 定义为包含 text1[i] 的最长公共子序列长度
        int res = 0;
        // text1是处理串, 暴力假设 a是/不是, 然后记录长度
        // for (int i = 0; i < len; ++i) {
        //     int j = 0;
        //     int k = 0;
        //     int now_len = 0;
        //     while (k < text2.size())
        //     {
        //         if (text1[i + j] == text2[k]) {
        //             // 当前为
        //             ++j;
        //             ++now_len;
        //         // } else if (i + j + 1 < len && text1[i + j + 1] == text2[k]) {
        //             // 当前不是
        //             // ++j;
        //             // ++now_len;
        //         } else {
        //         }
        //         ++k;
        //     }

        //     res = max(res, now_len);
        // }
        // printf("%s %s\n", text1.c_str(), text2.c_str());
        for (int i = len - 1; i >= 0; --i) {
            res = max(res, BFS(i, text2.size(), text1, text2, dp));
        }
        // for (auto& it : dp) {
        //     res = max(res, it);
        // }
        return res;
    }
};
```

## 暴力但BFS

写的时候有点感觉: 不就是 选 与 不选 吗?

但是, 确实不知道它要二维状态.

所以我都是一维状态(以text1[0,1,2,..i]的[i]结尾的最长子序列长度 为dp状态, 这样导致每次都要遍历text2 ...)

```C++
class Solution {
public:
    int BFS(int now_i, int now_j /*text2*/, string& text1, string& text2, vector<int>& dp) {
        if (now_i >= text1.size())
            return 0;

        if (dp[now_i])
            return dp[now_i];

        int res = 0;
        for (int i = now_j; i < text2.size(); ++i) {
            if (text1[now_i] == text2[i]) {
                for (int j = now_i; j < text1.size(); ++j)
                    res = max(res, BFS(j + 1, i + 1, text1, text2, dp) + 1);
            }
        }

        dp[now_i] = res;
        return res;
    }

    int longestCommonSubsequence(string text1, string text2) {
        // 选择一个较短的作为处理串
        if (text1.size() > text2.size()) {
            text1.swap(text2);
        }
        int len = text1.size();
        vector<int> dp(len, 0); // dp[i] 定义为包含 text1[i] 的最长公共子序列长度
        int res = 0;
        for (int i = 0; i < len; ++i) {
            res = max(res, BFS(i, 0, text1, text2, dp));
        }
        return res;
    }
};
```

# 题解
[教你一步步思考动态规划！（Python/Java/C++/Go）](https://leetcode.cn/problems/qjnOS7/solutions/2139882/jiao-ni-yi-bu-bu-si-kao-dong-tai-gui-hua-f5k9)

```bilibili ##BV1TM4y1o7ug##

```

## 算法术语科普

1. 子数组/子串 --> 是连续的
2. 子序列 --> 不一定是连续的. (只需要保证位置是原来的, 之间的可以删除或不删除

---

## 递归: 记忆化搜索

简直啦! 一语惊醒梦中人!

<span style="color:red">**和背包问题一样, 子序列你只需要考虑每个字母, 你是选还是不选**: <br><br>从最后一个字母开始考虑, 练练组合, 就有4种情况: 选选, 选不, 不选, 不不; 那么问题就从 m 个字母 + n 个字母 的`原问题`, 变成了 m - 1 个字母 + n - 1个字母的`子问题`</span>

注意以下两种特殊情况:

1. 在`text1[i]` == `text2[j]` 时, 需要 `dfs(i - 1, j)` 和 `dfs(i, j - 1)` 吗?

    - 不需要.

2. 在`text1[i]` != `text2[j]` 时, 需要 **$dfs(i - 1, j - 1)$** 吗?

    - 不需要, 因为在 `dfs(i - 1, j)` 或 `dfs(i, j - 1)` 时, 已经包含了 dfs(i - 1, j - 1)过来的情况.

    - 所以恒有:  $dfs(i, j)\ge\left\{\begin{matrix}  dfs(i - 1, j)\\  dfs(i, j - 1)\\\end{matrix}\left.\begin{matrix} \\ \\\end{matrix}\right\}\right. \ge dfs(i - 1, j - 1)$

即

$$\begin{array}{c}dfs(i, j) = \left\{\begin{matrix}  & dfs(i - 1, j - 1) + 1 & text1[i] = text2[j]\\  & max(dfs(i -1 , j), dfs(i, j - 1)) & text1[i] \ne  text2[j]\end{matrix}\right.\end{array}$$


```C++
class Solution {
public:
    int BFS(int i, int j, string& text1, string& text2, vector<vector<int>>& dp) {
        if (i < 0 || j < 0)
            return 0;
        
        if (dp[i][j])
            return dp[i][j];
        
        int res = 0;
        if (text1[i] == text2[j])
            res = BFS(i - 1, j - 1, text1, text2, dp) + 1; // 这里可以直接 goto 到 return 了
        
        // 请忽略后面max的res (之前还不知道要省略)
        res = max(BFS(i, j - 1, text1, text2, dp), max(BFS(i - 1, j, text1, text2, dp), res));

        dp[i][j] = res;
        return dp[i][j];
    }

    int longestCommonSubsequence(string text1, string text2) {
        // dp[i][j] 即以text1[0,1,2,...,i]的[i]结尾 text2[0,1,2,...,j]的[j]结尾的最长子序列长度
        // 类似于str的01背包, 有4种情况: 选选, 选不, 不选, 不不
        // 当且仅当 [i] == [j] 的时候可以 选选
        int m = text1.size();
        int n = text2.size();
        // + 1 防止越界
        vector<vector<int>> dp(m + 1, vector<int>(n + 1, 0));

        return BFS(m, n, text1, text2, dp) - 1;
    }
};
```

## 转化为**递推**dp

递归-->递推: 即 将原来记忆化搜索(递归)的边界条件作为初始值!

即

$$\begin{array}{c}dp(i, j) = \left\{\begin{matrix}  & dp(i - 1, j - 1) + 1 & text1[i] = text2[j]\\  & max(dp(i -1 , j), dp(i, j - 1)) & text1[i] \ne  text2[j]\end{matrix}\right.\end{array}$$

然后是 - 1 会数组越界, 所以整体 + 1

$$\begin{array}{c}dp(i + 1, j + 1) = \left\{\begin{matrix}  & dp(i , j) + 1 & text1[i] = text2[j]\\  & max(dp(i , j + 1), dp(i + 1, j)) & text1[i] \ne  text2[j]\end{matrix}\right.\end{array}$$

变dp数组即可, 判断条件的不需要变.

```C++
class Solution {
public:
    int longestCommonSubsequence(string text1, string text2) {
        int m = text1.size();
        int n = text2.size();
        vector<vector<int>> dp(m + 1, vector<int>(n + 1, 0));

        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                if (text1[i] == text2[j])
                    dp[i + 1][j + 1] = dp[i][j] + 1;
                else
                    dp[i + 1][j + 1] = max(dp[i + 1][j], dp[i][j + 1]);
            }
        }

        return dp[m][n];
    }
};
```

## 状态压缩?

我不会!