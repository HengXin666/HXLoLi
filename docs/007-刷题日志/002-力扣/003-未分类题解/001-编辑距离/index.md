# 72. 编辑距离
原题链接: [72. 编辑距离](https://leetcode.cn/problems/edit-distance/description/)

给你两个单词 `word1` 和 `word2`， 请返回将 `word1` 转换成 `word2` 所使用的最少操作数。

你可以对一个单词进行如下三种操作：

- 插入一个字符
- 删除一个字符
- 替换一个字符
 
## 示例 1：

输入：word1 = "horse", word2 = "ros"<br>
输出：3<br>
解释：<br>
horse -> rorse (将 'h' 替换为 'r')<br>
rorse -> rose (删除 'r')<br>
rose -> ros (删除 'e')<br>
## 示例 2：

输入：word1 = "intention", word2 = "execution"<br>
输出：5<br>
解释：<br>
intention -> inention (删除 't')<br>
inention -> enention (将 'i' 替换为 'e')<br>
enention -> exention (将 'n' 替换为 'x')<br>
exention -> exection (将 'n' 替换为 'c')<br>
exection -> execution (插入 'u')<br>
 

## 提示：

0 <= word1.length, word2.length <= 500<br>
word1 和 word2 由小写英文字母组成<br>

# 题解
我们非常容易想到:

有这几种转移方式:
1. 不操作
2. 插入
3. 删除
4. 替换

而且容易想通: 

- 当 `word1[i] == word2[j]` 时, 就是 `不操作`.

那么 插入/删除/替换 怎么搞?

另外, 我们虽然定义了转移过程, 但是还没有定义状态! 请问状态怎么来的?

**只因题做多了, 自然就有那个感觉了!...**

## 思考方法1
解决两个字符串的动态规划问题，一般都是用两个指针 i, j 分别指向两个字符串的最后，然后一步步往前走，缩小问题的规模。

## 思考方法2
手画一个二维矩阵, 然后推一次, 找规律.

---
不过这些, 都是建立在已经确定了状态的前提下!

dp[i][j] 即 word1[0...i] 和 word2[0...j] 的最小编辑距离。


## BFS + memo

```C++
class Solution {
public:
    int BFS(int i, int j, string& word1, string& word2, vector<vector<int>>& dp) {
        if (i < 0)
            return j + 1; // 下标为j的字符串的长度是 j + 1
        if (j < 0)
            return i + 1;
        if (dp[i][j])
            return dp[i][j];
        
        if (word1[i] == word2[j])
            dp[i][j] = BFS(i - 1, j - 1, word1, word2, dp);
        else
            dp[i][j] = min( BFS(i, j - 1, word1, word2, dp), // 插入
                min(
                    BFS(i - 1, j - 1, word1, word2, dp),     // 替换
                    BFS(i - 1, j, word1, word2, dp)          // 删除
                )
            ) + 1;
        return dp[i][j];
    }

    int minDistance(string word1, string word2) {
        int m = word1.size();
        int n = word2.size();
        vector<vector<int>> dp(m, vector<int>(n));
        return BFS(m - 1, n - 1, word1, word2, dp);
    }
};
```


## 动态规划

```C++
class Solution {
public:
    int minDistance(string word1, string word2) {
        int m = word1.size();
        int n = word2.size();
        vector<vector<int>> dp(m + 1, vector<int>(n + 1));
        for (int i = 0; i <= m; ++i)
            dp[i][0] = i;
        for (int j = 0; j <= n; ++j)
            dp[0][j] = j;

        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                if (word1[i] == word2[j])
                    dp[i + 1][j + 1] = dp[i][j];
                else
                    dp[i + 1][j + 1] = min(
                        dp[i + 1][j],        // 插入
                        min(
                            dp[i][j + 1],    // 删除
                            dp[i][j]         // 替换
                        )
                    ) + 1;
            }
        }

        return dp[m][n];
    }
};
```

## 状态压缩 空间复杂度: O(n)
我不会.