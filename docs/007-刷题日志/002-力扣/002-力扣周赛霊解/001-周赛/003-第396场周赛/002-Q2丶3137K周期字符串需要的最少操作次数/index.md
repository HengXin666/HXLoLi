# 3137. K 周期字符串需要的最少操作次数
链接: [3137. K 周期字符串需要的最少操作次数](https://leetcode.cn/problems/minimum-number-of-operations-to-make-word-k-periodic/)

给你一个长度为 n 的字符串 word 和一个整数 k ，其中 k 是 n 的因数。

在一次操作中，你可以选择任意两个下标 $i$ 和 $j$，其中`0 <= i, j < n`，且这两个下标都可以被 $k$ 整除，然后用从 $j$ 开始的长度为 $k$ 的子串替换从 $i$ 开始的长度为 $k$ 的子串。也就是说，将子串`word[i..i + k - 1]`替换为子串`word[j..j + k - 1]`。

返回使 $word$ 成为 $K$ 周期字符串 所需的 **最少** 操作次数。

如果存在某个长度为`k`的字符串`s`，使得`word`可以表示为任意次数连接`s`，则称字符串`word`是`K`周期字符串 。例如，如果`word == "ababab"`，那么`word`就是 `s = "ab"`时的`2`周期字符串 。

提示:
- 1 <= n == word.length <= 105
- 1 <= k <= word.length
- k 能整除 word.length 。
- word 仅由小写英文字母组成。

# 题解
## 我的
值得注意的是, 本题的切入点是`0 <= i, j < n`(注意没有`i < j`)并且`k 能整除 word.length`

那么只需要贪心的, 从0开始的记录每一个长度为k的子字符串的出现次数, 然后返回 $字符串总长度/k - 出现次数最多的$ 即可!

```C++
class Solution {
public:
    int minimumOperationsToMakeKPeriodic(string word, int k) {
        unordered_map<string, int> hash;
        int maxx = 0;
        for (int i = 0; i < word.size(); i += k)
            maxx = max(maxx, ++hash[word.substr(i,k)]);
        return word.size() / k - maxx;
    }
};
```

那个男人的代码和我差不多, 就是他选择 $[i - k, i]$ 这部分作为子串