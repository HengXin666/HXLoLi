# 3085. 成为 K 特殊字符串需要删除的最少字符数
原题: [3085. 成为 K 特殊字符串需要删除的最少字符数](https://leetcode.cn/problems/minimum-deletions-to-make-string-k-special/)

第 389 场周赛 Q3 中等

---

给你一个字符串`word`和一个整数`k`。

如果`|freq(word[i]) - freq(word[j])| <= k`对于字符串中所有下标`i`和`j` 都成立，则认为`word`是`k`特殊字符串。

此处，`freq(x)`表示字符`x`在`word`中的出现频率，而`|y|`表示`y`的绝对值。

返回使`word`成为`k`特殊字符串 需要删除的字符的最小数量。

## 示例 1：

输入：word = "aabcaba", k = 0

输出：3

解释：可以删除 2 个 "a" 和 1 个 "c" 使 word 成为 0 特殊字符串。word 变为 "baba"，此时 freq('a') == freq('b') == 2。

## 示例 2：

输入：word = "dabdcbdcdcd", k = 2

输出：2

解释：可以删除 1 个 "a" 和 1 个 "d" 使 word 成为 2 特殊字符串。word 变为 "bdcbdcdcd"，此时 freq('b') == 2，freq('c') == 3，freq('d') == 4。

## 示例 3：

输入：word = "aaabaaa", k = 2

输出：1

解释：可以删除 1 个 "b" 使 word 成为 2特殊字符串。因此，word 变为 "aaaaaa"，此时每个字母的频率都是 6。

## 提示：
$
1 <= word.length <= 10^5\\
0 <= k <= 10^5\\
word 仅由小写英文字母组成。
$

# 题解
## 我比赛时的代码
显然我已经坐牢 $1h$ 了, 我以为是贪心?模拟? 所以 找到条件`最大频率 - 最小频率 <= k`, 然后就是每次都找到 `chNum` 数组中 除了0以外的 所有值的 最大值 和 最小值, 然后就是和 k 的某些添加判断, 可以过样例, 但是是错的!

```C++
class Solution {
public:
    int minimumDeletions(string word, int k) {
        vector<int> chNum(26);
        int res = 0;
        for (char& it: word) {
            ++chNum[it - 'a'];
        }
        
        while (1) {
            int maxN = -1;
            int minN = -1;
            sort(chNum.begin(), chNum.end(), [](const int& a, const int &b){
               return a > b; 
            });
            
            for (int i = 0; i < 26; ++i) {
                if (chNum[i]) {
                    if (maxN == -1 || chNum[maxN] < chNum[i])
                        maxN = i;
                    
                    if (minN == -1 || chNum[minN] > chNum[i])
                        minN = i;
                }
                else
                    break;
            }

            if (chNum[maxN] - chNum[minN] > k) {
                // 减少 chNum[maxN] 的数量
                ++res;
                if (chNum[maxN + 1] - chNum[minN] > k || 
                   chNum[minN - 1] - chNum[minN] > k)
                    --chNum[minN];
                else
                    --chNum[maxN];
            }
            else
                return res;
        }
    }
};
```

我还是不甘心, 但丝毫没有怀疑思路的正确性, 单纯就是以为条件还差点, 就是妹发现... (不然怎么坐牢? 要是知道写不出来就不写了呀~)

然后我又加了一个new东西: 求出需要修改的字符个数`tmp[i][0]` 和 修改后牵扯(影响)到几个人`tmp[i][0]` 然后按 [0] 为主 [1] 为副 排序.

最后再一波操作判断...(~~**DaSaBi**~~)

```C++
class Solution {
public:
    int minimumDeletions(string word, int k) {
        vector<int> chNum(26);
        int res = 0;
        for (char& it: word) {
            ++chNum[it - 'a'];
        }
        
        while (1) {
            // int maxN = -1;
            // int minN = -1;
            sort(chNum.begin(), chNum.end(), [](const int& a, const int &b){
               return a > b; 
            });
            
            vector<vector<int>> tmp(26, vector<int>(2));
            
            for (int i = 0; i < 26; ++i) {
                if (chNum[i]) {
                    for (int j = 25; j >= 0; --j) {
                        if (chNum[j]) {
                            int x = abs(chNum[i] - chNum[j]) - k;
                            tmp[i][0] = max(tmp[i][0], x);   
                            if (x > 0)
                                ++tmp[i][1];
                        }
                    }
                }
                else
                    break;
            }
            
            int tmp_max = -1;
            int minN = 0;
            for (int j = 0; j < 26; ++j) {
                if (tmp[j][0] > 0) {
                    if (tmp_max == -1 || tmp[tmp_max][0] < tmp[j][0]
                       || (tmp[tmp_max][0] == tmp[j][0] && 
                          tmp[tmp_max][1] < tmp[j][1]))
                        tmp_max = j;
                }
            }
            
            if (tmp_max == -1)
                return res;
            else {
                for (int j = 0; j < 26; ++j) {
                    if (tmp[j][0] > 0 && tmp[j][0] != tmp[tmp_max][0]) {
                        minN += tmp[j][0];
                    }
                }
                if (tmp[tmp_max][0] >= minN || minN == 0) {
                    --chNum[tmp_max];
                    ++res;
                }
                else {
                    for (int j = 0; j < 26; ++j) {
                        if (tmp[j][0] > 0 && tmp[j][0] != tmp[tmp_max][0]) {
                            res += chNum[j];
                            chNum[j] = 0;
                            tmp[j][0] = 0;
                        }
                    }
                }
            }
        }
    }
};

/* 测试用例
"aabcaba"
0
"dabdcbdcdcd"
2
"aaabaaa"
2
"aaaaa"
2
"ahahnhahhah"
1
"aaabaaac"
0
"aaaaaaabbcc"
0
*/
```

## AC代码
看了 0x3f大佬的题解后, tmd 后知后觉...

**定理**：必然有一种字母是不需要删除的。

**反证法**：如果每种字母都至少删除一个，那么可以都增加一，不影响字母数量之差。

就比如: `aaaabaa`, `k = 1`, 只需要删除 `b` 这个字符就OK了, 因为隐式的存在`|freq(word[i]) - freq(word[j])| <= k`即`|freq('a') - freq('a')| = 0 <= k`

所以有: (灵神的代码是逆向思维写的, 但是我写着写着就变成正向了qwq)

```C++
class Solution {
public:
    int minimumDeletions(string word, int k) {
        // 删除的最少字符数 == word.size() - 保留的最多字符数
        // 需要的是 最大频率 - 最小频率 <= k
        // 不妨 设 字符x 最小频率为 f[x], 那么最大频率为 f[x] + k
        /*
        1. 如果 f[i] < f[x] 那么就直接舍弃(全部删除)
           需要删除 f[i] 次
            例如: aaabaa, k=1 : f[a] = 5, f[b] = 1
                1) 选择 f[a] 为 最小频率, 那么 删除 f[b] = 1
                2) 选择 f[b] 为 最小频率, 那么 删除 f[a] - min(f[b] + k, f[a]) = 3
        
        2. 如果 f[i] >= f[x] 那么就需要将 f[i] 控制到 [f[x], f[x] + k] 内, 多余的要删除
        */

        int del = 1e7;
        vector<int> ch(26);
        for (char& it : word)
            ++ch[it - 'a'];
        
        sort(ch.begin(), ch.end(), [](const int& a, const int& b){return a > b;});

        // 枚举所有可能的 f[i] 为最小频率, 找出需要删掉的最少次数
        for (int i = 0; i < 26; ++i) {
            int tmp_min = 0; // 需要删除的字符数量
            for (int j = 0; j < 26; ++j) {
                if (!ch[j])
                    break;
                
                if (ch[j] < ch[i]) { // 情况 1
                    tmp_min += ch[j];
                } else { // 情况 2
                    // 如果 大于 [f[i], f[i] + k] 区间, 就需要删除 即 min 的 ch[i] + k (最大合法数目), ch[j] 减它 就是 要删除的个数嘛~
                    tmp_min += ch[j] - min(ch[i] + k, ch[j]);
                }
            }

            del = min(del, tmp_min); // 寻找最小需要删除的
        }

        return del;
    }
};
```
