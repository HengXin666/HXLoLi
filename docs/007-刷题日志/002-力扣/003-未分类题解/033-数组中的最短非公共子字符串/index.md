# 3076. 数组中的最短非公共子字符串
第388场周赛Q3 中等

链接: [3076. 数组中的最短非公共子字符串](https://leetcode.cn/problems/shortest-uncommon-substring-in-an-array/description/)

## 题目

给你一个数组 arr ，数组中有 n 个 非空 字符串。

请你求出一个长度为 n 的字符串 answer ，满足：

answer[i] 是 arr[i] 最短 的子字符串，且它不是 arr 中其他任何字符串的子字符串。如果有多个这样的子字符串存在，answer[i] 应该是它们中字典序最小的一个。如果不存在这样的子字符串，answer[i] 为空字符串。
请你返回数组 answer 。

## 示例 1：

输入：arr = ["cab","ad","bad","c"]<br>
输出：["ab","","ba",""]<br>
解释：求解过程如下：
- 对于字符串 "cab" ，最短没有在其他字符串中出现过的子字符串是 "ca" 或者 "ab" ，我们选择字典序更小的子字符串，也就是 "ab" 。
- 对于字符串 "ad" ，不存在没有在其他字符串中出现过的子字符串。
- 对于字符串 "bad" ，最短没有在其他字符串中出现过的子字符串是 "ba" 。
- 对于字符串 "c" ，不存在没有在其他字符串中出现过的子字符串。

## 示例 2：

输入：arr = ["abc","bcd","abcd"]<br>
输出：["","","abcd"]<br>
解释：求解过程如下：
- 对于字符串 "abc" ，不存在没有在其他字符串中出现过的子字符串。
- 对于字符串 "bcd" ，不存在没有在其他字符串中出现过的子字符串。
- 对于字符串 "abcd" ，最短没有在其他字符串中出现过的子字符串是 "abcd" 。
 

提示：

$
n == arr.length\\
2 <= n <= 100\\
1 <= arr[i].length <= 20\\
arr[i] 只包含小写英文字母。\\
$

# 题解
## 我的
看数据规模, 得: 暴力!

1. 先生成所有`arr[i]`的字符串的子串到哈希表中

2. 然后判断选择出`arr[i]`的所有**没有在其他字符串中出现过的**子字符串(即上哈希表中没有出现的), 然后排序出最短的、字典序最小的即可

时间复杂度: $O(N^2+N^2*NlogN)$ 即 $O(N^3logN)$

AC代码: [2024年3月11日] 双100%
```C++
class Solution {
public:
    void helper(const std::string& str, map<string, int>& hash){
        set<string> cache;
        for(int i = 0; i < str.size(); i++) {
            std::string tmp;
            for(int j = i; j < str.size(); j++) {
                tmp += str[j];
                cache.insert(tmp);
            }
        }

        // 防止多次统计同一个子字符串的个数
        // 例如 "bbb", 那么按照上面的生成方法会有子字符串 "bbb" * 1, "bb" * 2, "b" * 3
        for (const string& it : cache) {
            ++hash[it];
        }
    }
    
    vector<string> shortestSubstrings(vector<string>& arr) {
        vector<string> res;
        map<string, int> hash; // 对于arr[i]中所有子字符串的出现次数
        for (string& it : arr) {
            helper(it, hash);
        }
        
        for (string& it : arr) {
            vector<string> cache;
            for(int i = 0; i < it.size(); i++) {
                std::string tmp;
                for(int j = i; j < it.size(); j++) {
                    tmp += it[j];
                    if (hash[tmp] == 1) { // 这个子字符串只出现一次 => 只能是本arr[i]中出现的
                        cache.push_back(tmp);
                    }
                }
            }
            
            sort(cache.begin(), cache.end(), [](const string& s1, const string& s2) {
               if (s1.size() < s2.size() || (s1.size() == s2.size() && s1 < s2))
                   return 1;
                return 0;
            });
            
            if (cache.size() > 0)
                res.push_back(cache[0]);
            else
                res.push_back(""); // 没有返回添加的字符串
        }
        
        return res;
    }
};
```

### 生成字符串的所有子字符串

这个是模版
```C++
vector<string> helper(const std::string& str) {
    vector<string> res;
    for(int i = 0; i < str.size(); ++i) {
        std::string tmp;
        for(int j = i; j < str.size(); ++j) {
            tmp += str[j];
            res.push_back(tmp);
        }
    }

    return std::move(res);
}
```


## 大佬的
题解很多, 比如 后缀数组/后缀自动机(时间复杂度为: $O(\sum^{arr.size()}_{i=0}{arr[i].size()})$ ) 字典树

请自己学习, 当前我不会...