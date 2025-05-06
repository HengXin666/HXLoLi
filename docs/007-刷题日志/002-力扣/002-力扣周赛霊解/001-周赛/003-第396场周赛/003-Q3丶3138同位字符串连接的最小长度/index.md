# 3138. 同位字符串连接的最小长度
链接: [3138. 同位字符串连接的最小长度](https://leetcode.cn/problems/minimum-length-of-anagram-concatenation/)

给你一个字符串 $s$，它由某个字符串 $t$ 和若干 $t$ 的 **同位字符串** 连接而成。

请你返回字符串 $t$ 的 **最小** 可能长度。

同位字符串 指的是重新排列一个单词得到的另外一个字符串，原来字符串中的每个字符在新字符串中都恰好只使用一次。

提示:
- 1 <= s.length <= $10^5$
- s 只包含小写英文字母。

# 题解
## 垃圾题目翻译! 比赛的时候根本看不懂题目在说什么!
再加上测试用例非常弱! 导致使用gcd也可以过, 我通过猜测错误提示找的规律... 你不wa还ac了, 又赛后重测, 我连改的机会都没有啊qwq(一开始看不明白题目然后看了第四题感觉可以, 就先写第四题了(然后发现没有那么简单...又跑回来读题目了))

```C++
class Solution {
    int gcd(int a, int b) {
        if (b == 0) 
            return a;
        return gcd(b, a % b);
    }
public:
    int minAnagramLength(string s) {
        vector<int> cnt(26);
        int maxx = 0;
        for (char c : s) {
            maxx = max(maxx, ++cnt[c - 'a']);
        }
        
        if (maxx == s.size())
            return 1;
        
        int max_gcd = maxx;
        for (int i = 0; i < 26; ++i) {
            if (cnt[i]) {
                for (int j = 0; j < 26; ++j) {
                    if (cnt[j] && i != j) {
                        max_gcd = min(max_gcd, gcd(cnt[i], cnt[j]));
                    }
                }
            }
        }
        int res = 0;
        for (int it : cnt) {
            if (it) {
                res += it / max_gcd;
            }
        }
        
        return res;
    }
};
```

使用`aabb`即可hk掉!

## 正解
> 由于 $10^5$ 以内的因子个数至多为 $128$ ( $83160$ 的因子个数)，所以我们可以暴力枚举 $n$ 的因子作为 $k$。

代码写的不忍直视... (实际上差不多, 下次还是不要用 $i$ 代表枚举的因数了, 容易搞混)
```C++
class Solution {
public:
    int minAnagramLength(string s) {
        // 枚举 可以整除 s 的数, 因为 s.size = k * i (1 <= i <= s.size, k 是整数)
        // 然后统计 所有子串的 字符出现频率 看看是否相等
        // abba -> ab + ba
        // aabb != aa + bb != ab + ab 应该是 aabb
        for (int i = 1, endIndex = s.size() - 1; i <= endIndex; ++i) {
            if (s.size() % i) // 不能整除
                continue;
            int cnt[26] = {0};
            for (int j = 0; j < i; ++j)
                ++cnt[s[j] - 'a'];
            
            for (int k = i * 2; k <= endIndex + i; k += i) {
                int tmp[26] = {0};
                
                for (int j = k - i; j < k; ++j) {
                    ++tmp[s[j] - 'a'];
                }

                for (int j = 0; j < 26; ++j) {
                    if (cnt[j] != tmp[j]) {
                        goto A;
                    }
                }
            }
            return i;
            A:
                ;
        }
        return s.size();
    }
};
```

看看那个男人:

```C++
class Solution {
public:
    int minAnagramLength(string s) {
        int n = s.length();
        for (int k = 1; k <= n / 2; k++) {
            if (n % k) {
                continue;
            }
            array<int, 26> cnt0{};
            for (int j = 0; j < k; j++) {
                cnt0[s[j] - 'a']++;
            }
            for (int i = k * 2; i <= n; i += k) {
                array<int, 26> cnt{};
                for (int j = i - k; j < i; j++) {
                    cnt[s[j] - 'a']++;
                }
                if (cnt != cnt0) {
                    goto next;
                }
            }
            return k;
            next:;
        }
        return n;
    }
};
// By 0x3f
```