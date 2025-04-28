# 3144. 分割字符频率相等的最少子字符串
链接: [3144. 分割字符频率相等的最少子字符串](https://leetcode.cn/problems/minimum-substring-partition-of-equal-character-frequency/)

给你一个字符串 s ，你需要将它分割成一个或者更多的 平衡 子字符串。比方说，s == "ababcc" 那么 ("abab", "c", "c") ，("ab", "abc", "c") 和 ("ababcc") 都是合法分割，但是 ("a", "bab", "cc") ，("aba", "bc", "c") 和 ("ab", "abcc") 不是，不平衡的子字符串用粗体表示。

请你返回 s 最少 能分割成多少个平衡子字符串。

注意：一个 平衡 字符串指的是字符串中所有字符出现的次数都相同。

提示:
- 1 <= s.length <= 1000
- s 只包含小写英文字母。

# 题解
## 我的(赛后再战版)
### 没有任何优化的八嘎版
需要改进:
- 此处使用了两个for循环, 一个切, 一个计算字母出现次数, 实际上可以优化成一个for(内部的for因为又重新从0开始计算了)

- 判断是否平衡: 出现字母次数相同的数学判断方法见下面(最终版)

```C++
class Solution {
    const int inf = 1e9;
public:
    int minimumSubstringsInPartition(string s) {
        // 划分型DP 记忆化搜索
        // 枚举切分的位置, 如果可以(平衡)就切
        // 就得到子问题了
        // dfs(i) -> 切: min(dfs(i - k)) + 1
        //        -> 不切 (已经平衡)
        vector<int> memo(s.size() + 1, inf);
        function<int(int)> dfs = [&](int i) -> int {
            if (i <= 0)
                return i + 1;

            if (memo[i] != inf)
                return memo[i];
            // 枚举切分的位置 k
            // [0, 切:[k, i]]
            for (int k = i; k >= 0; --k) {
                int cnt[26] = {0};
                for (int l = i; l >= k; --l) {
                    ++cnt[s[l] - 'a'];
                }
                int tmp = -1; // 判断是否平衡
                for (int& it : cnt) {
                    if (it) {
                        if (tmp == -1)
                            tmp = it;
                        else
                            if (tmp != it) {
                                tmp = -2;   
                                break;
                            }
                    }
                }
                if (tmp == -2)
                    continue;

                memo[i] = min(memo[i], dfs(k - 1) + 1);
            }
            return memo[i];
        };

        return dfs(s.size() - 1);
    }
};
```


### 最终
```C++
class Solution {
    const int inf = 0;
public:
    int minimumSubstringsInPartition(string s) {
        // 划分型DP 记忆化搜索
        // 枚举切分的位置, 如果可以(平衡)就切
        // 就得到子问题了
        // dfs(i) -> 切: min(dfs(i - k)) + 1
        //        -> 不切 (已经平衡)
        vector<int> memo(s.size(), inf);
        function<int(int)> dfs = [&](int i) -> int {
            if (i < 0)
                return 0;

            if (memo[i] != inf)
                return memo[i];
            // 枚举切分的位置 k
            // [0, 切:[k, i]]
            memo[i] = 1e9;
            // 判断是否平衡
            // 如果 [k, i] 有 a 种字母, 字母最大出现为 x
            // 当且仅当 a * x = i - k + 1 (子字符串长度) 时, 为平衡
            int cnt[26] = {0};
            int maxx = 0, a = 0;
            for (int k = i; k >= 0; --k) {
                a += ++cnt[s[k] - 'a'] == 1;
                maxx = max(maxx, cnt[s[k] - 'a']);

                if (a * maxx == i - k + 1)
                    memo[i] = min(memo[i], dfs(k - 1) + 1);
            }
            return memo[i];
        };

        return dfs(s.size() - 1);
    }
};
```

### 翻译为递推

```C++
class Solution {
public:
    int minimumSubstringsInPartition(string s) {
        // 划分型DP 记忆化搜索
        // 枚举切分的位置, 如果可以(平衡)就切
        // 就得到子问题了
        // dfs(i) -> 切: min(dfs(i - k)) + 1
        //        -> 不切 (已经平衡)
        
        // f[i + 1] = min(f[k] + 1, 0 <= k <= i)
        vector<int> f(s.size() + 1, 1e9);
        f[0] = 0;
        for (int i = 0; i < s.size(); ++i) {
            int maxx = 0, k = 0, cnt[26] = {0};
            // for (int j = 0; j <= i; ++j) { 为什么这样就不对
            for (int j = i; j >= 0; --j) {
                k += ++cnt[s[j] - 'a'] == 1;
                maxx = max(maxx, cnt[s[j] - 'a']);
                if (maxx * k == i - j + 1) // j + 1
                    f[i + 1] = min(f[i + 1], f[j] + 1); // i - j
            }
        }
        return f[s.size()];
    }
};
```

疑问: 为什么需要从 $j: i \to 0$ 开始 $for$ ?, $j: 0 \to i$ 就不对? 

> 在这段代码中, $j$ 代表切分位置，$i$ 代表当前字符串的末尾位置。从 $j: i \to 0$ 开始 $for$ 的原因是因为这样能够确保在切分时我们总是在末尾向前考虑，这样可以更有效地保证切分的平衡性。
>
> 让我们来看一下在 $j: i \to 0$ 的情况下会发生什么：
>
> 1. 我们从当前字符串的末尾开始，逐渐向前考虑可能的切分位置。
> 2. 每次考虑一个切分位置 $j$，我们计算从 $j$ 到 $i$ 这段子字符串的特性，这个特性是用于判断是否能够切分并且保持平衡。
> 3. 如果当前考虑的子字符串可以切分且保持平衡，则更新切分位置的最优解。
> 4. 继续向前考虑下一个可能的切分位置，重复以上步骤。
> 
> 在这个过程中，我们始终保持着对字符串末尾的考虑，因此我们可以确保在每次切分时，我们都是在尽可能远离末尾的位置进行的。这样做的好处是，我们有更高的概率找到最优的切分位置，因为我们给了字符串更多的“空间”来进行切分，而不会过早地将字符串分割得太细。
> 
> 相比之下，如果我们从 $j: 0 \to i$ 开始 $for$ 的话，我们会在每次切分时都从字符串的开头开始考虑，这可能会导致过早地将字符串分割得太细，错过了更优的切分位置。
> 
> 因此，通过从 $j: i \to 0$ 开始 $for$，我们能够更好地控制切分的过程，提高找到最优解的可能性。(By $GPT-\text{4o}$ )