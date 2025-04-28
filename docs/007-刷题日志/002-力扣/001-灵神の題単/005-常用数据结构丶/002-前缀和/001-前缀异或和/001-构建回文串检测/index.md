# 1177. 构建回文串检测

链接: [1177. 构建回文串检测](https://leetcode.cn/problems/can-make-palindrome-from-substring/)

算术评级: 4 第 152 场周赛 Q3 `1848`

给你一个字符串 s，请你对 s 的子串进行检测。

每次检测，待检子串都可以表示为 $queries[i] = [left, right, k]$。我们可以 重新排列 子串 $s[left], ..., s[right]$，并从中选择 **最多** $k$ 项替换成任何小写英文字母。 

如果在上述检测过程中，子串可以变成回文形式的字符串，那么检测结果为`true`，否则结果为`false`。

返回答案数组 `answer[]`，其中 $answer[i]$ 是第 $i$ 个待检子串 $queries[i]$ 的检测结果。

注意：在替换时，子串中的每个字母都必须作为 独立的 项进行计数，也就是说，如果 $s[left..right] = "aaa"$ 且 $k = 2$，我们只能替换其中的两个字母。（另外，任何检测都不会修改原始字符串 $s$，可以认为每次检测都是独立的）

提示：

- 1 <= s.length, queries.length <= 10^5
- 0 <= queries[i][0] <= queries[i][1] < s.length
- 0 <= queries[i][2] <= s.length
- s 中只有小写英文字母

# 题解
## 0x3f
- [一步步优化！从前缀和到前缀异或和（附题单！）](https://leetcode.cn/problems/can-make-palindrome-from-substring/solutions/2309725/yi-bu-bu-you-hua-cong-qian-zhui-he-dao-q-yh5p)

<style>
  .container {
    width: 100%;
    position: relative;
    display: inline-block;
  }
  
  .question-mark {
    display: inline-block;
    transition: transform 3s ease;
  }
  
  .answer-box {
    text-shadow: 2px 2px 2px red;
    padding: 10px;
    background-color: 9999;
    border: 1px solid #990099;
    border-radius: 5px;
    opacity: 0;
    transition: left 2s ease, opacity 2s ease;
  }
  
  .container:hover .question-mark {
    transform: rotate(540deg);
  }
  
  .container:hover .answer-box {
    opacity: 1;
  }
</style>

<div class="container">
  
### 一、思考
回文意味着什么<span class="question-mark">?</span>
  
<div class="answer-box">
    
回文意味着从左往右第 $i$ 个字母和从右往左第 $i$ 个字母是相同的。（回文串关于回文中心是对称的。）
  
</div>

题目允许重新排列字母，那么以字母 $a$ 为例，它要如何排列<span class="question-mark">?</span> 偶数个 $a$ 要如何排<span class="question-mark">?</span> 奇数个 $a$ 要如何排列<span class="question-mark">?</span>

<div class="answer-box">
    
如果有偶数个 $a$，那么可以均分成两部分，分别放置在字符串的中心对称位置上。例如有 4 个 $a$，可以在字符串的最左边放置 2 个 $a$，最右边放置 2 个 $a$，这样字符串中的 $a$ 是回文的。其它字母如果出现偶数次，也同理。

如果有奇数个 $a$，多出的一个 $a$ 要单独拿出来讨论:

- 假如只有 $a$ 出现奇数次，其它字母都出现偶数次。此时字符串的长度一定是奇数，那么可以把多出的这个 $a$ 放在字符串的中心，我们仍然可以得到一个回文串，无需替换任何字母。

- 如果有两种字母出现奇数次（假设是字母 $a, b$ ），由于多出的一个 $a$ 和一个 $b$ 无法组成回文串，可以把一个 $b$ 改成 $a$ (或者把一个 $a$ 改成 $b$ )，这样 $a$ 和 $b$ 就都出现偶数次了。

- 如果有三种字母出现奇数次（假设是字母 $a, b, c$ ），把一个 $b$ 改成 $c$，就转换成只有 $a$ 出现奇数次的情况了。

</div>

什么情况下一定要替换字母<span class="question-mark">?</span> 要替换多少个<span class="question-mark">?</span> 

<div class="answer-box">

- 一般地，如果有 $m$ 种字母出现奇数次，只需修改其中 $\left\lfloor\dfrac{m}{2}\right\rfloor$ 个字母。换句话说，如果第 $i$ 次询问有 $\left\lfloor\dfrac{m}{2}\right\rfloor\le k$ ，那么 $answer[i]$ 为真，反之为假。

</div>

如何快速求出子串中每种字母的个数<span class="question-mark">?</span>

<div class="answer-box">

可以创建 $26$ 个前缀和数组，分别统计每种字母。以字母 $a$ 为例，在计算前缀和时，如果 $s[i]=a$ 就视作 1，否则视作 0。

</div>

</div>

故有:

```C++
class Solution {
public:
    vector<bool> canMakePaliQueries(string s, vector<vector<int>>& queries) {
        vector<vector<int>> hashSumArr(26, vector<int>(s.size() + 1));
        for (int i = 0; i < s.size(); ++i)
            for (int j = 0; j < 26; ++j)
                hashSumArr[j][i + 1] = hashSumArr[j][i] + (s[i] - 'a' == j);

        vector<bool> res(queries.size());
        for (int i = 0; i < res.size(); ++i) {
            int m = 0; // 出现奇数个字母的个数
            for (int j = 0; j < 26; ++j)
                m += (hashSumArr[j][queries[i][1] + 1] - hashSumArr[j][queries[i][0]]) & 1;
            res[i] = queries[i][2] >= (m >> 1);
        }
        return res;
    }
};
```

### 二、进一步思考
由于只关心每种字母出现次数的奇偶性，所以不需要在前缀和中存储每种字母的出现次数，只需要保存每种字母出现次数的奇偶性。

突然发现`std::array`可以直接等于, 就不需要那么多 $\sum for \times 26$ 了
```C++
class Solution {
public:
    vector<bool> canMakePaliQueries(string s, vector<vector<int>> &queries) {
        int n = s.length(), q = queries.size();
        vector<array<int, 26>> sum(n + 1);
        for (int i = 0; i < n; i++) {
            sum[i + 1] = sum[i];
            sum[i + 1][s[i] - 'a']++;
            sum[i + 1][s[i] - 'a'] %= 2; // 偶数是 0
        }

        vector<bool> ans(q);
        for (int i = 0; i < q; i++) {
            auto &query = queries[i];
            int left = query[0], right = query[1], k = query[2], m = 0;
            for (int j = 0; j < 26; j++)
                m += sum[right + 1][j] != sum[left][j];
            ans[i] = m / 2 <= k;
        }
        return ans;
    }
}; // By 0x3f
```

### 三。前缀异或和
由于异或运算满足 1 和 0 的结果是 1 ，而 0 和 0，以及 1 和 1 的结果都是 0 ，所以可以用异或替换上面的减法。

```C++
class Solution {
public:
    vector<bool> canMakePaliQueries(string s, vector<vector<int>> &queries) {
        int n = s.length(), q = queries.size();
        vector<array<int, 26>> sum(n + 1);
        for (int i = 0; i < n; i++) {
            sum[i + 1] = sum[i];
            sum[i + 1][s[i] - 'a'] ^= 1; // 奇数变偶数，偶数变奇数
        }

        vector<bool> ans(q);
        for (int i = 0; i < q; i++) {
            auto &query = queries[i];
            int left = query[0], right = query[1], k = query[2], m = 0;
            for (int j = 0; j < 26; j++)
                m += sum[right + 1][j] ^ sum[left][j];
            ans[i] = m / 2 <= k;
        }
        return ans;
    }
}; // By 0x3f
```

又因为只有26个字母, 可以使用二进制压缩! 从而「并行运算」!

```C++
class Solution {
public:
    vector<bool> canMakePaliQueries(string s, vector<vector<int>> &queries) {
        int n = s.length(), q = queries.size(), sum[n + 1];
        sum[0] = 0;
        for (int i = 0; i < n; i++) {
            int bit = 1 << (s[i] - 'a');
            sum[i + 1] = sum[i] ^ bit; // 该比特对应字母的奇偶性：奇数变偶数，偶数变奇数
        }

        vector<bool> ans(q); // 预分配空间
        for (int i = 0; i < q; i++) {
            auto &query = queries[i];
            int left = query[0], right = query[1], k = query[2];
            int m = __builtin_popcount(sum[right + 1] ^ sum[left]);
            ans[i] = m / 2 <= k;
        }
        return ans;
    }
}; // By 0x3f
```
