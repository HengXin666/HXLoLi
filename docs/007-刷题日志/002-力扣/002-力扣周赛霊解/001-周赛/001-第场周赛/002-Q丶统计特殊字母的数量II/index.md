# 3121. 统计特殊字母的数量 II
给你一个字符串 word。如果 word 中同时出现某个字母 c 的小写形式和大写形式，并且 每个 小写形式的 c 都出现在第一个大写形式的 c 之前，则称字母 c 是一个 **特殊字母** 。

返回 word 中 特殊字母 的数量。

 
```
示例 1:
输入：word = "aaAbcBC"
输出：3
解释：
特殊字母是 'a'、'b' 和 'c'。

示例 2:
输入：word = "abc"
输出：0
解释：
word 中不存在特殊字母。

示例 3:
输入：word = "AbBCab"
输出：0
解释：
word 中不存在特殊字母。
```

提示：
```
1 <= word.length <= 2 * 10^5
word 仅由小写和大写英文字母组成。
```

# 题解
## 我的
wa了一发, 没有完全理解题目...

题目要求: 所有的大写字母必需保证后面没有对应的小写字母, 如`aaaaaAa`就不行

我依旧使用Q1的代码, 只需要保证如果这个

```C++
class Solution {
public:
    int numberOfSpecialChars(string word) {
        vector<int> arrx(26); // 小写
        vector<int> arrd(26); // 大写

        for (char& it : word) {
            if (it >= 'A' && it <= 'Z') // 大写
                ++arrd[it - 'A'];
            else { // 小写
                if (arrd[it - 'a']) {   // 如果之前出现过大写
                    arrx[it - 'a'] = 0; // 小写置零
                }
                else
                    ++arrx[it - 'a']; // 小写有
            }
        }

        int res = 0;
        for (int i = 0; i < 26; ++i)
            if (arrd[i] > 0 && arrx[i] > 0)
                ++res;
        return res;
    }
};
```

## 0x3f
> [两种方法：状态机 / 位运算（Python/Java/C++/Go）](https://leetcode.cn/problems/count-the-number-of-special-characters-ii/solutions/2749235/zhuang-tai-ji-on-yi-ci-bian-li-pythonjav-ajaz)
### 方法一: 状态机

<img src="https://pic.leetcode.cn/1713671840-HgbYWt-394C.png">

答案为状态为 2 的字母种数。

```C++
class Solution {
public:
    int numberOfSpecialChars(string word) {
        int ans = 0;
        int state[27]{};
        for (char c : word) {
            int x = c & 31; // 转成数字 1~26
            if (c & 32) { // 小写字母
                if (state[x] == 0) {
                    state[x] = 1;
                } else if (state[x] == 2) {
                    state[x] = -1;
                    ans--;
                }
            } else { // 大写字母
                if (state[x] == 0) {
                    state[x] = -1;
                } else if (state[x] == 1) {
                    state[x] = 2;
                    ans++;
                }
            }
        }
        return ans;
    }
};
```

### 方法二: 位运算
附 ASCII 的性质:
- 对于大写英文字母: 其二进制从右往左第 6 个比特值一定是 0。
- 对于小写英文字母: 其二进制从右往左第 6 个比特值一定是 1。
- 对于任何英文字母: 其小写字母二进制低 5 位，一定和其大写字母二进制低 5 位相等。

我参照我的方法写的: 二进制版本
```C++
class Solution {
public:
    int numberOfSpecialChars(string word) {
        int bitSet[2] = {0}; // 大0, 小1

        // 如果是小写, 则之前不能有大写
        for (char it : word) {
            if ((it >> 5) & 1) { // 小写
                // 判断之前是否出现对应的大写
                if (bitSet[0] & (1 << (it & 31))) {
                    // 出现过, 则保证它不合法
                    bitSet[1] &= ~(1 << (it & 31)); // 小写归0, 大写依旧是记录之前的状态
                } else {
                    // 没有出现, 小写计算
                    bitSet[1] |= (1 << (it & 31));
                }
            } else {
                // 大写计数
                bitSet[0] |= (1 << (it & 31));
            }
        }
        
        // 大写和小写的交集即 有 大小写
        return __builtin_popcount(bitSet[0] & bitSet[1]);
    }
};
```

灵神的: (妙妙妙!), 使用一个而外的集合记录不合法的!

```C++
class Solution {
public:
    int numberOfSpecialChars(string word) {
        int lower = 0, upper = 0, invalid = 0;
        for (char c : word) {
            int bit = 1 << (c & 31);
            if (c & 32) { // 小写字母
                lower |= bit;
                if (upper & bit) {  // c 也在 upper 中
                    invalid |= bit; // 不合法
                }
            } else { // 大写字母
                upper |= bit;
            }
        }
        // 从交集 lower & upper 中去掉不合法的字母 invalid
        return __builtin_popcount(lower & upper & ~invalid);
    }
};
```
