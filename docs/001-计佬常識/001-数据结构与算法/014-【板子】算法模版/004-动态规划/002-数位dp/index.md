# 数位 DP
## 0x3f v1.0模版

可以统计, 对于范围 $[a , n]$, (其中 $a$ 为常数) 的数的条件的统计.

该条件和`数位`有关 (因此数据范围会很吓人, 如 $n < 10^{10^5}$, 但实际上只是 $\log_{10}{n}$ 的时间复杂度)

> [!TIP]
> 本模版只需要记忆化`[i]`即可, 不需要记忆化`isLimt、isNum`, 因为在`isNum == false`的条件下的递归只会进行一次, 剩下的都是`isLimt == false && isNum == true`的多次计算, 因此记忆化只针对`isLimt == false && isNum == true`即可.

- [2376. 统计特殊整数](https://leetcode.cn/problems/count-special-integers/)

模版参数需要十分理解!

```C++
auto dfs = [&](
    this auto&& dfs, 
    int i,          // 当前为第几位 (一般从 i = 0 为入口, 也就是先算最高位)
    bool isLimt,    // 之前的数位是否为最大 (如题目要求 n < 123, 现在 i = 2, isLimt = ture, 那么当前位就不能超过`3`, 这相当于前面已经是`12`了)
    bool isNum      // 之前是否填了数字? 如果为 flase, 则表示前面留空, 类似于表示前面全部填零(前导零)
) -> int {};
```


```C++
class Solution {
public:
    int countSpecialNumbers(int N) {
        auto str = to_string(N);
        int n = str.size();
        vector<vector<int>> memo(n, vector<int>(1 << 10, -1));
        auto dfs = [&](this auto&& dfs, 
            int i, int vis, bool isLimt, bool isNum) -> int {
            if (i == n)
                return isNum;
            if (!isLimt && isNum && memo[i][vis] != -1)
                return memo[i][vis];
            int res = 0;
            if (!isNum)
                res = dfs(i + 1, vis, false, false);
            int up = isLimt ? str[i] - '0' : 9;

            // 如果去前面有数字, 那么可以选0
            for (int j = !isNum; j <= up; ++j) {
                if ((vis >> j) & 1) {
                    res += dfs(i + 1, vis ^ (1 << j), isLimt && j == up, true);
                }
            }
            if (!isLimt && isNum)
                memo[i][vis] = res;
            return res;
        };
        return dfs(0, (1 << 10) - 1, true, false);
    }
};
```

- [902. 最大为 N 的数字组合](https://leetcode.cn/problems/numbers-at-most-n-given-digit-set/)
 
```C++
class Solution {
public:
    int atMostNGivenDigitSet(vector<string>& digits, int N) {
        auto str = to_string(N);
        int n = str.size();
        vector<int> memo(n, -1);
        auto dfs = [&](this auto&& dfs, int i, bool isLimt, bool isNum) -> int {
            if (i == n)
                return isNum;
            if (!isLimt && isNum && memo[i] != -1)
                return memo[i];
            int res = 0;
            if (!isNum) {
                res = dfs(i + 1, false, false);
            }
            char up = isLimt ? str[i] : '9';
            for (auto& d : digits) {
                if (d[0] > up)
                    break;
                res += dfs(i + 1, isLimt && d[0] == up, true);
            }
            if (!isLimt && isNum)
                memo[i] = res;
            return res;
        };
        return dfs(0, true, false);
    }
};
```
