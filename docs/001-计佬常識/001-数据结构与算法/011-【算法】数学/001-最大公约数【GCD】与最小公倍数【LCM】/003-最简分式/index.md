# 最简分式
> [!TIP]
> <span style="color:yellow">有时候可能出现使用`实数`计算分式的情况, 在特别时候, 我们不能使用`float`, 甚至不能使用`double`(精度损失).</span>

那这种时候, 如果我们需要哈希这个分数, 可以使用`{分子, 分母}`的整数来表示, 只需要保证这个是 **最简分式** 即可.

## 如何求解最简分式?

如果分式可化简, 那么说明有`公因子`, 故我们分子分母同时`除以`**最大的公因子**即可, 最大的公因子 即 最大公约数 即 $gcd$

示例: 题目: [2001. 可互换矩形的组数](https://leetcode.cn/problems/number-of-pairs-of-interchangeable-rectangles/)(不能使用`float`)

```C++
class Solution {
public:
    long long interchangeableRectangles(vector<vector<int>>& rectangles) {
        // r[i][0] / r[i][1] == r[j][0] / r[j][1]
        map<pair<int, int>, int> cnt; // unordered没有pair<int, int>的哈希函数...
        long long res = 0;
        for (auto& it : rectangles) {
            int g = gcd(it[0], it[1]); // 化简为最简分式
            res += cnt[{it[0] / g, it[1] / g}]++;
        }
        return res;
    }
};
```

插播一个小技巧, 这题的数据范围 10^5, 可以使用十进制压缩, 用`ll`这样一个数存储两个数进去!

```C++
class Solution {
    using ll = long long;
    const int ZZZ = 1e9;
public:
    long long interchangeableRectangles(vector<vector<int>>& rectangles) {

        unordered_map<ll, int> cnt;
        long long res = 0;
        for (auto& it : rectangles) {
            int g = gcd(it[0], it[1]); // 化简为最简分式
            res += cnt[(ll)it[0] / g * ZZZ + it[1] / g]++;
        }
        
        return res;
    }
};
```
