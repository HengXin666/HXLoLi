# 2312. 卖木头块
题目: [2312. 卖木头块](https://leetcode.cn/problems/selling-pieces-of-wood/description/)

第 298 场周赛 Q4 2363 困难

给你两个整数 m 和 n ，分别表示一块矩形木块的高和宽。同时给你一个二维整数数组 prices ，其中 prices[i] = [hi, wi, pricei] 表示你可以以 pricei 元的价格卖一块高为 $h_i$ 宽为 $w_i$ 的矩形木块。

每一次操作中，你必须按下述方式之一执行切割操作，以得到两块更小的矩形木块：

- 沿垂直方向按高度 **完全** 切割木块，或
- 沿水平方向按宽度 **完全** 切割木块

在将一块木块切成若干小木块后，你可以根据 prices 卖木块。你可以卖多块同样尺寸的木块。你不需要将所有小木块都卖出去。你 **不能** 旋转切好后木块的高和宽。

请你返回切割一块大小为 m x n 的木块后，能得到的 **最多** 钱数。

注意你可以切割木块任意次。

## 示例 1：

输入：m = 3, n = 5, prices = [[1,4,2],[2,2,7],[2,1,3]]<br>
输出：19<br>
解释：上图展示了一个可行的方案。包括：
- 2 块 2 x 2 的小木块，售出 2 * 7 = 14 元。
- 1 块 2 x 1 的小木块，售出 1 * 3 = 3 元。
- 1 块 1 x 4 的小木块，售出 1 * 2 = 2 元。
总共售出 14 + 3 + 2 = 19 元。
19 元是最多能得到的钱数。

## 示例 2：

输入：m = 4, n = 6, prices = [[3,2,10],[1,4,2],[4,1,3]]<br>
输出：32 <br>
解释：上图展示了一个可行的方案。包括：
- 3 块 3 x 2 的小木块，售出 3 * 10 = 30 元。
- 1 块 1 x 4 的小木块，售出 1 * 2 = 2 元。
总共售出 30 + 2 = 32 元。
32 元是最多能得到的钱数。
注意我们不能旋转 1 x 4 的木块来得到 4 x 1 的木块。
 

提示：

$
1 <= m, n <= 200\\
1 <= prices.length <= 2 * 10^4\\
prices[i].length == 3\\
1 <= h_i <= m\\
1 <= w_i <= n\\
1 <= price_i <= 10^6\\
所有 (h_i, w_i) 互不相同 。
$

# 题解

题目的核心意思是, 切割的时候**一切到底**

即`不可能`出现以下情况: 数字代表不同的木块 (即 L 型)

```C Error
切割前
0 0 0 0 0
0 0 0 0 0
0 0 0 0 0
0 0 0 0 0

切割后:
1 1 1 2 2
1 1 1 2 2
2 2 2 2 2
2 2 2 2 2
```

故切割后也必须是一个矩形!

所以有 大矩形 --切割-> 小矩形 显然可以dp (有子问题)

状态定义就是题目问什么就假设什么!

```C++
class Solution {
public:
// 其实就是让你把prices里的小木块塞到大木块里但是不能重叠，求能够获得的最大价值
    long long sellingWood(int m, int n, vector<vector<int>>& prices) {
        // dp
        // 定义切割一块高为i, 宽为j 可以得到的最多钱数 dp[i][j]
        // 那么可以直接卖(如果存在的话)
        // 也可以分开售卖
        vector<vector<int>> hash(m + 1 ,vector<int>(n + 1));

        for (auto& it : prices)
            hash[it[0]][it[1]] = it[2];
        
        vector<vector<long long>> dp(m + 1, vector<long long>(n + 1));
        for (int i = 1; i <= m; ++i) {
            for (int j = 1; j <= n; ++j) {
                dp[i][j] = hash[i][j];

                for (int k = 1; k < j; ++k) // 竖切 把 j 分成 k 和 j - k
                    dp[i][j] = max(dp[i][j], dp[i][k] + dp[i][j - k]);
                
                for (int k = 1; k < i; ++k) // 横切 把 i 分成 k 和 i - k
                    dp[i][j] = max(dp[i][j], dp[i - k][j] + dp[k][j]);
            }
        }

        return dp[m][n];
    }
};
```

## 优化

但实际上，横着切开，虽然位置不同，但得到的结果是相同的

即 `f[i][j - k] + f[i][k]` 和 `f[i][k] + f[i][j - k]` 的区别

所以 枚举 $k$ 的时候，只需要枚举到一半的位置，宽度至多为 $\left\lfloor\dfrac{i}{2}\right\rfloor$ , 高度至多为 $\left\lfloor\dfrac{j}{2}\right\rfloor$ 。

作者：灵茶山艾府
链接：https://leetcode.cn/problems/selling-pieces-of-wood/solutions/1611240/by-endlesscheng-mrmd/
来源：力扣（LeetCode）
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。

```C++
class Solution {
public:
    long long sellingWood(int m, int n, vector<vector<int>> &prices) {
        vector<vector<long long>> f(m + 1, vector<long long>(n + 1));
        for (auto &p : prices) {
            f[p[0]][p[1]] = p[2];
        }
        for (int i = 1; i <= m; i++) {
            for (int j = 1; j <= n; j++) {
                for (int k = 1; k <= j / 2; k++) f[i][j] = max(f[i][j], f[i][k] + f[i][j - k]); // 垂直切割
                for (int k = 1; k <= i / 2; k++) f[i][j] = max(f[i][j], f[k][j] + f[i - k][j]); // 水平切割
            }
        }
        return f[m][n];
    }
};

// 作者：灵茶山艾府
// 链接：https://leetcode.cn/problems/selling-pieces-of-wood/solutions/1611240/by-endlesscheng-mrmd/
// 来源：力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```
