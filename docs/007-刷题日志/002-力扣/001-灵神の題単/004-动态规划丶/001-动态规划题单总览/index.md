# 分享丨【题单】动态规划（入门/背包/状态机/划分/区间/状压/数位/树形/数据结构优化）

> By <span style="color:rgb(255, 161, 22)">灵茶山艾府</span>
>
> 最近编辑于 2024-07-01

## HX注

> 恕我直言, 这个价值三千七百万!
>
> [前缀和优化 DP【力扣周赛 410】](https://www.bilibili.com/video/BV1Cf421v7Ky/) 43:28 开始, 聊一聊怎么刷题, 其中提到了dp (前面有手把手指导记忆化1:1翻译成dp的步骤)
>
> <span style="color:gold">学习dp的重点, 就是锻炼`寻找子问题`的能力, 剩下的都是代码能力上的东西!</span>

## 前言

掌握动态规划（DP）是没有捷径的，咱们唯一能做的，就是投入时间猛猛刷题。好比学数学，只看书看视频而不做习题，是不能说学会的。

我能做的，是帮你节省找题的时间，并把这些题分类整理好。有着相同套路的题，一起做效率会更高，也更能领悟到 DP 的精髓。所以推荐按照专题刷。

题目已按照难度分排序（右侧数字为难度分）。如果遇到难度很大，题解都看不懂的题目，建议直接跳过，二刷的时候再来尝试。

## 一、入门 DP

![动态规划算法题DP题单动态规划题单入门动态规划题目动态规划新手教程力扣DP力扣动态规划leetcode动态规划leetcode dp 灵茶山艾府 灵神 灵神题单](https://pic.leetcode.cn/1710769845-JRnIfA-dp-2.jpg)

**记忆化搜索**是新手村神器（甚至可以用到游戏后期），推荐先看 [动态规划入门：从记忆化搜索到递推](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1Xj411K7oF/)。

但记忆化搜索并不是万能的，某些题目只有写成递推，才能结合数据结构等来优化时间复杂度，多数题目还可以优化空间复杂度。所以尽量在写完记忆化搜索后，把递推的代码也写一下。熟练之后直接写递推也可以。

### §1.1 爬楼梯

- [x] [70. 爬楼梯](https://leetcode.cn/problems/climbing-stairs/)（[题解](https://leetcode.cn/problems/climbing-stairs/solution/jiao-ni-yi-bu-bu-si-kao-dong-tai-gui-hua-7zm1/)）
- [x] [746. 使用最小花费爬楼梯](https://leetcode.cn/problems/min-cost-climbing-stairs/)（[题解](https://leetcode.cn/problems/min-cost-climbing-stairs/solution/jiao-ni-yi-bu-bu-si-kao-dong-tai-gui-hua-j99e/)）
- [x] [377. 组合总和 Ⅳ](https://leetcode.cn/problems/combination-sum-iv/) 本质是爬楼梯，相当于每次往上爬 $\textit{nums}[i]$ 步
- [x] [2466. 统计构造好字符串的方案数](https://leetcode.cn/problems/count-ways-to-build-good-strings/) 1694
- [x] [2266. 统计打字方案数](https://leetcode.cn/problems/count-number-of-texts/) 1857 (线性dp 乘法原理)
-   [2533\. 好二进制字符串的数量](https://leetcode.cn/problems/number-of-good-binary-strings/)（会员题）

### §1.2 打家劫舍

- [x] [198. 打家劫舍](https://leetcode.cn/problems/house-robber/) ~1500
- [x] [740. 删除并获得点数](https://leetcode.cn/problems/delete-and-earn/) ~1600 (左不能, 右不能, 就是可转化为打家劫舍~~(值域?)~~)
- [x] [2320. 统计放置房子的方式数](https://leetcode.cn/problems/count-number-of-ways-to-place-houses/) 1608
- [x] [213. 打家劫舍 II](https://leetcode.cn/problems/house-robber-ii/) ~1650
- [x] [3186. 施咒的最大总伤害](https://leetcode.cn/problems/maximum-total-damage-with-spell-casting/) ~1850 (对cpp不友好(卡常?!); 值域打家劫舍)

通用套路: $f[i] = \max(f[i - 1], f[i - 2] + a[i])$ # 不能选择相邻(a[i - 1], a[i + 1]), 则分: 不选, 选 两种情况

对于选择了a[i]就不能选择`a[i] - 2 ，a[i] - 1 ，a[i] + 1 或者 a[i] + 2`的同理: $f[i] = max(f[i - 1], f[j - 1] + a[i])$ # 不选, 选(其中`a[j - 1] < arr[i] - 2`)

对于值域做法, 可以先使用哈希表统计, 映射到数组, 再通过索引做记忆化/dp的参数

### §1.3 最大子数组和 (最大子段和)

有两种做法:

1. 定义状态 $f[i]$ 表示以 $α[i]$ 结尾的最大子数组和，不和 $i$ 左边拼起来就是 $f[i]=a[i]$, 和 $i$ 左边拼起来就是 $f[i] = f[i － 1] + a[i]$, 取最大值就得到了状态转移方程 $f[j] = \max(f[i - 1],0) + a[i]$, 答案为 $\max(f)$。这个做法也叫做 $Kadane$ 算法。

2.  用前缀和解决，具体见 [我的题解](https://leetcode.cn/problems/maximum-subarray/solution/qian-zhui-he-zuo-fa-ben-zhi-shi-mai-mai-abu71/)。

- [x] [53. 最大子数组和](https://leetcode.cn/problems/maximum-subarray/) ~1400
- [x] [2606. 找到最大开销的子字符串](https://leetcode.cn/problems/find-the-substring-with-maximum-cost/) 1422
- [x] [1749. 任意子数组和的绝对值的最大值](https://leetcode.cn/problems/maximum-absolute-sum-of-any-subarray/) 1542
- [学习] [1191. K 次串联后最大子数组之和](../002-最大子数组和/001-K次串联后最大子数组之和/index.md) 1748
- [学习] [918. 环形子数组的最大和](../002-最大子数组和/002-环形子数组的最大和/index.md) 1777
- [学习] [2321. 拼接数组的最大分数](../002-最大子数组和/003-拼接数组的最大分数/index.md) 1791

**思维扩展**：

- [x] [152. 乘积最大子数组](https://leetcode.cn/problems/maximum-product-subarray/) (提示: 前一个数是正/负两种状态)

## 二、网格图 DP

对于一些二维 DP（例如背包、最长公共子序列），如果把 DP 矩阵画出来，其实状态转移可以视作**在网格图上的移动**。所以在学习相对更抽象的二维 DP 之前，做一些形象的网格图 DP 会让后续的学习更轻松（比如 0-1 背包的空间优化写法为什么要倒序遍历）。

[讲解](https://leetcode.cn/problems/li-wu-de-zui-da-jie-zhi-lcof/solution/jiao-ni-yi-bu-bu-si-kao-dpcong-hui-su-da-epvl/)

### §2.1 基础

- [x] [LCR 166. 珠宝的最高价值](https://leetcode.cn/problems/li-wu-de-zui-da-jie-zhi-lcof/)
- [x] [62. 不同路径](https://leetcode.cn/problems/unique-paths/)
- [x] [63. 不同路径 II](https://leetcode.cn/problems/unique-paths-ii/)
- [x] [64. 最小路径和](https://leetcode.cn/problems/minimum-path-sum/)
- [x] [120. 三角形最小路径和](https://leetcode.cn/problems/triangle/)
- [x] [931. 下降路径最小和](https://leetcode.cn/problems/minimum-falling-path-sum/) 1573
- [x] [2684. 矩阵中移动的最大次数](https://leetcode.cn/problems/maximum-number-of-moves-in-a-grid/) 1626
- [x] [2304. 网格中的最小路径代价](https://leetcode.cn/problems/minimum-path-cost-in-a-grid/) 1658
- [x] [1289. 下降路径最小和 II](https://leetcode.cn/problems/minimum-falling-path-sum-ii/) 1697

### §2.2 进阶

- [x] [1594. 矩阵的最大非负积](https://leetcode.cn/problems/maximum-non-negative-product-in-a-matrix/) 1807 要求最后取模!, 思路同: [152. 乘积最大子数组](https://leetcode.cn/problems/maximum-product-subarray/)
- [x] [1301. 最大得分的路径数目](https://leetcode.cn/problems/number-of-paths-with-max-score/) 1853
- [x] [2435. 矩阵中和能被 K 整除的路径](https://leetcode.cn/problems/paths-in-matrix-whose-sum-is-divisible-by-k/) 1952 (套路: 把路径和模 $k$ 的结果当成一个扩展维度)
- [x] [174. 地下城游戏](../003-网格图DP/001-地下城游戏/index.md) (正难则反dp)
- [x] [329. 矩阵中的最长递增路径](https://leetcode.cn/problems/longest-increasing-path-in-a-matrix/) (记忆化 / 预处理 + 拓扑排序)
- [x] [2328. 网格图中递增路径的数目](https://leetcode.cn/problems/number-of-increasing-paths-in-a-grid/) 2001 (同 329)
- [学习] [2267. 检查是否有合法括号字符串路径](https://leetcode.cn/problems/check-if-there-is-a-valid-parentheses-string-path/) 2085 (括号匹配固定套路: 任何时刻左括号数量 >= 右括号数量, 可定义 $c =$ 左括号 - 右括号, 或者说 左括号: ++c, 右括号: --c; c >= 0 (任何时刻), c == 1, (最后一个, 并且最后为 右括号(不包括它)); 剪枝: 必须保证: 左右括号数 % 2 == 0)
- [学习] [1937. 扣分后的最大得分](https://leetcode.cn/problems/maximum-number-of-points-with-cost/) 2106 (写出 dp 式子, 拆开绝对值, 分类讨论(化简式子); 奇妙的for方式!)
- [学习] [1463. 摘樱桃 II](https://leetcode.cn/problems/cherry-pickup-ii/)
- [学习] [741. 摘樱桃](https://leetcode.cn/problems/cherry-pickup/)
-   [2510\. 检查是否有路径经过相同数量的 0 和 1](https://leetcode.cn/problems/check-if-there-is-a-path-with-equal-number-of-0s-and-1s/)（会员题）

## 三、背包

讲解：[0-1 背包 完全背包](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV16Y411v7Y6/)

### §3.1 0-1 背包

每个物品只能选一次。

- [x] [2915. 和为目标值的最长子序列的长度](https://leetcode.cn/problems/length-of-the-longest-subsequence-that-sums-to-target/) 1659
- [x] [416. 分割等和子集](https://leetcode.cn/problems/partition-equal-subset-sum/)
- [x] [494. 目标和](https://leetcode.cn/problems/target-sum/) (有点水平这题qwq)
- [x] [2787. 将一个数字表示成幂的和的方案数](https://leetcode.cn/problems/ways-to-express-an-integer-as-sum-of-powers/) 1818
- [x] [3180. 执行操作可获得的最大总奖励 I](https://leetcode.cn/problems/maximum-total-reward-using-operations-i/) 1849 (难难难~ 不会... 价值为体积类背包?)
- [x] [474. 一和零](https://leetcode.cn/problems/ones-and-zeroes/)（二维）(蒙的状态转移方程)
- [x] [学习] [1049. 最后一块石头的重量 II](https://leetcode.cn/problems/last-stone-weight-ii/) 2092 (脑筋急转弯: 把所有的 +- 提出来: 就是 ~~价值为体积类背包~~ 求 下取整`sum/2`容量的最大价值, 然后 sum - 2 * f[])
- [x] [1774. 最接近目标价格的甜点成本](https://leetcode.cn/problems/closest-dessert-cost/) (记忆化更好, 更少无用状态)
- [x] [学习] [879. 盈利计划](https://leetcode.cn/problems/profitable-schemes/) 2204 (初始化问题, 状态定义与边界)
-   [3082\. 求出所有子序列的能量和](https://leetcode.cn/problems/find-the-sum-of-the-power-of-all-subsequences/) 2242
-   [956\. 最高的广告牌](https://leetcode.cn/problems/tallest-billboard/) 2381 (定义状态为高度差值, 对于一方的高度可以 +1/-1/0, 注意身份切换)
-   [2518\. 好分区的数目](https://leetcode.cn/problems/number-of-great-partitions/) 2415
-   [2742\. 给墙壁刷油漆](https://leetcode.cn/problems/painting-the-walls/) 2425
-   [LCP 47. 入场安检](https://leetcode.cn/problems/oPs9Bm/)
-   [2291\. 最大股票收益](https://leetcode.cn/problems/maximum-profit-from-trading-stocks/)（会员题）
-   [2431\. 最大限度地提高购买水果的口味](https://leetcode.cn/problems/maximize-total-tastiness-of-purchased-fruits/)（会员题）

### §3.2 完全背包

物品可以重复选，无个数限制。

- [x] [322. 零钱兑换](https://leetcode.cn/problems/coin-change/)
- [x] [518. 零钱兑换 II](https://leetcode.cn/problems/coin-change-ii/)
- [x] [279. 完全平方数](https://leetcode.cn/problems/perfect-squares/)
- [x] [1449. 数位成本和为目标值的最大数字](https://leetcode.cn/problems/form-largest-integer-with-digits-that-add-up-to-target/) 1927 (小心字符串的max是单纯的字典序的)

### §3.3 多重背包

物品可以重复选，有个数限制。

> 注：力扣上只有求方案数的题目。

- [x] [2585. 获得分数的方法数](https://leetcode.cn/problems/number-of-ways-to-earn-points/) 1910 (注意, 同一类题目是完全相同的, 因此 枚举题目数量得在最内层for)
-   [2902\. 和带限制的子多重集合的数目](https://leetcode.cn/problems/count-of-sub-multisets-with-bounded-sum/) 2759

### §3.4 分组背包

同一组内的物品至多/恰好选一个。

- [x] [1155. 掷骰子等于目标和的方法数](https://leetcode.cn/problems/number-of-dice-rolls-with-target-sum/) 1654 (前缀和才可以滚动压缩)
- [x] [学习] [1981. 最小化目标值与所选元素的差](https://leetcode.cn/problems/minimize-the-difference-between-target-and-chosen-elements/) 2010 (没有绝对值我可能还会... dp二进制优化?!)
-   [2218\. 从栈中取出 K 个硬币的最大面值和](https://leetcode.cn/problems/maximum-value-of-k-coins-from-piles/) 2158

## 四、经典线性 DP

### §4.1 最长公共子序列（LCS）

讲解：[最长公共子序列 编辑距离](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1TM4y1o7ug/)

一般定义 $f[i][j]$ 表示对 $(s[:i], t[:j])$ 的求解结果。

- [x] [1143. 最长公共子序列](https://leetcode.cn/problems/longest-common-subsequence/)
- [x] [583. 两个字符串的删除操作](https://leetcode.cn/problems/delete-operation-for-two-strings/)
- [x] [712. 两个字符串的最小 ASCII 删除和](https://leetcode.cn/problems/minimum-ascii-delete-sum-for-two-strings/)
- [x] [72. 编辑距离](https://leetcode.cn/problems/edit-distance/)
- [x] [学习] [97. 交错字符串](https://leetcode.cn/problems/interleaving-string/) (定义 s1[0:i] s2[0:j] 为 f[i][j] 可以凑出 s3[0: i + j])
- [x] [学习] [115. 不同的子序列](https://leetcode.cn/problems/distinct-subsequences/) (状态定义正确, 但是初始化没有, 状态转移不对!)
- [x] [1035. 不相交的线](https://leetcode.cn/problems/uncrossed-lines/) 1806
- [x] [1458. 两个子序列的最大点积](https://leetcode.cn/problems/max-dot-product-of-two-subsequences/) 1824 (学到了一种: 不能用0来规避越界的初始化, 得像官解一样，先下手为强`int t = f[i][j] = nums1[i] * nums2[j];`)
- [x] [1092. 最短公共超序列](https://leetcode.cn/problems/shortest-common-supersequence/) 1977 (用string dp 再怎么优化也是超内存(过了但不是正解), 正解是改为dp长度, 然后双指针构造字符串(这个并不是很会...))
- [x] [1639. 通过给定词典构造目标字符串的方案数](https://leetcode.cn/problems/number-of-ways-to-form-a-target-string-given-a-dictionary/) 2082 (记忆化, 但是 $O(n^3)$, 预处理同一列字符个数, 可以有 $O(n^2)$ )
- [x] [44. 通配符匹配](https://leetcode.cn/problems/wildcard-matching/) (`@cache`)
- [x] [10. 正则表达式匹配](https://leetcode.cn/problems/regular-expression-matching/) (`@cache`)

### [](#§42-最长递增子序列（lis）)§4.2 最长递增子序列（LIS）

讲解：[最长递增子序列](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1ub411Q7sB/)

做法有很多:

1.  枚举选哪个（见讲解）。
2.  贪心+二分（见讲解）。
3.  计算 $a$ 和把 $a$ 排序后的数组 $\textit{sortedA}$ 的最长公共子序列。
4.  数据结构优化（见 2407 题）。

- [x] [300. 最长递增子序列](https://leetcode.cn/problems/longest-increasing-subsequence/)
- [x] [673. 最长递增子序列的个数](https://leetcode.cn/problems/number-of-longest-increasing-subsequence/)
- [x] [2826. 将三个组排序](https://leetcode.cn/problems/sorting-three-groups/) 1721 (也可以状态机dp)
- [x] [1671. 得到山形数组的最少删除次数](https://leetcode.cn/problems/minimum-number-of-removals-to-make-mountain-array/) 1913 (前后缀分解)
- [x] [1964. 找出到每个位置为止最长的有效障碍赛跑路线](https://leetcode.cn/problems/find-the-longest-valid-obstacle-course-at-each-position/) 1933 (裸题, 但是需要二分log优化~)
- [x] [2111. 使数组 K 递增的最少操作次数](https://leetcode.cn/problems/minimum-operations-to-make-the-array-k-increasing/) 1941 (转换为互不相交子序列, (注意可能有子序列长度不一致!) 二分log优化dp~)
- [x] [1626. 无矛盾的最佳球队](https://leetcode.cn/problems/best-team-with-no-conflicts/) 2027 (按年龄排序转换~)
- [x] [354. 俄罗斯套娃信封问题](https://leetcode.cn/problems/russian-doll-envelopes/)（二维 LIS）(把握好排序) [如何排序 & 如何扩展到高维度情况](https://leetcode.cn/problems/russian-doll-envelopes/solutions/634104/ru-he-pai-xu-ru-he-kuo-zhan-dao-gao-wei-wr4qc)
- [x] [1691. 堆叠长方体的最大高度](https://leetcode.cn/problems/maximum-height-by-stacking-cuboids/) 2172 (把握好排序)
-   [960\. 删列造序 $i$](https://leetcode.cn/problems/delete-columns-to-make-sorted-i/) 2247
-   [2407\. 最长递增子序列 II](https://leetcode.cn/problems/longest-increasing-subsequence-ii/) 2280 (值域线段树优化dp...)
-   [1187\. 使数组严格递增](https://leetcode.cn/problems/make-array-strictly-increasing/) 2316
-   [1713\. 得到子序列的最少操作次数](https://leetcode.cn/problems/minimum-operations-to-make-a-subsequence/) 2351

**思维扩展**：

- [x] [368. 最大整除子集](https://leetcode.cn/problems/largest-divisible-subset/) (有点妙~)

**思考题**：

给定整数 $k$，构造一个数组 $a$，使得 $a$ 恰好有 $k$ 个最长递增子序列。

[解答（评论）](https://leetcode.cn/problems/number-of-longest-increasing-subsequence/description/comments/2218054)

## 五、状态机 DP

讲解：[状态机 DP](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1ho4y1W7QK/)

一般定义 $f[i][j]$ 表示前缀 $a[:i]$ 在状态 $j$ 下的最优值。一般 $j$ 都很小。代表题目是「买卖股票」系列。

注: 某些题目做法不止一种，除了状态机 DP 以外，也有前后缀分解的做法。

-   [121\. 买卖股票的最佳时机](https://leetcode.cn/problems/best-time-to-buy-and-sell-stock/)
-   [122\. 买卖股票的最佳时机 II](https://leetcode.cn/problems/best-time-to-buy-and-sell-stock-ii/)
-   [123\. 买卖股票的最佳时机 $i$](https://leetcode.cn/problems/best-time-to-buy-and-sell-stock-$i$/)
-   [188\. 买卖股票的最佳时机 IV](https://leetcode.cn/problems/best-time-to-buy-and-sell-stock-iv/)
-   [309\. 买卖股票的最佳时机含冷冻期](https://leetcode.cn/problems/best-time-to-buy-and-sell-stock-with-cooldown/)
-   [714\. 买卖股票的最佳时机含手续费](https://leetcode.cn/problems/best-time-to-buy-and-sell-stock-with-transaction-fee/)
- [x] [1493. 删掉一个元素以后全为 1 的最长子数组](https://leetcode.cn/problems/longest-subarray-of-1s-after-deleting-one-element/) 1423
- [x] [1395. 统计作战单位数](https://leetcode.cn/problems/count-number-of-teams/)
- [x] [2745. 构造最长的新字符串](https://leetcode.cn/problems/construct-the-longest-new-string/) 1607
- [x] [2222. 选择建筑的方案数](https://leetcode.cn/problems/number-of-ways-to-select-buildings/) 1657
- [x] [376. 摆动序列](https://leetcode.cn/problems/wiggle-subsequence/) 做到 $\mathcal{O}(n)$ 时间
- [x] [1567. 乘积为正数的最长子数组长度](https://leetcode.cn/problems/maximum-length-of-subarray-with-positive-product/) 1710
- [x] [2708. 一个小组的最大实力值](https://leetcode.cn/problems/maximum-strength-of-a-group/) 做到 $\mathcal{O}(n)$ 时间
- [x] [2826. 将三个组排序](https://leetcode.cn/problems/sorting-three-groups/) 1721
- [x] [2786. 访问数组中的位置使分数最大](https://leetcode.cn/problems/visit-array-positions-to-maximize-score/) 1733
- [x] [1262. 可被三整除的最大和](https://leetcode.cn/problems/greatest-sum-divisible-by-three/) 1762
- [x] [1363. 形成三的最大倍数](https://leetcode.cn/problems/largest-multiple-of-three/) (dp简单, 复原出字符串难qwq)
- [x] [1911. 最大子序列交替和](https://leetcode.cn/problems/maximum-alternating-subsequence-sum/) 1786
- [x] [2771. 构造最长非递减子数组](https://leetcode.cn/problems/longest-non-decreasing-subarray-from-two-arrays/) 1792
- [x] [1186. 删除一次得到子数组最大和](https://leetcode.cn/problems/maximum-subarray-sum-with-one-deletion/) 1799
- [x] [1594. 矩阵的最大非负积](https://leetcode.cn/problems/maximum-non-negative-product-in-a-matrix/) 1807
- [x] [3196. 最大化子数组的总成本](https://leetcode.cn/problems/maximize-total-cost-of-alternating-subarrays/) ~1850 也有划分型 DP 做法
- [x] [935. 骑士拨号器](https://leetcode.cn/problems/knight-dialer/) (同类状态可以乘算!)
- [x] [1537. 最大得分](https://leetcode.cn/problems/get-the-maximum-score/) 1961 (转换为图dp / 双指针)
-   [2919. 使数组变美的最小增量运算数](https://leetcode.cn/problems/minimum-increment-operations-to-make-array-beautiful/) 2031 (还没有搞懂)
- [x] [801. 使序列递增的最小交换次数](https://leetcode.cn/problems/minimum-swaps-to-make-sequences-increasing/) 2066 (定义换/不换)
-   [1955\. 统计特殊子序列的数目](https://leetcode.cn/problems/count-number-of-special-subsequences/) 2125
-   [3068\. 最大节点价值之和](https://leetcode.cn/problems/find-the-maximum-sum-of-node-values/) 2268
-   [LCP 19. 秋叶收藏集](https://leetcode.cn/problems/UlBDOe/)
-   [276\. 栅栏涂色](https://leetcode.cn/problems/paint-fence/)（会员题）
-   [1746\. 经过一次操作后的最大子数组和](https://leetcode.cn/problems/maximum-subarray-sum-after-one-operation/)（会员题）
-   [2036\. 最大交替子数组和](https://leetcode.cn/problems/maximum-alternating-subarray-sum/)（会员题）
-   [2361\. 乘坐火车路线的最少费用](https://leetcode.cn/problems/minimum-costs-using-the-train-line/)（会员题）

## 六、划分型 DP

### §6.1 判定能否划分

一般定义 $f[i][j]$ 表示长为 $i$ 的前缀 $a[:i]$ 能否划分。

枚举最后一个子数组的左端点 $L$，从 $f[L]$ 转移到 $f[i][j]$，并考虑 $a[L:j]$ 是否满足要求。

- [x] [2369. 检查数组是否存在有效划分](https://leetcode.cn/problems/check-if-there-is-a-valid-partition-for-the-array/) 1780
- [x] [139. 单词拆分](https://leetcode.cn/problems/word-break/)

### §6.2 计算划分最优值

计算最少（最多）可以划分出多少段、最优划分得分等。

一般定义 $f[i][j]$ 表示长为 $i$ 的前缀 $a[:i]$ 在题目约束下，分割出的最少（最多）子数组个数（或者定义成分割方案数）。

枚举最后一个子数组的左端点 $L$，从 $f[L]$ 转移到 $f[i][j]$，并考虑 $a[L:j]$ 对最优解的影响。

- [x] [132. 分割回文串 II](https://leetcode.cn/problems/palindrome-partitioning-ii/) (有预处理回文子串的dp)
- [x] [2707. 字符串中的额外字符](https://leetcode.cn/problems/extra-characters-in-a-string/) 1736
- [x] [3196. 最大化子数组的总成本](https://leetcode.cn/problems/maximize-total-cost-of-alternating-subarrays/) ~1850 也有状态机 DP 做法
- [x] [2767. 将字符串分割为最少的美丽子字符串](https://leetcode.cn/problems/partition-string-into-minimum-beautiful-substrings/) 1865
- [x] [91. 解码方法](https://leetcode.cn/problems/decode-ways/)
- [x] [639. 解码方法 II](https://leetcode.cn/problems/decode-ways-ii/)
- [x] [LCR 165. 解密数字](https://leetcode.cn/problems/ba-shu-zi-fan-yi-cheng-zi-fu-chuan-lcof/)
- [x] [1416. 恢复数组](https://leetcode.cn/problems/restore-the-array/) 1920
- [x] [2472. 不重叠回文子字符串的最大数目](https://leetcode.cn/problems/maximum-number-of-non-overlapping-palindrome-substrings/) 2013 (中心扩展 + DP (我暴力判断回文的~, 两千个'a'那个样例无法通过))
- [x] [1105. 填充书架](https://leetcode.cn/problems/filling-bookcase-shelves/) 2014 (有难度的处理和状态转移思考, 从递归开始好思考~)
- [x] [2547. 拆分数组的最小代价](https://leetcode.cn/problems/minimum-cost-to-split-an-array/) 2020 (我甚至使用滑动窗口预处理,题解都没有需要..可以线段树优化qwq..)
-   [2430\. 对字母串可执行的最大删除数](https://leetcode.cn/problems/maximum-deletions-on-a-string/) 2102
-   [2463\. 最小移动总距离](https://leetcode.cn/problems/minimum-total-distance-traveled/) 2454
-   [2977\. 转换字符串的最小成本 II](https://leetcode.cn/problems/minimum-cost-to-convert-string-ii/) 2696
-   [2052\. 将句子分隔成行的最低成本](https://leetcode.cn/problems/minimum-cost-to-separate-sentence-into-rows/)（会员题）
-   [2464\. 有效分割中的最少子数组数目](https://leetcode.cn/problems/minimum-subarrays-in-a-valid-split/)（会员题）

### [](#§63-约束划分个数)§6.3 约束划分个数

将数组分成 (恰好/至多) $k$ 个连续子数组，计算与这些子数组有关的最优值。

一般定义 $f[i][j]$ 表示将长为 $j$ 的前缀 $a[:j]$ 分成 $i$ 个连续子数组所得到的最优解。

枚举最后一个子数组的左端点 $L$，从 $f[i-1][L]$ 转移到 $f[i][j]$，并考虑 $a[L:j]$ 对最优解的影响。

- [x] [410. 分割数组的最大值](https://leetcode.cn/problems/split-array-largest-sum/) (qwq)
- [x] [1043. 分隔数组以得到最大和](https://leetcode.cn/problems/partition-array-for-maximum-sum/) 1916
- [x] [1745. 分割回文串 IV](https://leetcode.cn/problems/palindrome-partitioning-iv/) 1925
- [x] [813. 最大平均值和的分组](https://leetcode.cn/problems/largest-sum-of-averages/) 1937
- [x] [1278. 分割回文串 iii](https://leetcode.cn/problems/palindrome-partitioning-iii/) 1979
- [x] [1335. 工作计划的最低难度](https://leetcode.cn/problems/minimum-difficulty-of-a-job-schedule/) 2035
-   [1473\. 粉刷房子 iii](https://leetcode.cn/problems/paint-house-iii/) 2056 (有点难度qwq..)
- [x] [1478. 安排邮筒](https://leetcode.cn/problems/allocate-mailboxes/) 2190
-   [1959\. K 次调整数组大小浪费的最小总空间](https://leetcode.cn/problems/minimum-total-space-wasted-with-k-resizing-operations/) 2310 \*转换
-   [2478\. 完美分割的方案数](https://leetcode.cn/problems/number-of-beautiful-partitions/) 2344
-   [3077\. K 个不相交子数组的最大能量值](https://leetcode.cn/problems/maximum-strength-of-k-disjoint-subarrays/) 2557
-   [2911\. 得到 K 个半回文串的最少修改次数](https://leetcode.cn/problems/minimum-changes-to-make-k-semi-palindromes/) 2608
-   [3117\. 划分数组得到最小的值之和](https://leetcode.cn/problems/minimum-sum-of-values-by-dividing-array/) 2735

### §6.4 不相交区间

- [x] [2830. 销售利润最大化](https://leetcode.cn/problems/maximize-the-profit-as-the-salesman/) 1851
- [x] [2008. 出租车的最大盈利](https://leetcode.cn/problems/maximum-earnings-from-taxi/) 1872
-   [1235. 规划兼职工作](https://leetcode.cn/problems/maximum-profit-in-job-scheduling/) 2023 也可以用堆 (二分优化)
-   [1751\. 最多可以参加的会议数目 II](https://leetcode.cn/problems/maximum-number-of-events-that-can-be-attended-ii/) 2041

## 七、其他线性 DP

### §7.1 一维

发生在前缀/后缀之间的转移，例如从 $f[i-1]$ 转移到 $f[i][j]$，或者从 $f[j]$ 转移到 $f[i][j]$。

- [x] [2944. 购买水果需要的最少金币数](https://leetcode.cn/problems/minimum-number-of-coins-for-fruits/) 1709
- [x] [2140. 解决智力问题](https://leetcode.cn/problems/solving-questions-with-brainpower/) 1709
- [x] [983. 最低票价](https://leetcode.cn/problems/minimum-cost-for-tickets/) 1786
- [x] [2901. 最长相邻不相等子序列 II](https://leetcode.cn/problems/longest-unequal-adjacent-groups-subsequence-ii/) 1899 (拼结果比较妙! 我是暴力的, 0x3f好像是使用一个from来记录, 然后可以从最后一个找到它的上一个, 类似于父节点. 从而还原!)
- [x] [3144. 分割字符频率相等的最少子字符串](https://leetcode.cn/problems/minimum-substring-partition-of-equal-character-frequency/) 1917 (很妙的计算`平衡字符串`的方法)
- [x] [871. 最低加油次数](https://leetcode.cn/problems/minimum-number-of-refueling-stops/) 2074 (贪心堆...dp不会)
-   [2896\. 执行操作使两个字符串相等](https://leetcode.cn/problems/apply-operations-to-make-two-strings-equal/) 2172
-   [2167\. 移除所有载有违禁货物车厢所需的最少时间](https://leetcode.cn/problems/minimum-time-to-remove-all-cars-containing-illegal-goods/) 2219
-   [2188\. 完成比赛的最少时间](https://leetcode.cn/problems/minimum-time-to-finish-the-race/) 2315
-   [1259\. 不相交的握手](https://leetcode.cn/problems/handshakes-that-dont-cross/)（会员题）

### §7.2 特殊子序列

比较特殊的一类题型，递推比记忆化搜索好写。

- [x] [2501. 数组中最长的方波](https://leetcode.cn/problems/longest-square-streak-in-an-array/) 1480
- [x] [1218. 最长定差子序列](https://leetcode.cn/problems/longest-arithmetic-subsequence-of-given-difference/) 1597
- [x] [1027. 最长等差数列](https://leetcode.cn/problems/longest-arithmetic-subsequence/) 1759
- [x] [3202. 找出有效子序列的最大长度 II](https://leetcode.cn/problems/find-the-maximum-length-of-valid-subsequence-ii/) ~1850
- [x] [873. 最长的斐波那契子序列的长度](https://leetcode.cn/problems/length-of-longest-fibonacci-subsequence/) 1911 (我的纯哈希超时了, 还得像题解那样改为索引+vector的/不然就unordered_set暴力)
- [x] [446. 等差数列划分 II - 子序列](https://leetcode.cn/problems/arithmetic-slices-ii-subsequence/) (学习状态!: 定义i结尾, 公差为d的子序列个数为f[i][j])
-   [1048\. 最长字符串链](https://leetcode.cn/problems/longest-string-chain/)
-   [3098\. 求出所有子序列的能量和](https://leetcode.cn/problems/find-the-sum-of-subsequence-powers/) 2553

### §7.3 矩阵快速幂优化

部分题目由于数据范围小，也可以用线性 DP。

- [x] [70. 爬楼梯](https://leetcode.cn/problems/climbing-stairs/)
- [x] [509. 斐波那契数](https://leetcode.cn/problems/fibonacci-number/)
- [x] [1137. 第 N 个泰波那契数](https://leetcode.cn/problems/n-th-tribonacci-number/)
-   [1220\. 统计元音字母序列的数目](https://leetcode.cn/problems/count-vowels-permutation/)
-   [552\. 学生出勤记录 II](https://leetcode.cn/problems/student-attendance-record-ii/)
-   [790\. 多米诺和托米诺平铺](https://leetcode.cn/problems/domino-and-tromino-tiling/)
-   [2851\. 字符串转换](https://leetcode.cn/problems/string-transformation/) 2858
-   [2912\. 在网格上移动到目的地的方法数](https://leetcode.cn/problems/number-of-ways-to-reach-destination-in-the-grid/)（会员题）

### [](#§74-子矩形)§7.4 子矩形

-   [3148\. 矩阵中的最大得分](https://leetcode.cn/problems/maximum-difference-score-in-a-grid/) 1820
-   [221\. 最大正方形](https://leetcode.cn/problems/maximal-square/)
-   [1277\. 统计全为 1 的正方形子矩阵](https://leetcode.cn/problems/count-square-submatrices-with-all-ones/)
-   [2088\. 统计农场中肥沃金字塔的数目](https://leetcode.cn/problems/count-fertile-pyramids-in-a-land/) 2105
-   [3197\. 包含所有 1 的最小矩形面积 II](https://leetcode.cn/problems/find-the-minimum-area-to-cover-all-ones-ii/) $\mathcal{O}(mn)$ 做法

### [](#§75-多维)§7.5 多维

-   [2400\. 恰好移动 k 步到达某一位置的方法数目](https://leetcode.cn/problems/number-of-ways-to-reach-a-position-after-exactly-k-steps/) 1751
-   [2370\. 最长理想子序列](https://leetcode.cn/problems/longest-ideal-subsequence/) 1835
-   [3176\. 求出最长好子序列 I](https://leetcode.cn/problems/find-the-maximum-length-of-a-good-subsequence-i/) 1849
-   [1269\. 停在原地的方案数](https://leetcode.cn/problems/number-of-ways-to-stay-in-the-same-place-after-some-steps/) 1854
-   [3122\. 使矩阵满足条件的最少操作次数](https://leetcode.cn/problems/minimum-number-of-operations-to-satisfy-conditions/) 1905
-   [576\. 出界的路径数](https://leetcode.cn/problems/out-of-boundary-paths/)
-   [403\. 青蛙过河](https://leetcode.cn/problems/frog-jump/)
-   [1223\. 掷骰子模拟](https://leetcode.cn/problems/dice-roll-simulation/) 2008
-   [1320\. 二指输入的的最小距离](https://leetcode.cn/problems/minimum-distance-to-type-a-word-using-two-fingers/) 2028
-   [1575\. 统计所有可行路径](https://leetcode.cn/problems/count-all-possible-routes/) 2055
-   [3154\. 到达第 K 级台阶的方案数](https://leetcode.cn/problems/find-number-of-ways-to-reach-the-k-th-stair/) 2071
-   [2318\. 不同骰子序列的数目](https://leetcode.cn/problems/number-of-distinct-roll-sequences/) 2090
-   [2209\. 用地毯覆盖后的最少白色砖块](https://leetcode.cn/problems/minimum-white-tiles-after-covering-with-carpets/) 2106
-   [1444\. 切披萨的方案数](https://leetcode.cn/problems/number-of-ways-of-cutting-a-pizza/) 2127
-   [1420\. 生成数组](https://leetcode.cn/problems/build-array-where-you-can-find-the-maximum-exactly-k-comparisons/) 2176
-   [3193\. 统计逆序对的数目](https://leetcode.cn/problems/count-the-number-of-inversions/) ~2250
-   [629\. K 个逆序对数组](https://leetcode.cn/problems/k-inverse-pairs-array/) ~2300
-   [1866\. 恰有 K 根木棍可以看到的排列数目](https://leetcode.cn/problems/number-of-ways-to-rearrange-sticks-with-k-sticks-visible/) 2333
-   [2312\. 卖木头块](https://leetcode.cn/problems/selling-pieces-of-wood/) 2363
-   [3177\. 求出最长好子序列 II](https://leetcode.cn/problems/find-the-maximum-length-of-a-good-subsequence-ii/) 2365
-   [887\. 鸡蛋掉落](https://leetcode.cn/problems/super-egg-drop/) 2377
-   [1884\. 鸡蛋掉落-两枚鸡蛋](https://leetcode.cn/problems/egg-drop-with-2-eggs-and-n-floors/)
-   [1388\. 3n 块披萨](https://leetcode.cn/problems/pizza-with-3n-slices/) 2410
-   [1900\. 最佳运动员的比拼回合](https://leetcode.cn/problems/the-earliest-and-latest-rounds-where-players-compete/) 2455
-   [1883\. 准时抵达会议现场的最小跳过休息次数](https://leetcode.cn/problems/minimum-skips-to-arrive-at-meeting-on-time/) 2588 避免浮点运算的技巧
-   [LCP 57. 打地鼠](https://leetcode.cn/problems/ZbAuEH/)
-   [256\. 粉刷房子](https://leetcode.cn/problems/paint-house/)（会员题）
-   [265\. 粉刷房子 II](https://leetcode.cn/problems/paint-house-ii/)（会员题）
-   [568\. 最大休假天数](https://leetcode.cn/problems/maximum-vacation-days/)（会员题）
-   [1692\. 计算分配糖果的不同方式](https://leetcode.cn/problems/count-ways-to-distribute-candies/)（会员题）
-   [2143\. 在两个数组的区间中选取数字](https://leetcode.cn/problems/choose-numbers-from-two-arrays-in-range/)（会员题）

## 八、区间 DP

讲解：[区间 DP](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1Gs4y1E7EU/)

从数组的左右两端不断缩短，求解关于某段下标区间的最优值。

一般定义 $f[i][j]$ 表示下标区间 $[i,j]$ 的最优值。

### §8.1 最长回文子序列

- [x] [516. 最长回文子序列](https://leetcode.cn/problems/longest-palindromic-subsequence/)
- [超难] [730. 统计不同回文子序列](https://leetcode.cn/problems/count-different-palindromic-subsequences/)
- [x] [1312. 让字符串成为回文串的最少插入次数](https://leetcode.cn/problems/minimum-insertion-steps-to-make-a-string-palindrome/) 1787
-   [1771\. 由子序列构造的最长回文串的长度](https://leetcode.cn/problems/maximize-palindrome-length-from-subsequences/) 2182
-   [1682\. 最长回文子序列 II](https://leetcode.cn/problems/longest-palindromic-subsequence-ii/)（会员题）
-   [1216\. 验证回文串 $i$](https://leetcode.cn/problems/valid-palindrome-$i$/)（会员题）
-   [1246\. 删除回文子数组](https://leetcode.cn/problems/palindrome-removal/)（会员题）

### [](#§82-其他区间-dp)§8.2 其他区间 DP

- [x] [5\. 最长回文子串](https://leetcode.cn/problems/longest-palindromic-substring/) (马拉车, dp不是最优 => 不好想)
- [x] [3040\. 相同分数的最大操作数目 II](https://leetcode.cn/problems/maximum-number-of-operations-with-the-same-score-ii/) 1709
-   [375\. 猜数字大小 II](https://leetcode.cn/problems/guess-number-higher-or-lower-ii/)
-   [1130\. 叶值的最小代价生成树](https://leetcode.cn/problems/minimum-cost-tree-from-leaf-values/) 1919
-   [96\. 不同的二叉搜索树](https://leetcode.cn/problems/unique-binary-search-trees/)
-   [1770\. 执行乘法运算的最大分数](https://leetcode.cn/problems/maximum-score-from-performing-multiplication-operations/) 2068
-   [1547\. 切棍子的最小成本](https://leetcode.cn/problems/minimum-cost-to-cut-a-stick/) 2116
-   [1039\. 多边形三角剖分的最低得分](https://leetcode.cn/problems/minimum-score-triangulation-of-polygon/solution/shi-pin-jiao-ni-yi-bu-bu-si-kao-dong-tai-aty6/) 2130
-   [1000\. 合并石头的最低成本](https://leetcode.cn/problems/minimum-cost-to-merge-stones/) 2423
-   [2019\. 解出数学表达式的学生分数](https://leetcode.cn/problems/the-score-of-students-solving-math-expression/) 2584
-   [87\. 扰乱字符串](https://leetcode.cn/problems/scramble-string/)
-   [312\. 戳气球](https://leetcode.cn/problems/burst-balloons/)
-   [664\. 奇怪的打印机](https://leetcode.cn/problems/strange-printer/)
-   [546\. 移除盒子](https://leetcode.cn/problems/remove-boxes/) 同 CF1107E，可能是力扣上最难的 DP
-   [471\. 编码最短长度的字符串](https://leetcode.cn/problems/encode-string-with-shortest-length/)（会员题）
-   [3018\. 可处理的最大删除操作数 I](https://leetcode.cn/problems/maximum-number-of-removal-queries-that-can-be-processed-i/)（会员题）

## [](#九、状态压缩-dp（状压-dp）)九、状态压缩 DP（状压 DP）

### [](#§91-排列型-①-相邻无关)§9.1 排列型 ① 相邻无关

学习指南：

-   [教你一步步思考状压 DP：从记忆化搜索到递推](https://leetcode.cn/problems/beautiful-arrangement/solution/jiao-ni-yi-bu-bu-si-kao-zhuang-ya-dpcong-c6kd/)
-   [从集合论到位运算，常见位运算技巧分类总结](https://leetcode.cn/circle/discuss/CaOJ45/)

暴力做法是枚举所有排列，对每个排列计算和题目有关的值，时间复杂度（通常来说）是 O(n⋅n!)\\mathcal{O}(n\\cdot n!)O(n⋅n!)。可以解决 n≤10n\\le 10n≤10 的问题。

状压 DP 可以把时间复杂度（通常来说）优化至 O(n⋅2n)\\mathcal{O}(n\\cdot 2^n)O(n⋅2n)。可以解决 n≤20n\\le 20n≤20 的问题。

一般有两种定义方式：

1.  定义 f\[S\]f\[S\]f\[S\] 表示已经排列好的元素（下标）集合为 SSS 时，和题目有关的最优值。通过枚举当前位置要填的元素（下标）来转移。
2.  定义 f\[S\]f\[S\]f\[S\] 表示可以选的元素（下标）集合为 SSS 时，和题目有关的最优值。通过枚举当前位置要填的元素（下标）来转移。

> 注：部分题目由于爆搜+剪枝也能过，难度分仅供参考。

-   [526\. 优美的排列](https://leetcode.cn/problems/beautiful-arrangement/)
-   [1879\. 两个数组最小的异或值之和](https://leetcode.cn/problems/minimum-xor-sum-of-two-arrays/) 2145
-   [2850\. 将石头分散到网格图的最少移动次数](https://leetcode.cn/problems/minimum-moves-to-spread-stones-over-grid/)
-   [1947\. 最大兼容性评分和](https://leetcode.cn/problems/maximum-compatibility-score-sum/)
-   [1799\. N 次操作后的最大分数和](https://leetcode.cn/problems/maximize-score-after-n-operations/)
-   [2172\. 数组的最大与和](https://leetcode.cn/problems/maximum-and-sum-of-array/) 2392
-   [1066\. 校园自行车分配 II](https://leetcode.cn/problems/campus-bikes-ii/)（会员题）
-   [2992\. 自整除排列的数量](https://leetcode.cn/problems/number-of-self-divisible-permutations/)（会员题）
-   [2403\. 杀死所有怪物的最短时间](https://leetcode.cn/problems/minimum-time-to-kill-all-monsters/)（会员题）

### [](#§92-排列型-②-相邻相关)§9.2 排列型 ② 相邻相关

一般定义 f\[S\]\[i\]f\[S\]\[i\]f\[S\]\[i\] 表示未选（或者已选）的集合为 SSS，且上一个填的元素（下标）为 $i$ 时，和题目有关的最优值。通过枚举当前位置要填的元素（下标）来转移。

时间复杂度（通常来说）是 O(n2⋅2n)\\mathcal{O}(n^2\\cdot 2^n)O(n2⋅2n)。

[讲解：从全排列到状压 DP](https://leetcode.cn/problems/find-the-minimum-cost-array-permutation/solution/zhuang-ya-dpcong-ji-yi-hua-sou-suo-dao-d-s9t5/)

-   [996\. 正方形数组的数目](https://leetcode.cn/problems/number-of-squareful-arrays/) 1932
-   [2741\. 特别的排列](https://leetcode.cn/problems/special-permutations/) 2021
-   [1681\. 最小不兼容性](https://leetcode.cn/problems/minimum-incompatibility/) 2390
-   [3149\. 找出分数最低的排列](https://leetcode.cn/problems/find-the-minimum-cost-array-permutation/) 2642

### [](#§93-旅行商问题（tsp）)§9.3 旅行商问题（TSP）

本质上就是排列型 ②。

-   [943\. 最短超级串](https://leetcode.cn/problems/find-the-shortest-superstring/) 2186
-   [847\. 访问所有节点的最短路径](https://leetcode.cn/problems/shortest-path-visiting-all-nodes/) 2201
-   [LCP 13. 寻宝](https://leetcode.cn/problems/xun-bao/)
-   [2247\. K 条高速公路的最大旅行费用](https://leetcode.cn/problems/maximum-cost-of-trip-with-k-highways/)（会员题）

### [](#§94-枚举子集的子集)§9.4 枚举子集的子集

一般定义 f\[S\]f\[S\]f\[S\] 表示未选（或者已选）的集合为 SSS 时，和题目有关的最优值。通过枚举 SSS（或者 SSS 的补集 ∁US\\complement\_US∁U​S）的子集来转移。

时间复杂度（通常来说）是 O(3n)\\mathcal{O}(3^n)O(3n)，证明见 [题解](https://leetcode.cn/problems/parallel-courses-ii/solution/zi-ji-zhuang-ya-dpcong-ji-yi-hua-sou-suo-oxwd/)。

值得注意的是，枚举子集的子集还可以用「选或不选」来做，对于存在无效状态的情况，可以做到更优的时间复杂度。具体见 [1349 题解](https://leetcode.cn/problems/maximum-students-taking-exam/solution/jiao-ni-yi-bu-bu-si-kao-dong-tai-gui-hua-9y5k/) 最后的写法。

-   [2305\. 公平分发饼干](https://leetcode.cn/problems/fair-distribution-of-cookies/) 1887
-   [1986\. 完成任务的最少工作时间段](https://leetcode.cn/problems/minimum-number-of-work-sessions-to-finish-the-tasks/) 1995
-   [1494\. 并行课程 II](https://leetcode.cn/problems/parallel-courses-ii/) 2082
-   [1723\. 完成所有工作的最短时间](https://leetcode.cn/problems/find-minimum-time-to-finish-all-jobs/) 2284
-   [1655\. 分配重复整数](https://leetcode.cn/problems/distribute-repeating-integers/) 2307
-   [1349\. 参加考试的最大学生数](https://leetcode.cn/problems/maximum-students-taking-exam/) 2386
-   [1681\. 最小不兼容性](https://leetcode.cn/problems/minimum-incompatibility/) 2390 有 O(n2⋅2n)\\mathcal{O}(n^2\\cdot 2^n)O(n2⋅2n) 做法
-   [2572\. 无平方子集计数](https://leetcode.cn/problems/count-the-number-of-square-free-subsets/) 2420
-   [1994\. 好子集的数目](https://leetcode.cn/problems/the-number-of-good-subsets/) 2465
-   [LCP 04. 覆盖](https://leetcode.cn/problems/broken-board-dominoes/)
-   [LCP 53. 守护太空城](https://leetcode.cn/problems/EJvmW4/)
-   [465\. 最优账单平衡](https://leetcode.cn/problems/optimal-account-balancing/)（会员题）
-   [2152\. 穿过所有点的所需最少直线数量](https://leetcode.cn/problems/minimum-number-of-lines-to-cover-points/)（会员题）

### [](#§95-其他状压-dp)§9.5 其他状压 DP

-   [698\. 划分为k个相等的子集](https://leetcode.cn/problems/partition-to-k-equal-sum-subsets/)
-   [1411\. 给 N x 3 网格图涂色的方案数](https://leetcode.cn/problems/number-of-ways-to-paint-n-3-grid/) 1845
-   [2002\. 两个回文子序列长度的最大乘积](https://leetcode.cn/problems/maximum-product-of-the-length-of-two-palindromic-subsequences/) 1869
-   [473\. 火柴拼正方形](https://leetcode.cn/problems/matchsticks-to-square/)
-   [1931\. 用三种不同颜色为网格涂色](https://leetcode.cn/problems/painting-a-grid-with-three-different-colors/) 2170
-   [1125\. 最小的必要团队](https://leetcode.cn/problems/smallest-sufficient-team/) 2251
-   [1434\. 每个人戴不同帽子的方案数](https://leetcode.cn/problems/number-of-ways-to-wear-different-hats-to-each-other/) 2273
-   [464\. 我能赢吗](https://leetcode.cn/problems/can-i-win/)
-   [691\. 贴纸拼词](https://leetcode.cn/problems/stickers-to-spell-word/)
-   [1595\. 连通两组点的最小成本](https://leetcode.cn/problems/minimum-cost-to-connect-two-groups-of-points/) 2538
-   [1815\. 得到新鲜甜甜圈的最多组数](https://leetcode.cn/problems/maximum-number-of-groups-getting-fresh-donuts/) 2559
-   [1659\. 最大化网格幸福感](https://leetcode.cn/problems/maximize-grid-happiness/) 2655
-   [LCP 69. Hello LeetCode!](https://leetcode.cn/problems/rMeRt2/)
-   [LCP 76. 魔法棋盘](https://leetcode.cn/problems/1ybDKD/)
-   [LCP 82. 万灵之树](https://leetcode.cn/problems/cnHoX6/)
-   [351\. 安卓系统手势解锁](https://leetcode.cn/problems/android-unlock-patterns/)（会员题）
-   [2184\. 建造坚实的砖墙的方法数](https://leetcode.cn/problems/number-of-ways-to-build-sturdy-brick-wall/)（会员题）

## [](#十、数位-dp)十、数位 DP

[v1.0 模板讲解](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1rS4y1s721/?t=19m36s)

[v2.0 模板讲解](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1Fg4y1Q7wv/?t=31m28s)

-   [2719\. 统计整数数目](https://leetcode.cn/problems/count-of-integers/)
-   [788\. 旋转数字](https://leetcode.cn/problems/rotated-digits/)
-   [902\. 最大为 N 的数字组合](https://leetcode.cn/problems/numbers-at-most-n-given-digit-set/) 1990
-   [233\. 数字 1 的个数](https://leetcode.cn/problems/number-of-digit-one/)
-   [面试题 17.06. 2 出现的次数](https://leetcode.cn/problems/number-of-2s-in-range-lcci/)
-   [600\. 不含连续 1 的非负整数](https://leetcode.cn/problems/non-negative-integers-without-consecutive-ones/)
-   [2376\. 统计特殊整数](https://leetcode.cn/problems/count-special-integers/) 2120
-   [1012\. 至少有 1 位重复的数字](https://leetcode.cn/problems/numbers-with-repeated-digits/) 2230
-   [357\. 统计各位数字都不同的数字个数](https://leetcode.cn/problems/count-numbers-with-unique-digits/)
-   [3007\. 价值和小于等于 K 的最大数字](https://leetcode.cn/problems/maximum-number-that-sum-of-the-prices-is-less-than-or-equal-to-k/) 2258 做法不止一种
-   [2827\. 范围中美丽整数的数目](https://leetcode.cn/problems/number-of-beautiful-integers-in-the-range/) 2324
-   [2999\. 统计强大整数的数目](https://leetcode.cn/problems/count-the-number-of-powerful-integers/) 2351
-   [2801\. 统计范围内的步进数字数目](https://leetcode.cn/problems/count-stepping-numbers-in-range/) 2367
-   [1397\. 找到所有好字符串](https://leetcode.cn/problems/find-all-good-strings/) 2667
-   [1215\. 步进数](https://leetcode.cn/problems/stepping-numbers/)（会员题）
-   [1067\. 范围内的数字计数](https://leetcode.cn/problems/digit-count-in-range/)（会员题）
-   [3032\. 统计各位数字都不同的数字个数 II](https://leetcode.cn/problems/count-numbers-with-unique-digits-ii/)（会员题）
-   [1742\. 盒子中小球的最大数量](https://leetcode.cn/problems/maximum-number-of-balls-in-a-box/) \*非暴力做法 枚举数位和+DP
-   [2843\. 统计对称整数的数目](https://leetcode.cn/problems/count-symmetric-integers/) \*非暴力做法

## [](#十一、数据结构优化-dp)十一、数据结构优化 DP

### [](#§111-前缀和优化-dp)§11.1 前缀和优化 DP

-   [2327\. 知道秘密的人数](https://leetcode.cn/problems/number-of-people-aware-of-a-secret/) 1894
-   [1997\. 访问完所有房间的第一天](https://leetcode.cn/problems/first-day-where-you-have-been-in-all-the-rooms/) 2260
-   [2478\. 完美分割的方案数](https://leetcode.cn/problems/number-of-beautiful-partitions/) 2344
-   [837\. 新 21 点](https://leetcode.cn/problems/new-21-game/) 2350
-   [2463\. 最小移动总距离](https://leetcode.cn/problems/minimum-total-distance-traveled/) 2454
-   [629\. K 个逆序对数组](https://leetcode.cn/problems/k-inverse-pairs-array/)
-   [1977\. 划分数字的方案数](https://leetcode.cn/problems/number-of-ways-to-separate-numbers/) 2817
-   [3130\. 找出所有稳定的二进制数组 II](https://leetcode.cn/problems/find-all-possible-stable-binary-arrays-ii/) 2825

### [](#§112-单调栈优化-dp)§11.2 单调栈优化 DP

前置题单：[单调栈（矩形系列/字典序最小/贡献法）](https://leetcode.cn/circle/discuss/9oZFK9/)

-   [1335\. 工作计划的最低难度](https://leetcode.cn/problems/minimum-difficulty-of-a-job-schedule/) 2035
-   [2866\. 美丽塔 II](https://leetcode.cn/problems/beautiful-towers-ii/) 2072
-   [2617\. 网格图中最少访问的格子数](https://leetcode.cn/problems/minimum-number-of-visited-cells-in-a-grid/) 2582
-   [2355\. 你能拿走的最大图书数量](https://leetcode.cn/problems/maximum-number-of-books-you-can-take/)（会员题）

### [](#§113-单调队列优化-dp)§11.3 单调队列优化 DP

一般用来维护一段转移来源的最值。

1.  前提：区间右端点变大时，左端点也在变大（同滑动窗口）。
2.  转移前，去掉队首无用数据。
3.  计算转移（直接从队首转移）。
4.  把数据（一般是 $f[i][j]$）插入队尾前，去掉队尾无用数据。

-   [2944\. 购买水果需要的最少金币数](https://leetcode.cn/problems/minimum-number-of-coins-for-fruits/) 1709 可以用单调队列优化到 O(n)\\mathcal{O}(n)O(n)
-   [1696\. 跳跃游戏 VI](https://leetcode.cn/problems/jump-game-vi/) 1954
-   [1425\. 带限制的子序列和](https://leetcode.cn/problems/constrained-subsequence-sum/) 2032
-   [375\. 猜数字大小 II](https://leetcode.cn/problems/guess-number-higher-or-lower-ii/) 可以用单调队列优化到 O(n2)\\mathcal{O}(n^2)O(n2)
-   [1687\. 从仓库到码头运输箱子](https://leetcode.cn/problems/delivering-boxes-from-storage-to-ports/) 2610
-   [2463\. 最小移动总距离](https://leetcode.cn/problems/minimum-total-distance-traveled/) 做到 O(nm)\\mathcal{O}(nm)O(nm)（注：还有 O((n+m)log⁡(n+m))\\mathcal{O}((n+m)\\log(n+m))O((n+m)log(n+m)) 的反悔贪心做法）
-   [3117\. 划分数组得到最小的值之和](https://leetcode.cn/problems/minimum-sum-of-values-by-dividing-array/) 2735
-   [2945\. 找到最大非递减数组的长度](https://leetcode.cn/problems/find-maximum-non-decreasing-array-length/) 2943
-   [2969\. 购买水果需要的最少金币数 II](https://leetcode.cn/problems/minimum-number-of-coins-for-fruits-ii/)（会员题）

### [](#§114-树状数组/线段树优化-dp)§11.4 树状数组/线段树优化 DP

-   [1626\. 无矛盾的最佳球队](https://leetcode.cn/problems/best-team-with-no-conflicts/) 2027
-   [2407\. 最长递增子序列 II](https://leetcode.cn/problems/longest-increasing-subsequence-ii/) 2280
-   [2770\. 达到末尾下标所需的最大跳跃次数](https://leetcode.cn/problems/maximum-number-of-jumps-to-reach-the-last-index/) 见我题解下的 [评论](https://leetcode.cn/problems/maximum-number-of-jumps-to-reach-the-last-index/solutions/2336752/dong-tai-gui-hua-cong-ji-yi-hua-sou-suo-2ptkg/comments/2060234)
-   [2926\. 平衡子序列的最大和](https://leetcode.cn/problems/maximum-balanced-subsequence-sum/) 2448
-   [2916\. 子数组不同元素数目的平方和 II](https://leetcode.cn/problems/subarrays-distinct-element-sum-of-squares-ii/) 2816

### [](#§115-字典树优化-dp)§11.5 字典树优化 DP

-   [139\. 单词拆分](https://leetcode.cn/problems/word-break/)
-   [140\. 单词拆分 II](https://leetcode.cn/problems/word-break-ii/)
-   [472\. 连接词](https://leetcode.cn/problems/concatenated-words/) ~2300
-   [2977\. 转换字符串的最小成本 II](https://leetcode.cn/problems/minimum-cost-to-convert-string-ii/) 2696

### [](#§116-其他优化-dp)§11.6 其他优化 DP

-   [2713\. 矩阵中严格递增的单元格数](https://leetcode.cn/problems/maximum-strictly-increasing-cells-in-a-matrix/) 2387
-   [3181\. 执行操作可获得的最大总奖励 II](https://leetcode.cn/problems/maximum-total-reward-using-operations-ii/) 2688 bitset 优化
-   [LCP 59. 搭桥过河](https://leetcode.cn/problems/NfY1m5/)
-   [2263\. 数组变为有序的最小操作次数](https://leetcode.cn/problems/make-array-non-decreasing-or-non-increasing/)（会员题）Slope Trick

## [](#十二、树形-dp)十二、树形 DP

注：可能有同学觉得树形 DP 没有重复访问同一个状态（重叠子问题），并不能算作 DP，而是算作普通的递归。这么说也有一定道理，不过考虑到思维方式和 DP 是一样的自底向上，所以仍然叫做树形 DP。此外，如果是自顶向下的递归做法，是存在重叠子问题的，一般要结合记忆化搜索实现。

### [](#§121-树的直径)§12.1 树的直径

讲解：[树形 DP：树的直径](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV17o4y187h1/)

-   [543\. 二叉树的直径](https://leetcode.cn/problems/diameter-of-binary-tree/)
-   [124\. 二叉树中的最大路径和](https://leetcode.cn/problems/binary-tree-maximum-path-sum/)
-   [687\. 最长同值路径](https://leetcode.cn/problems/longest-univalue-path/)
-   [2246\. 相邻字符不同的最长路径](https://leetcode.cn/problems/longest-path-with-different-adjacent-characters/) 2126
-   [3203\. 合并两棵树后的最小直径](https://leetcode.cn/problems/find-minimum-diameter-after-merging-two-trees/) ~2150
-   [1617\. 统计子树中城市之间最大距离](https://leetcode.cn/problems/count-subtrees-with-max-distance-between-cities/) 2309
-   [2538\. 最大价值和与最小价值和的差值](https://leetcode.cn/problems/difference-between-maximum-and-minimum-price-sum/) 2398
-   [1245\. 树的直径](https://leetcode.cn/problems/tree-diameter/)（会员题）

> 注：求直径也有两次 DFS 的做法。

### [](#§122-树上最大独立集)§12.2 树上最大独立集

讲解：[树形 DP：打家劫舍$i$](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1vu4y1f7dn/)

-   [337\. 打家劫舍 $i$](https://leetcode.cn/problems/house-robber-$i$/)（没有上司的舞会）
-   [2646\. 最小化旅行的价格总和](https://leetcode.cn/problems/minimize-the-total-price-of-the-trips/) 2238
-   [2378\. 选择边来最大化树的得分](https://leetcode.cn/problems/choose-edges-to-maximize-score-in-a-tree/)（会员题）

### [](#§123-树上最小支配集)§12.3 树上最小支配集

讲解：[树形 DP：监控二叉树](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1oF411U7qL/)，包含 968 的变形题。

-   [968\. 监控二叉树](https://leetcode.cn/problems/binary-tree-cameras/) 2124

### [](#§124-换根-dp)§12.4 换根 DP

也叫二次扫描法。

[【图解】一张图秒懂换根 DP！](https://leetcode.cn/problems/sum-of-distances-in-tree/solution/tu-jie-yi-zhang-tu-miao-dong-huan-gen-dp-6bgb/)

-   [834\. 树中距离之和](https://leetcode.cn/problems/sum-of-distances-in-tree/) 2197
-   [2581\. 统计可能的树根数目](https://leetcode.cn/problems/count-number-of-possible-root-nodes/) 2228
-   [2858\. 可以到达每一个节点的最少边反转次数](https://leetcode.cn/problems/minimum-edge-reversals-so-every-node-is-reachable/) 2295
-   [310\. 最小高度树](https://leetcode.cn/problems/minimum-height-trees/) 也可以用拓扑排序做

### [](#§125-其他树形-dp)§12.5 其他树形 DP

-   [2925\. 在树上执行操作以后得到的最大分数](https://leetcode.cn/problems/maximum-score-after-applying-operations-on-a-tree/) 1940
-   [3068\. 最大节点价值之和](https://leetcode.cn/problems/find-the-maximum-sum-of-node-values/) 2268
-   [2920\. 收集所有金币可获得的最大积分](https://leetcode.cn/problems/maximum-points-after-collecting-coins-from-all-nodes/) 2351
-   [1916\. 统计为蚁群构筑房间的不同顺序](https://leetcode.cn/problems/count-ways-to-build-rooms-in-an-ant-colony/) 2486
-   [LCP 10. 二叉树任务调度](https://leetcode.cn/problems/er-cha-shu-ren-wu-diao-du/)
-   [LCP 34. 二叉树染色](https://leetcode.cn/problems/er-cha-shu-ran-se-UGC/)
-   [LCP 64. 二叉树灯饰](https://leetcode.cn/problems/U7WvvU/)
-   [2313\. 二叉树中得到结果所需的最少翻转次数](https://leetcode.cn/problems/minimum-flips-in-binary-tree-to-get-result/)（会员题）

## [](#十三、图-dp)十三、图 DP

-   [787\. K 站中转内最便宜的航班](https://leetcode.cn/problems/cheapest-flights-within-k-stops/) 1786
-   [1786\. 从第一个节点出发到最后一个节点的受限路径数](https://leetcode.cn/problems/number-of-restricted-paths-from-first-to-last-node/) 2079
-   [2050\. 并行课程 $i$](https://leetcode.cn/problems/parallel-courses-$i$/) 2084
-   [1976\. 到达目的地的方案数](https://leetcode.cn/problems/number-of-ways-to-arrive-at-destination/) 2095
-   [1857\. 有向图中最大颜色值](https://leetcode.cn/problems/largest-color-value-in-a-directed-graph/) 2313
-   [1928\. 规定时间内到达终点的最小花费](https://leetcode.cn/problems/minimum-cost-to-reach-destination-in-time/) 2413
-   [LCP 07. 传递信息](https://leetcode.cn/problems/chuan-di-xin-xi/)
-   [1548\. 图中最相似的路径](https://leetcode.cn/problems/the-most-similar-path-in-a-graph/)（会员题）

另见[【题单】图论算法](https://leetcode.cn/circle/discuss/01LUak/) 中的「全源最短路：Floyd」，本质是多维 DP。

## [](#十四、博弈-dp)十四、博弈 DP

-   [1025\. 除数博弈](https://leetcode.cn/problems/divisor-game/) 1435 有数学做法
-   [877\. 石子游戏](https://leetcode.cn/problems/stone-game/) 1590 有数学做法
-   [486\. 预测赢家](https://leetcode.cn/problems/predict-the-winner/)
-   [1510\. 石子游戏 IV](https://leetcode.cn/problems/stone-game-iv/) 1787
-   [1690\. 石子游戏 VII](https://leetcode.cn/problems/stone-game-vii/) 1951
-   [1406\. 石子游戏 $i$](https://leetcode.cn/problems/stone-game-$i$/) 2027
-   [1140\. 石子游戏 II](https://leetcode.cn/problems/stone-game-ii/) 2035
-   [1563\. 石子游戏 V](https://leetcode.cn/problems/stone-game-v/) 2087
-   [464\. 我能赢吗](https://leetcode.cn/problems/can-i-win/)
-   [1872\. 石子游戏 V$i$](https://leetcode.cn/problems/stone-game-v$i$/) 2440
-   [913\. 猫和老鼠](https://leetcode.cn/problems/cat-and-mouse/) 2567
-   [294\. 翻转游戏 II](https://leetcode.cn/problems/flip-game-ii/)（会员题）

## [](#十五、概率/期望-dp)十五、概率/期望 DP

-   [688\. 骑士在棋盘上的概率](https://leetcode.cn/problems/knight-probability-in-chessboard/)
-   [837\. 新 21 点](https://leetcode.cn/problems/new-21-game/) 2350
-   [1467\. 两个盒子中球的颜色数相同的概率](https://leetcode.cn/problems/probability-of-a-two-boxes-having-the-same-number-of-distinct-balls/) 2357
-   [808\. 分汤](https://leetcode.cn/problems/soup-servings/) 2397
-   [LCR 185. 统计结果概率](https://leetcode.cn/problems/nge-tou-zi-de-dian-shu-lcof/)
-   [九坤-04. 筹码游戏](https://leetcode.cn/contest/ubiquant2022/problems/I3Gm2h/)
-   [1230\. 抛掷硬币](https://leetcode.cn/problems/toss-strange-coins/)（会员题）

## [](#专题：输出具体方案（打印方案）)专题：输出具体方案（打印方案）

注意这些题目和回溯的区别，某些回溯题目要求输出**所有**方案，这里只要求输出**一个**。

[讲解](https://leetcode.cn/problems/shortest-common-supersequence/solution/cong-di-gui-dao-di-tui-jiao-ni-yi-bu-bu-auy8z/)

-   [368\. 最大整除子集](https://leetcode.cn/problems/largest-divisible-subset/)
-   [1449\. 数位成本和为目标值的最大数字](https://leetcode.cn/problems/form-largest-integer-with-digits-that-add-up-to-target/) 1927
-   [1092\. 最短公共超序列](https://leetcode.cn/problems/shortest-common-supersequence/) 1977
-   [943\. 最短超级串](https://leetcode.cn/problems/find-the-shortest-superstring/) 2186
-   [1125\. 最小的必要团队](https://leetcode.cn/problems/smallest-sufficient-team/) 2251
-   [3149\. 找出分数最低的排列](https://leetcode.cn/problems/find-the-minimum-cost-array-permutation/) 2642 字典序最小
-   [656\. 金币路径](https://leetcode.cn/problems/coin-path/)（会员题）字典序最小
-   [471\. 编码最短长度的字符串](https://leetcode.cn/problems/encode-string-with-shortest-length/)（会员题）

## [](#专题：前后缀分解)专题：前后缀分解

部分题目也可以用状态机 DP 解决。

-   [42\. 接雨水](https://leetcode.cn/problems/trapping-rain-water/)（[讲解](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1Qg411q7ia/?t=3m05s)）
-   [123\. 买卖股票的最佳时机 $i$](https://leetcode.cn/problems/best-time-to-buy-and-sell-stock-$i$/) 拆分成两个 121 题
-   [1422\. 分割字符串的最大得分](https://leetcode.cn/problems/maximum-score-after-splitting-a-string/) 1238
-   [2256\. 最小平均差](https://leetcode.cn/problems/minimum-average-difference/) 1395
-   [1493\. 删掉一个元素以后全为 1 的最长子数组](https://leetcode.cn/problems/longest-subarray-of-1s-after-deleting-one-element/) 1423
-   [845\. 数组中的最长山脉](https://leetcode.cn/problems/longest-mountain-in-array/) 1437 \*也可以分组循环
-   [2909\. 元素和最小的山形三元组 II](https://leetcode.cn/problems/minimum-sum-of-mountain-triplets-ii/) 1479
-   [2483\. 商店的最少代价](https://leetcode.cn/problems/minimum-penalty-for-a-shop/) 1495
-   [1525\. 字符串的好分割数目](https://leetcode.cn/problems/number-of-good-ways-to-split-a-string/) 1500
-   [2874\. 有序三元组中的最大值 II](https://leetcode.cn/problems/maximum-value-of-an-ordered-triplet-ii/) 1583
-   [1031\. 两个非重叠子数组的最大和](https://leetcode.cn/problems/maximum-sum-of-two-non-overlapping-subarrays/) 1680
-   [689\. 三个无重叠子数组的最大和](https://leetcode.cn/problems/maximum-sum-of-3-non-overlapping-subarrays/)
-   [2420\. 找到所有好下标](https://leetcode.cn/problems/find-all-good-indices/) 1695
-   [2100\. 适合野炊的日子](https://leetcode.cn/problems/find-good-days-to-rob-the-bank/) 1702
-   [926\. 将字符串翻转到单调递增](https://leetcode.cn/problems/flip-string-to-monotone-increasing/)
-   [334\. 递增的三元子序列](https://leetcode.cn/problems/increasing-triplet-subsequence/)
-   [2712\. 使所有字符相等的最小成本](https://leetcode.cn/problems/minimum-cost-to-make-all-characters-equal/) 1791
-   [1653\. 使字符串平衡的最少删除次数](https://leetcode.cn/problems/minimum-deletions-to-make-string-balanced/) 1794
-   [1186\. 删除一次得到子数组最大和](https://leetcode.cn/problems/maximum-subarray-sum-with-one-deletion/) 1799
-   [1477\. 找两个和为目标值且不重叠的子数组](https://leetcode.cn/problems/find-two-non-overlapping-sub-arrays-each-with-target-sum/) 1851
-   [2680\. 最大或值](https://leetcode.cn/problems/maximum-or/) 1912
-   [1671\. 得到山形数组的最少删除次数](https://leetcode.cn/problems/minimum-number-of-removals-to-make-mountain-array/) 1913
-   [238\. 除自身以外数组的乘积](https://leetcode.cn/problems/product-of-array-except-self/) ~2000
-   [1888\. 使二进制字符串字符交替的最少反转次数](https://leetcode.cn/problems/minimum-number-of-flips-to-make-the-binary-string-alternating/) 2006
-   [2906\. 构造乘积矩阵](https://leetcode.cn/problems/construct-product-matrix/) 2075
-   [2167\. 移除所有载有违禁货物车厢所需的最少时间](https://leetcode.cn/problems/minimum-time-to-remove-all-cars-containing-illegal-goods/) 2219
-   [2484\. 统计回文子序列数目](https://leetcode.cn/problems/count-palindromic-subsequences/) 2223
-   [2163\. 删除元素后和的最小差值](https://leetcode.cn/problems/minimum-difference-in-sums-after-removal-of-elements/) 2225
-   [2565\. 最少得分子序列](https://leetcode.cn/problems/subsequence-with-the-minimum-score/) 2432
-   [2552\. 统计上升四元组](https://leetcode.cn/problems/count-increasing-quadruplets/) 2433
-   [3003\. 执行操作后的最大分割数量](https://leetcode.cn/problems/maximize-the-number-of-partitions-after-operations/) 3039
-   [487\. 最大连续 1 的个数 II](https://leetcode.cn/problems/max-consecutive-ones-ii/)（会员题）
-   [1746\. 经过一次操作后的最大子数组和](https://leetcode.cn/problems/maximum-subarray-sum-after-one-operation/)（会员题）

## [](#专题：把-x-变成-y)专题：把 X 变成 Y

部分题目也可以用 BFS 解决。

-   [397\. 整数替换](https://leetcode.cn/problems/integer-replacement/)
-   [2998\. 使 X 和 Y 相等的最少操作次数](https://leetcode.cn/problems/minimum-number-of-operations-to-make-x-and-y-equal/) 1795
-   [2059\. 转化数字的最小运算数](https://leetcode.cn/problems/minimum-operations-to-convert-number/) 1850
-   [991\. 坏了的计算器](https://leetcode.cn/problems/broken-calculator/) 1909
-   [1553\. 吃掉 N 个橘子的最少天数](https://leetcode.cn/problems/minimum-number-of-days-to-eat-n-oranges/) 2048

## [](#专题：跳跃游戏)专题：跳跃游戏

-   [1306\. 跳跃游戏 $i$](https://leetcode.cn/problems/jump-game-$i$/) 1397
-   [2770\. 达到末尾下标所需的最大跳跃次数](https://leetcode.cn/problems/maximum-number-of-jumps-to-reach-the-last-index/) 1533
-   [403\. 青蛙过河](https://leetcode.cn/problems/frog-jump/)
-   [1340\. 跳跃游戏 V](https://leetcode.cn/problems/jump-game-v/) 1866
-   [1871\. 跳跃游戏 VII](https://leetcode.cn/problems/jump-game-vii/) 1896
-   [1696\. 跳跃游戏 VI](https://leetcode.cn/problems/jump-game-vi/) 1954
-   [975\. 奇偶跳](https://leetcode.cn/problems/odd-even-jump/) 2079
-   [1654\. 到家的最少跳跃次数](https://leetcode.cn/problems/minimum-jumps-to-reach-home/) 2124
-   [LCP 09. 最小跳跃次数](https://leetcode.cn/problems/zui-xiao-tiao-yue-ci-shu/)
-   [LCP 20. 快速公交](https://leetcode.cn/problems/meChtZ/)
-   [656\. 金币路径](https://leetcode.cn/problems/coin-path/)（会员题）
-   [2297\. 跳跃游戏 V$i$](https://leetcode.cn/problems/jump-game-v$i$/)（会员题）

## [](#其他-dp)其他 DP

-   [1526\. 形成目标数组的子数组最少增加次数](https://leetcode.cn/problems/minimum-number-of-increments-on-subarrays-to-form-a-target-array/) 1872
-   [823\. 带因子的二叉树](https://leetcode.cn/problems/binary-trees-with-factors/) 1900
-   [940\. 不同的子序列 II](https://leetcode.cn/problems/distinct-subsequences-ii/) 1985
-   [135\. 分发糖果](https://leetcode.cn/problems/candy/)
-   [650\. 两个键的键盘](https://leetcode.cn/problems/2-keys-keyboard/)
-   [467\. 环绕字符串中唯一的子字符串](https://leetcode.cn/problems/unique-substrings-in-wraparound-string/)
-   [2262\. 字符串的总引力](https://leetcode.cn/problems/total-appeal-of-a-string/) 2033
-   [828\. 统计子串中的唯一字符](https://leetcode.cn/problems/count-unique-characters-of-all-substrings-of-a-given-string/) 2034
-   [2746\. 字符串连接删减字母](https://leetcode.cn/problems/decremental-string-concatenation/) 2126
-   [2930\. 重新排列后包含指定子字符串的字符串数目](https://leetcode.cn/problems/number-of-strings-which-can-be-rearranged-to-contain-substring/) 2227
-   [3041\. 修改数组后最大化数组中的连续元素数目](https://leetcode.cn/problems/maximize-consecutive-elements-in-an-array-after-modification/) 2231
-   [1569\. 将子数组重新排序得到同一个二叉搜索树的方案数](https://leetcode.cn/problems/number-of-ways-to-reorder-array-to-get-same-bst/) 2288
-   [818\. 赛车](https://leetcode.cn/problems/race-car/) 2392
-   [920\. 播放列表的数量](https://leetcode.cn/problems/number-of-music-playlists/) 2400
-   [1388\. 3n 块披萨](https://leetcode.cn/problems/pizza-with-3n-slices/) 2410
-   [1987\. 不同的好子序列数目](https://leetcode.cn/problems/number-of-unique-good-subsequences/) 2422（同 940 题）
-   [903\. DI 序列的有效排列](https://leetcode.cn/problems/valid-permutations-for-di-sequence/) 2433
-   [2272\. 最大波动的子字符串](https://leetcode.cn/problems/substring-with-largest-variance/) 2516
-   [1896\. 反转表达式值的最少操作次数](https://leetcode.cn/problems/minimum-cost-to-change-the-final-value-of-expression/) 2532
-   [1531\. 压缩字符串 II](https://leetcode.cn/problems/string-compression-ii/) 2576
-   [964\. 表示数字的最少运算符](https://leetcode.cn/problems/least-operators-to-express-number/) 2594
-   [1787\. 使所有区间的异或结果为零](https://leetcode.cn/problems/make-the-xor-of-all-segments-equal-to-zero/) 2640
-   [2060\. 同源字符串检测](https://leetcode.cn/problems/check-if-an-original-string-exists-given-two-encoded-strings/) 2804
-   [2809\. 使数组和小于等于 x 的最少时间](https://leetcode.cn/problems/minimum-time-to-make-array-sum-at-most-x/) 2979 排序不等式
-   [LCP 14. 切分数组](https://leetcode.cn/problems/qie-fen-shu-zu/)
-   [LCP 36. 最多牌组数](https://leetcode.cn/problems/Up5XYM/)
-   [LCP 38. 守卫城堡](https://leetcode.cn/problems/7rLGCR/)
-   [LCP 43. 十字路口的交通](https://leetcode.cn/problems/Y1VbOX/)
-   [LCP 65. 舒适的湿度](https://leetcode.cn/problems/3aqs1c/)
-   [2189\. 建造纸牌屋的方法数](https://leetcode.cn/problems/number-of-ways-to-build-house-of-cards/)（会员题）
-   [2597\. 美丽子集的数目](https://leetcode.cn/problems/the-number-of-beautiful-subsets/) \*用 DP 解决
-   [2638\. 统计 K-Free 子集的总数](https://leetcode.cn/problems/count-the-number-of-k-free-subsets/)（会员题）上面这题的加强版