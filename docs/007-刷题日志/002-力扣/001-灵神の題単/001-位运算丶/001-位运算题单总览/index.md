# 【题单】位运算（基础/性质/拆位/试填/恒等式/贪心/脑筋急转弯）
> 来源: [分享丨【题单】位运算（基础/性质/拆位/试填/恒等式/贪心/脑筋急转弯）](https://leetcode.cn/circle/discuss/dHn9Vk/)
>
> 前置知识: [分享｜从集合论到位运算，常见位运算技巧分类总结！](https://leetcode.cn/circle/discuss/CaOJ45/)
>
> 创建题单的原因: [容斥原理 划分型 DP【力扣周赛 393】](https://www.bilibili.com/video/BV1dJ4m1V7hK/) 的Q3 我直接被秒杀!, Q4也是位运算(我看都没有看); 因此学习位运算十分有必要!

[3315. 构造最小位运算数组 II](https://blog.HXLoLi.com/blog/#/articles?articleId=21014 "##21014##") (lowbit)

### 基础题

- [x] [1486. 数组异或操作](../002-基础题/001-数组异或操作/index.md) 1181
- [x] [2595. 奇偶位数](../002-基础题/002-奇偶位数/index.md) 1207
- [x] [231. 2 的幂](../002-基础题/003-的幂/index.md)
- [x] [342. 4的幂](../002-基础题/003-的幂/index.md)
- [x] [476. 数字的补数](../002-基础题/004-数字的补数/index.md) 1235
- [x] [191. 位1的个数](../002-基础题/005-位的个数/index.md)
- [x] [338. 比特位计数](../002-基础题/006-比特位计数/index.md) 也可以 DP
- [x] [1356. 根据数字二进制下 1 的数目排序](https://leetcode.cn/problems/sort-integers-by-the-number-of-1-bits/) 1258
- [x] [461. 汉明距离](../002-基础题/007-汉明距离/index.md)
- [x] [2220. 转换数字的最少位翻转次数](https://leetcode.cn/problems/minimum-bit-flips-to-convert-number/) 1282
- [x] [868. 二进制间距](https://leetcode.cn/problems/binary-gap/) 1307 (模拟 / 双指针)
- [x] [2917. 找出数组中的 K-or 值](https://leetcode.cn/problems/find-the-k-or-of-an-array/) 1389
- [x] [693. 交替位二进制数](../002-基础题/008-交替位二进制数/index.md)

### 与或（AND/OR）的性质

- [x] [2980. 检查按位或是否存在尾随零](https://leetcode.cn/problems/check-if-bitwise-or-has-trailing-zeros/) 1234
- [x] [1318. 或运算的最小翻转次数](https://leetcode.cn/problems/minimum-flips-to-make-a-or-b-equal-to-c/) 1383
- [x] [2419. 按位与最大的最长子数组](https://leetcode.cn/problems/longest-subarray-with-maximum-bitwise-and/) 1496 (脑筋急转弯(`&`的性质))
- [x] [2871. 将数组分割成最多数目的子数组](../003-与或【AND、OR】的性质/001-将数组分割成最多数目的子数组/index.md) 1750
- [x] [2401. 最长优雅子数组](../003-与或【AND、OR】的性质/002-最长优雅子数组/index.md) 1750
- [x] [2680. 最大或值](../003-与或【AND、OR】的性质/003-最大或值/index.md) $O(1)$ 额外空间
- [x] [2411. 按位或最大的最小子数组长度](../003-与或【AND、OR】的性质/004-按位或最大的最小子数组长度/index.md) 1938
-   [898\. 子数组按位或操作](https://leetcode.cn/problems/bitwise-ors-of-subarrays/) 2133
-   [1521\. 找到最接近目标值的函数值](https://leetcode.cn/problems/find-a-value-of-a-mysterious-function-closest-to-target/) 2384

### 异或（XOR）的性质

- [x] [1720. 解码异或后的数组](https://leetcode.cn/problems/decode-xored-array/) 1284
- [x] [2433. 找出前缀异或的原始数组](https://leetcode.cn/problems/find-the-original-array-of-prefix-xor/) 1367 (前缀异或和差分复原)
- [x] [1310. 子数组异或查询](https://leetcode.cn/problems/xor-queries-of-a-subarray/) 1460 (前缀异或和/树状数组)
- [x] [2683. 相邻值的按位异或](../004-异或（XOR）的性质/001-相邻值的按位异或/index.md) 1518
- [x] [1829. 每个查询的最大异或值](../004-异或（XOR）的性质/002-每个查询的最大异或值/index.md) 1523
- [x] [2997. 使数组异或和等于 K 的最少操作次数](https://leetcode.cn/problems/minimum-number-of-operations-to-make-array-xor-equal-to-k/) 1525
- [x] [1442. 形成两个异或相等数组的三元组数目](../004-异或（XOR）的性质/003-形成两个异或相等数组的三元组数目/index.md) 1525 (前缀和 + 哈希表 => o(n))
- [x] [2429 最小异或](https://leetcode.cn/problems/minimize-xor/) 1532
- [x] [2527. 查询数组异或美丽值](../004-异或（XOR）的性质/004-查询数组异或美丽值/index.md) 1550 (orz)
- [x] [2317\. 操作后的最大异或和](https://leetcode.cn/problems/maximum-xor-after-operations/) 1679
- [x] [2588. 统计美丽子数组数目](../004-异或（XOR）的性质/005-统计美丽子数组数目/index.md) 1697 (固定套路! 学会转换!)
- [x] [2564. 子字符串异或查询](../004-异或（XOR）的性质/006-子字符串异或查询/index.md) 1959
-   [1734\. 解码异或后的排列](https://leetcode.cn/problems/decode-xored-permutation/) 2024
-   [2857\. 统计距离为 k 的点对](https://leetcode.cn/problems/count-pairs-of-points-with-distance-k/) 2082

### 拆位 / 贡献法

- [x] [477. 汉明距离总和](../005-拆位、贡献法/001-汉明距离总和/index.md)
- [x] [1863. 找出所有子集的异或总和再求和](../005-拆位、贡献法/002-找出所有子集的异或总和再求和/index.md) 可以做到 $O(n)$ 时间 (看不懂思密达)
- [x] [2425. 所有数对的异或和](../005-拆位、贡献法/003-所有数对的异或和/index.md) 1622 可以做到 $O(n+m)$ 时间
- [x] [2275. 按位与结果大于零的最长组合](../005-拆位、贡献法/004-按位与结果大于零的最长组合/index.md) 1642
- [x] [1835. 所有数对按位与结果的异或和](../005-拆位、贡献法/005-所有数对按位与结果的异或和/index.md) 1825 也有恒等式做法
-   [2505\. 所有子序列和的按位或](https://leetcode.cn/problems/bitwise-or-of-all-subsequence-sums/)（会员题）

### LogTrick 技巧

> LogTrick: 问一个数组的所有子数组的元素 AND/OR/lcm/gcd 的值为 k (或者是某个可能含有子数组的表达式的值) 的最近元素/子数组个数(二分`right-left`/三分(滑窗))
>
> LogTrick利用AND/OR/lcm/gcd的性质, 使得时间复杂度为 $O(n\log{n})$

视频讲解可以看`周赛400 Q4`: [Q4-3171. 找到按位或最接近 K 的子数组](../../../002-力扣周赛霊解/001-周赛/001-第场周赛/028-Q丶找到按位或最接近K的子数组/index.md)

- 原地写法: [三种基于 logTrick 的方法：二分 / 三指针 / 维护个数（Python/Java/C++/Go）](https://leetcode.cn/problems/number-of-subarrays-with-and-value-of-k/solutions/2833497/jian-ji-xie-fa-o1-kong-jian-pythonjavacg-u7fv/)

- 额外维护一个数组: [利用或运算的性质 + 通用模板（Python/Java/C++/Go）](https://leetcode.cn/problems/smallest-subarrays-with-maximum-bitwise-or/solutions/1830911/by-endlesscheng-zai1)

- [x] [3097. 或值至少为 K 的最短子数组 II](https://leetcode.cn/problems/shortest-subarray-with-or-at-least-k-ii/) 1891
- [x] [2411. 按位或最大的最小子数组长度](https://leetcode.cn/problems/smallest-subarrays-with-maximum-bitwise-or/) 1938
- [x] [3209. 子数组按位与值为 K 的数目](https://leetcode.cn/problems/number-of-subarrays-with-and-value-of-k/) ~2100
- [x] [3171. 找到按位或最接近 K 的子数组](https://leetcode.cn/problems/find-subarray-with-bitwise-or-closest-to-k/)
- [x] [1521. 找到最接近目标值的函数值](https://leetcode.cn/problems/find-a-value-of-a-mysterious-function-closest-to-target/) 2384 做法同 3171 题
- [x] [898. 子数组按位或操作](https://leetcode.cn/problems/bitwise-ors-of-subarrays/)

### 试填法

-   [3007\. 价值和小于等于 K 的最大数字](https://leetcode.cn/problems/maximum-number-that-sum-of-the-prices-is-less-than-or-equal-to-k/) 2258
-   [421\. 数组中两个数的最大异或值](https://leetcode.cn/problems/maximum-xor-of-two-numbers-in-an-array/)，[试填法题解](https://leetcode.cn/problems/maximum-xor-of-two-numbers-in-an-array/solution/tu-jie-jian-ji-gao-xiao-yi-tu-miao-dong-1427d/)
-   [2935\. 找出强数对的最大异或值 II](https://leetcode.cn/problems/maximum-strong-pair-xor-ii/) 2349
-   [3022\. 给定操作次数内使剩余元素的或值最小](https://leetcode.cn/problems/minimize-or-of-remaining-elements-using-operations/) 2918

### 恒等式

- [x] [1835. 所有数对按位与结果的异或和](../005-拆位、贡献法/005-所有数对按位与结果的异或和/index.md) 1825
-   [2354\. 优质数对的数目](https://leetcode.cn/problems/number-of-excellent-pairs/) 2076

### 思维题（贪心、脑筋急转弯等）

- [x] [2546\. 执行逐位运算使字符串相等](https://leetcode.cn/problems/apply-bitwise-operations-to-make-strings-equal/) 1605
- [x] [1558\. 得到目标数组的最少函数调用次数](https://leetcode.cn/problems/minimum-numbers-of-function-calls-to-make-target-array/) 1637
-   [2571\. 将整数减少到零需要的最少操作数](https://leetcode.cn/problems/minimum-operations-to-reduce-an-integer-to-0/) 1649 巧妙结论
-   [2568\. 最小无法得到的或值](https://leetcode.cn/problems/minimum-impossible-or/) 1754
-   [2939\. 最大异或乘积](https://leetcode.cn/problems/maximum-xor-product/) 2128
-   [2749\. 得到整数零需要执行的最少操作数](https://leetcode.cn/problems/minimum-operations-to-make-the-integer-zero/) 2132
-   [2835\. 使子序列的和等于目标的最少操作次数](https://leetcode.cn/problems/minimum-operations-to-form-subsequence-with-target-sum/) 2207
-   [2897\. 对数组执行操作使平方和最大](https://leetcode.cn/problems/apply-operations-on-array-to-maximize-sum-of-squares/) 2301
-   [810\. 黑板异或游戏](https://leetcode.cn/problems/chalkboard-xor-game/) 2341

### 其它

- [x] [136. 只出现一次的数字](https://leetcode.cn/problems/single-number/) (异或的性质)
- [x] [287. 寻找重复数](https://leetcode.cn/problems/find-the-duplicate-number/) (去除某个数, 然后统计1的个数复原它)
- [x] [260. 只出现一次的数字 III](../004-异或（XOR）的性质/008-只出现一次的数字III/index.md)
- [x] [137. 只出现一次的数字 II](../004-异或（XOR）的性质/007-只出现一次的数字II/index.md) (统计位数次数)
-   [645\. 错误的集合](https://leetcode.cn/problems/set-mismatch/)
-   [190\. 颠倒二进制位](https://leetcode.cn/problems/reverse-bits/)
- [x] [371. 两整数之和](https://leetcode.cn/problems/sum-of-two-integers/)
```C++
int getSum(int a, int b) {
    int tmp = (a & b) << 1;         // 记得要左移一位!
    int res = a ^ b;                // 相加(不理会进位的)
    while (tmp) {                   // 是否要进位
        int cache = tmp;
        tmp = (res & tmp) << 1;     // 进位
        res = res ^ cache;          // 相加(不理会进位的)
    }
    return res;
}
```
-   [201\. 数字范围按位与](https://leetcode.cn/problems/bitwise-and-of-numbers-range/)
-   [2154\. 将找到的值乘以 2](https://leetcode.cn/problems/keep-multiplying-found-values-by-two/) 可以做到 $O(n)$ 时间
- [x] [2044. 统计按位或能得到最大值的子集数目](https://leetcode.cn/problems/count-number-of-maximum-bitwise-or-subsets/) 1568 (子集型回溯)
-   [2438\. 二的幂数组中查询范围内的乘积](https://leetcode.cn/problems/range-product-queries-of-powers/) 1610
-   [1680\. 连接连续二进制数字](https://leetcode.cn/problems/concatenation-of-consecutive-binary-numbers/) 1630
-   [982\. 按位与为零的三元组](https://leetcode.cn/problems/triples-with-bitwise-and-equal-to-zero/) 2085
-   [1611\. 使整数变为 0 的最少操作次数](https://leetcode.cn/problems/minimum-one-bit-operations-to-make-integers-zero/) 2345