# [分享丨【题单】常用数据结构（前缀和/差分/栈/队列/堆/字典树/并查集/树状数组/线段树）](https://leetcode.cn/circle/discuss/mOr1u6/)

> By <span style="color:rgb(255, 161, 22)">灵茶山艾府</span>

题目已按照难度分排序，右侧数字为难度分。

如果遇到难度很大，题解都看不懂的题目，建议直接跳过，二刷的时候再来尝试。

![数据结构题单数据结构入门数据结构新手教程](https://pic.leetcode.cn/1713836847-cDYwhf-Programming.jfif)

## [](#一、前缀和)一、前缀和

### [](#§11-基础)§1.1 基础

[原理讲解](https://leetcode.cn/problems/range-sum-query-immutable/solution/qian-zhui-he-ji-qi-kuo-zhan-fu-ti-dan-py-vaar/)

-   [303\. 区域和检索 - 数组不可变](https://leetcode.cn/problems/range-sum-query-immutable/)
-   [2559\. 统计范围内的元音字符串数](https://leetcode.cn/problems/count-vowel-strings-in-ranges/) 1435
-   [2389\. 和有限的最长子序列](https://leetcode.cn/problems/longest-subsequence-with-limited-sum/)
-   [2438\. 二的幂数组中查询范围内的乘积](https://leetcode.cn/problems/range-product-queries-of-powers/) 1610
-   [2055\. 蜡烛之间的盘子](https://leetcode.cn/problems/plates-between-candles/) 1819
-   [1744\. 你能在你最喜欢的那天吃到你最喜欢的糖果吗？](https://leetcode.cn/problems/can-you-eat-your-favorite-candy-on-your-favorite-day/) 1859
-   [53\. 最大子数组和](https://leetcode.cn/problems/maximum-subarray/)（[题解](https://leetcode.cn/problems/maximum-subarray/solution/qian-zhui-he-zuo-fa-ben-zhi-shi-mai-mai-abu71/)）

### [](#§12-前缀和与哈希表)§1.2 前缀和与哈希表

-   [930\. 和相同的二元子数组](https://leetcode.cn/problems/binary-subarrays-with-sum/) 1592
-   [560\. 和为 K 的子数组](https://leetcode.cn/problems/subarray-sum-equals-k/)
-   [1524\. 和为奇数的子数组数目](https://leetcode.cn/problems/number-of-sub-arrays-with-odd-sum/) 1611
-   [974\. 和可被 K 整除的子数组](https://leetcode.cn/problems/subarray-sums-divisible-by-k/) 1676
-   [523\. 连续的子数组和](https://leetcode.cn/problems/continuous-subarray-sum/)
-   [437\. 路径总和 III](https://leetcode.cn/problems/path-sum-iii/)
-   [2588\. 统计美丽子数组数目](https://leetcode.cn/problems/count-the-number-of-beautiful-subarrays/) 1697
-   [525\. 连续数组](https://leetcode.cn/problems/contiguous-array/)
-   [3026\. 最大好子数组和](https://leetcode.cn/problems/maximum-good-subarray-sum/) 1817
-   [1546\. 和为目标值且不重叠的非空子数组的最大数目](https://leetcode.cn/problems/maximum-number-of-non-overlapping-subarrays-with-sum-equals-target/) 1855
-   [面试题 17.05. 字母与数字](https://leetcode.cn/problems/find-longest-subarray-lcci/)
-   [1124\. 表现良好的最长时间段](https://leetcode.cn/problems/longest-well-performing-interval/) 1908
-   [2488\. 统计中位数为 K 的子数组](https://leetcode.cn/problems/count-subarrays-with-median-k/) 1999
-   [1590\. 使数组和能被 P 整除](https://leetcode.cn/problems/make-sum-divisible-by-p/) 2039
-   [2845\. 统计趣味子数组的数目](https://leetcode.cn/problems/count-of-interesting-subarrays/) 2073
-   [1442\. 形成两个异或相等数组的三元组数目](https://leetcode.cn/problems/count-triplets-that-can-form-two-arrays-of-equal-xor/) 做到 $O(n)$
-   [2025\. 分割数组的最多方案数](https://leetcode.cn/problems/maximum-number-of-ways-to-partition-an-array/) 2218
-   [2949\. 统计美丽子字符串 II](https://leetcode.cn/problems/count-beautiful-substrings-ii/) 2445
-   [325\. 和等于 k 的最长子数组长度](https://leetcode.cn/problems/maximum-size-subarray-sum-equals-k/)（会员题）
-   [548\. 将数组分割成和相等的子数组](https://leetcode.cn/problems/split-array-with-equal-sum/)（会员题）
-   [1983\. 范围和相等的最宽索引对](https://leetcode.cn/problems/widest-pair-of-indices-with-equal-range-sum/)（会员题）
-   [2489\. 固定比率的子字符串数](https://leetcode.cn/problems/number-of-substrings-with-fixed-ratio/)（会员题）
-   [2950\. 可整除子串的数量](https://leetcode.cn/problems/number-of-divisible-substrings/)（会员题）

### [](#§13-距离和)§1.3 距离和

[图解](https://leetcode.cn/problems/minimum-operations-to-make-all-array-elements-equal/solution/yi-tu-miao-dong-pai-xu-qian-zhui-he-er-f-nf55/)

-   [1685\. 有序数组中差绝对值之和](https://leetcode.cn/problems/sum-of-absolute-differences-in-a-sorted-array/) 1496
-   [2615\. 等值距离和](https://leetcode.cn/problems/sum-of-distances/) 1793
-   [2602\. 使数组元素全部相等的最少操作次数](https://leetcode.cn/problems/minimum-operations-to-make-all-array-elements-equal/) 1903
-   [2968\. 执行操作使频率分数最大](https://leetcode.cn/problems/apply-operations-to-maximize-frequency-score/) 2444
-   [1703\. 得到连续 K 个 1 的最少相邻交换次数](https://leetcode.cn/problems/minimum-adjacent-swaps-for-k-consecutive-ones/) 2467
-   [3086\. 拾起 K 个 1 需要的最少行动次数](https://leetcode.cn/problems/minimum-moves-to-pick-k-ones/) 2673

### [](#§14-前缀异或和)§1.4 前缀异或和

-   [1310\. 子数组异或查询](https://leetcode.cn/problems/xor-queries-of-a-subarray/) 1460
- [x] [1177. 构建回文串检测](../002-前缀和/001-前缀异或和/001-构建回文串检测/index.md) 1848
-   [1371\. 每个元音包含偶数次的最长子字符串](https://leetcode.cn/problems/find-the-longest-substring-containing-vowels-in-even-counts/) 2041
-   [1542\. 找出最长的超赞子字符串](https://leetcode.cn/problems/find-longest-awesome-substring/) 2222
-   [1915\. 最美子字符串的数目](https://leetcode.cn/problems/number-of-wonderful-substrings/) 2235
-   [2791\. 树中可以形成回文的路径数](https://leetcode.cn/problems/count-paths-that-can-form-a-palindrome-in-a-tree/) 2677

### [](#§15-其它一维前缀和)§1.5 其它一维前缀和

-   [1895\. 最大的幻方](https://leetcode.cn/problems/largest-magic-square/) 1781
-   [1878\. 矩阵中最大的三个菱形和](https://leetcode.cn/problems/get-biggest-three-rhombus-sums-in-a-grid/) 1898 斜向前缀和
-   [2245\. 转角路径的乘积中最多能有几个尾随零](https://leetcode.cn/problems/maximum-trailing-zeros-in-a-cornered-path/) 2037
-   [1712\. 将数组分成三个子数组的方案数](https://leetcode.cn/problems/ways-to-split-array-into-three-subarrays/) 2079
-   [1862\. 向下取整数对和](https://leetcode.cn/problems/sum-of-floored-pairs/) 2170
-   [363\. 矩形区域不超过 K 的最大数值和](https://leetcode.cn/problems/max-sum-of-rectangle-no-larger-than-k/) 枚举上下边界转换成一维
-   [2281\. 巫师的总力量和](https://leetcode.cn/problems/sum-of-total-strength-of-wizards/) 2621
-   [2983\. 回文串重新排列查询](https://leetcode.cn/problems/palindrome-rearrangement-queries/) 2780
-   [2955\. 同端子串的数量](https://leetcode.cn/problems/number-of-same-end-substrings/)（会员题）
-   [2819\. 购买巧克力后的最小相对损失](https://leetcode.cn/problems/minimum-relative-loss-after-buying-chocolates/)（会员题）

### [](#§16-二维前缀和)§1.6 二维前缀和

[【图解】一张图秒懂二维前缀和！](https://leetcode.cn/problems/range-sum-query-2d-immutable/solution/tu-jie-yi-zhang-tu-miao-dong-er-wei-qian-84qp/)

-   [304\. 二维区域和检索 - 矩阵不可变](https://leetcode.cn/problems/range-sum-query-2d-immutable/)
-   [1314\. 矩阵区域和](https://leetcode.cn/problems/matrix-block-sum/) 1484
-   [3070\. 元素和小于等于 k 的子矩阵的数目](https://leetcode.cn/problems/count-submatrices-with-top-left-element-and-sum-less-than-k/) 1499
-   [1277\. 统计全为 1 的正方形子矩阵](https://leetcode.cn/problems/count-square-submatrices-with-all-ones/) 1613
-   [1738\. 找出第 K 大的异或坐标值](https://leetcode.cn/problems/find-kth-largest-xor-coordinate-value/) 1671
-   [221\. 最大正方形](https://leetcode.cn/problems/maximal-square/)
-   [1292\. 元素和小于等于阈值的正方形的最大边长](https://leetcode.cn/problems/maximum-side-length-of-a-square-with-sum-less-than-or-equal-to-threshold/) 1735
-   [1504\. 统计全 1 子矩形](https://leetcode.cn/problems/count-submatrices-with-all-ones/) 1845
-   [1074\. 元素和为目标值的子矩阵数量](https://leetcode.cn/problems/number-of-submatrices-that-sum-to-target/) 2189

二维前缀最小值：

-   [3148\. 矩阵中的最大得分](https://leetcode.cn/problems/maximum-difference-score-in-a-grid/) ~1800

## [](#二、差分)二、差分

### [](#§21-一维差分（扫描线）)§2.1 一维差分（扫描线）

[原理讲解](https://leetcode.cn/problems/car-pooling/solution/suan-fa-xiao-ke-tang-chai-fen-shu-zu-fu-9d4ra/)（推荐和[【图解】从一维差分到二维差分](https://leetcode.cn/problems/stamping-the-grid/solution/wu-nao-zuo-fa-er-wei-qian-zhui-he-er-wei-zwiu/) 一起看）

-   [2848\. 与车相交的点](https://leetcode.cn/problems/points-that-intersect-with-cars/) 1230
-   [1893\. 检查是否区域内所有整数都被覆盖](https://leetcode.cn/problems/check-if-all-the-integers-in-a-range-are-covered/) 1307
-   [1854\. 人口最多的年份](https://leetcode.cn/problems/maximum-population-year/) 1370
-   [2960\. 统计已测试设备](https://leetcode.cn/problems/count-tested-devices-after-test-operations/)
-   [1094\. 拼车](https://leetcode.cn/problems/car-pooling/) 1441
-   [1109\. 航班预订统计](https://leetcode.cn/problems/corporate-flight-bookings/) 1570
-   [56\. 合并区间](https://leetcode.cn/problems/merge-intervals/)
-   [57\. 插入区间](https://leetcode.cn/problems/insert-interval/)
-   [732\. 我的日程安排表 III](https://leetcode.cn/problems/my-calendar-iii/)
-   [2406\. 将区间分为最少组数](https://leetcode.cn/problems/divide-intervals-into-minimum-number-of-groups/) 1713
-   [2381\. 字母移位 II](https://leetcode.cn/problems/shifting-letters-ii/) 1793
-   [995\. K 连续位的最小翻转次数](https://leetcode.cn/problems/minimum-number-of-k-consecutive-bit-flips/) 1835
-   [1589\. 所有排列中的最大和](https://leetcode.cn/problems/maximum-sum-obtained-of-any-permutation/) 1871
-   [1943\. 描述绘画结果](https://leetcode.cn/problems/describe-the-painting/) 1969
-   [2251\. 花期内花的数目](https://leetcode.cn/problems/number-of-flowers-in-full-bloom/) 2022
-   [2772\. 使数组中的所有元素都等于零](https://leetcode.cn/problems/apply-operations-to-make-all-array-elements-equal-to-zero/) 2029
-   [798\. 得分最高的最小轮调](https://leetcode.cn/problems/smallest-rotation-with-highest-score/) 2130
-   [2528\. 最大化城市的最小供电站数目](https://leetcode.cn/problems/maximize-the-minimum-powered-city/) 2236
-   [1674\. 使数组互补的最少操作次数](https://leetcode.cn/problems/minimum-moves-to-make-array-complementary/) 2333
-   [3017\. 按距离统计房屋对数目 II](https://leetcode.cn/problems/count-the-number-of-houses-at-a-certain-distance-ii/) 2709
-   [253\. 会议室 II](https://leetcode.cn/problems/meeting-rooms-ii/)（会员题）
-   [370\. 区间加法](https://leetcode.cn/problems/range-addition/)（会员题）
-   [759\. 员工空闲时间](https://leetcode.cn/problems/employee-free-time/)（会员题）
-   [2021\. 街上最亮的位置](https://leetcode.cn/problems/brightest-position-on-street/)（会员题）
-   [2015\. 每段建筑物的平均高度](https://leetcode.cn/problems/average-height-of-buildings-in-each-segment/)（会员题）
-   [2237\. 计算街道上满足所需亮度的位置数量](https://leetcode.cn/problems/count-positions-on-street-with-required-brightness/)（会员题）
-   [3009\. 折线图上的最大交点数量](https://leetcode.cn/problems/maximum-number-of-intersections-on-the-chart/)（会员题）

### [](#§22-二维差分)§2.2 二维差分

[【图解】从一维差分到二维差分](https://leetcode.cn/problems/stamping-the-grid/solution/wu-nao-zuo-fa-er-wei-qian-zhui-he-er-wei-zwiu/)

-   [2536\. 子矩阵元素加 1](https://leetcode.cn/problems/increment-submatrices-by-one/) 1583
-   [850\. 矩形面积 II](https://leetcode.cn/problems/rectangle-area-ii/) 2236
-   [2132\. 用邮票贴满网格图](https://leetcode.cn/problems/stamping-the-grid/) 2364
-   [LCP 74. 最强祝福力场](https://leetcode.cn/problems/xepqZ5/)

## [](#三、栈)三、栈

### [](#§31-基础)§3.1 基础

-   [1441\. 用栈操作构建数组](https://leetcode.cn/problems/build-an-array-with-stack-operations/) 1180
-   [844\. 比较含退格的字符串](https://leetcode.cn/problems/backspace-string-compare/) 1228
-   [682\. 棒球比赛](https://leetcode.cn/problems/baseball-game/)
-   [2390\. 从字符串中移除星号](https://leetcode.cn/problems/removing-stars-from-a-string/) 1348
-   [1472\. 设计浏览器历史记录](https://leetcode.cn/problems/design-browser-history/) 1454
-   [946\. 验证栈序列](https://leetcode.cn/problems/validate-stack-sequences/) 1462
-   [71\. 简化路径](https://leetcode.cn/problems/simplify-path/)

### [](#§32-进阶)§3.2 进阶

-   [155\. 最小栈](https://leetcode.cn/problems/min-stack/)
-   [1381\. 设计一个支持增量操作的栈](https://leetcode.cn/problems/design-a-stack-with-increment-operation/)
-   [636\. 函数的独占时间](https://leetcode.cn/problems/exclusive-time-of-functions/)
-   [2434\. 使用机器人打印字典序最小的字符串](https://leetcode.cn/problems/using-a-robot-to-print-the-lexicographically-smallest-string/) 1953
-   [895\. 最大频率栈](https://leetcode.cn/problems/maximum-frequency-stack/) 2028
-   [1172\. 餐盘栈](https://leetcode.cn/problems/dinner-plate-stacks/) 2110
-   [2589\. 完成所有任务的最少时间](https://leetcode.cn/problems/minimum-time-to-complete-all-tasks/) 2381 做法不止一种
-   [716\. 最大栈](https://leetcode.cn/problems/max-stack/)（会员题）

### [](#§33-邻项消除)§3.3 邻项消除

-   [2696\. 删除子串后的字符串最小长度](https://leetcode.cn/problems/minimum-string-length-after-removing-substrings/) 1282
-   [1047\. 删除字符串中的所有相邻重复项](https://leetcode.cn/problems/remove-all-adjacent-duplicates-in-string/) 1286
-   [1544\. 整理字符串](https://leetcode.cn/problems/make-the-string-great/) 1344
-   [1003\. 检查替换后的词是否有效](https://leetcode.cn/problems/check-if-word-is-valid-after-substitutions/) 1427
-   [2216\. 美化数组的最少删除数](https://leetcode.cn/problems/minimum-deletions-to-make-array-beautiful/) 1510
-   [1209\. 删除字符串中的所有相邻重复项 II](https://leetcode.cn/problems/remove-all-adjacent-duplicates-in-string-ii/) 1542
-   [2211\. 统计道路上的碰撞次数](https://leetcode.cn/problems/count-collisions-on-a-road/) 1581
-   [735\. 小行星碰撞](https://leetcode.cn/problems/asteroid-collision/)
-   [2197\. 替换数组中的非互质数](https://leetcode.cn/problems/replace-non-coprime-numbers-in-array/) 2057
-   [2751\. 机器人碰撞](https://leetcode.cn/problems/robot-collisions/) 2092

### [](#§34-合法括号字符串)§3.4 合法括号字符串

注：部分题目可以不用栈，而是用一个数字记录嵌套深度。

-   [20\. 有效的括号](https://leetcode.cn/problems/valid-parentheses/)
-   [921\. 使括号有效的最少添加](https://leetcode.cn/problems/minimum-add-to-make-parentheses-valid/) 1242
-   [1021\. 删除最外层的括号](https://leetcode.cn/problems/remove-outermost-parentheses/) 1311
-   [1614\. 括号的最大嵌套深度](https://leetcode.cn/problems/maximum-nesting-depth-of-the-parentheses/) 1323
-   [1190\. 反转每对括号间的子串](https://leetcode.cn/problems/reverse-substrings-between-each-pair-of-parentheses/) 1486
-   [856\. 括号的分数](https://leetcode.cn/problems/score-of-parentheses/) 1563
-   [1249\. 移除无效的括号](https://leetcode.cn/problems/minimum-remove-to-make-valid-parentheses/) 1657
-   [1963\. 使字符串平衡的最小交换次数](https://leetcode.cn/problems/minimum-number-of-swaps-to-make-the-string-balanced/) 1689
-   [32\. 最长有效括号](https://leetcode.cn/problems/longest-valid-parentheses/)
-   [1111\. 有效括号的嵌套深度](https://leetcode.cn/problems/maximum-nesting-depth-of-two-valid-parentheses-strings/) 1749
-   [1541\. 平衡括号字符串的最少插入次数](https://leetcode.cn/problems/minimum-insertions-to-balance-a-parentheses-string/) 1759
-   [2116\. 判断一个括号字符串是否有效](https://leetcode.cn/problems/check-if-a-parentheses-string-can-be-valid/) 2038

### [](#§35-表达式解析)§3.5 表达式解析

-   [150\. 逆波兰表达式求值](https://leetcode.cn/problems/evaluate-reverse-polish-notation/)
-   [1006\. 笨阶乘](https://leetcode.cn/problems/clumsy-factorial/) 1408
-   [224\. 基本计算器](https://leetcode.cn/problems/basic-calculator/)
-   [227\. 基本计算器 II](https://leetcode.cn/problems/basic-calculator-ii/)
-   [726\. 原子的数量](https://leetcode.cn/problems/number-of-atoms/)
-   [1106\. 解析布尔表达式](https://leetcode.cn/problems/parsing-a-boolean-expression/) 1880
-   [591\. 标签验证器](https://leetcode.cn/problems/tag-validator/)
-   [736\. Lisp 语法解析](https://leetcode.cn/problems/parse-lisp-expression/)
-   [1096\. 花括号展开 II](https://leetcode.cn/problems/brace-expansion-ii/) 2349
-   [1896\. 反转表达式值的最少操作次数](https://leetcode.cn/problems/minimum-cost-to-change-the-final-value-of-expression/) 2532
-   [770\. 基本计算器 IV](https://leetcode.cn/problems/basic-calculator-iv/) 2863
-   [439\. 三元表达式解析器](https://leetcode.cn/problems/ternary-expression-parser/)（会员题）
-   [772\. 基本计算器 III](https://leetcode.cn/problems/basic-calculator-iii/)（会员题）
-   [1087\. 花括号展开](https://leetcode.cn/problems/brace-expansion/)（会员题）
-   [1597\. 根据中缀表达式构造二叉表达式树](https://leetcode.cn/problems/build-binary-expression-tree-from-infix-expression/)（会员题）
-   [1628\. 设计带解析函数的表达式树](https://leetcode.cn/problems/design-an-expression-tree-with-evaluate-function/)（会员题）

### [](#§36-对顶栈)§3.6 对顶栈

-   [2296\. 设计一个文本编辑器](https://leetcode.cn/problems/design-a-text-editor/) 1912

### [](#§37-单调栈)§3.7 单调栈

见 [单调栈题单](https://leetcode.cn/circle/discuss/9oZFK9/)。

## [](#四、队列)四、队列

队列常用在 BFS 中，见 [网格图题单](https://leetcode.cn/circle/discuss/YiXPXW/) 和 [图论题单](https://leetcode.cn/circle/discuss/01LUak/)。与此相比，栈常用在 DFS 中，但无需我们手动维护。

### [](#§41-基础)§4.1 基础

-   [933\. 最近的请求次数](https://leetcode.cn/problems/number-of-recent-calls/) 1338
-   [950\. 按递增顺序显示卡牌](https://leetcode.cn/problems/reveal-cards-in-increasing-order/) 1686
-   [649\. Dota2 参议院](https://leetcode.cn/problems/dota2-senate/)
-   [346\. 数据流中的移动平均值](https://leetcode.cn/problems/moving-average-from-data-stream/)（会员题）
-   [362\. 敲击计数器](https://leetcode.cn/problems/design-hit-counter/)（会员题）
-   [379\. 电话目录管理系统](https://leetcode.cn/problems/design-phone-directory/)（会员题）
-   [1429\. 第一个唯一数字](https://leetcode.cn/problems/first-unique-number/)（会员题）
-   [2534\. 通过门的时间](https://leetcode.cn/problems/time-taken-to-cross-the-door/)（会员题）

### [](#§42-设计)§4.2 设计

-   [225\. 用队列实现栈](https://leetcode.cn/problems/implement-stack-using-queues/)
-   [232\. 用栈实现队列](https://leetcode.cn/problems/implement-queue-using-stacks/)
-   [622\. 设计循环队列](https://leetcode.cn/problems/design-circular-queue/)
-   [641\. 设计循环双端队列](https://leetcode.cn/problems/design-circular-deque/)
-   [1670\. 设计前中后队列](https://leetcode.cn/problems/design-front-middle-back-queue/) 1610

### [](#§43-单调队列)§4.3 单调队列

个人觉得叫**单调双端队列**更准确。

[原理讲解](https://leetcode.cn/link/?target=https://www.bilibili.com/video/BV1bM411X72E/)

-   [239\. 滑动窗口最大值](https://leetcode.cn/problems/sliding-window-maximum/)
-   [LCR 184. 设计自助结算系统](https://leetcode.cn/problems/dui-lie-de-zui-da-zhi-lcof/)
-   [1438\. 绝对差不超过限制的最长连续子数组](https://leetcode.cn/problems/longest-continuous-subarray-with-absolute-diff-less-than-or-equal-to-limit/) 1672
-   [2398\. 预算内的最多机器人数目](https://leetcode.cn/problems/maximum-number-of-robots-within-budget/) 1917
-   [862\. 和至少为 K 的最短子数组](https://leetcode.cn/problems/shortest-subarray-with-sum-at-least-k/) 2307
-   [1499\. 满足不等式的最大值](https://leetcode.cn/problems/max-value-of-equation/) 2456
-   [2071\. 你可以安排的最多任务数目](https://leetcode.cn/problems/maximum-number-of-tasks-you-can-assign/) 2648

### [](#§44-单调队列优化-dp)§4.4 单调队列优化 DP

一般用来维护一段转移来源的最值。

1.  前提：区间右端点变大时，左端点也在变大（同滑动窗口）。
2.  转移前，去掉队首无用数据。
3.  计算转移（直接从队首转移）。
4.  把数据（一般是 $f[i]$ ）插入队尾前，去掉队尾无用数据。

-   [2944\. 购买水果需要的最少金币数](https://leetcode.cn/problems/minimum-number-of-coins-for-fruits/) 1709 可以用单调队列优化到 $O(n)$
-   [1696\. 跳跃游戏 VI](https://leetcode.cn/problems/jump-game-vi/) 1954
-   [1425\. 带限制的子序列和](https://leetcode.cn/problems/constrained-subsequence-sum/) 2032
-   [375\. 猜数字大小 II](https://leetcode.cn/problems/guess-number-higher-or-lower-ii/) 可以用单调队列优化到 $O(n^2)$
-   [1687\. 从仓库到码头运输箱子](https://leetcode.cn/problems/delivering-boxes-from-storage-to-ports/) 2610
-   [3117\. 划分数组得到最小的值之和](https://leetcode.cn/problems/minimum-sum-of-values-by-dividing-array/) 2735
-   [2945\. 找到最大非递减数组的长度](https://leetcode.cn/problems/find-maximum-non-decreasing-array-length/) 2943
-   [2969\. 购买水果需要的最少金币数 II](https://leetcode.cn/problems/minimum-number-of-coins-for-fruits-ii/)（会员题）

## [](#五、堆（优先队列）)五、堆（优先队列）

### [](#§51-基础)§5.1 基础

-   [1046\. 最后一块石头的重量](https://leetcode.cn/problems/last-stone-weight/) 1173
-   [2558\. 从数量最多的堆取走礼物](https://leetcode.cn/problems/take-gifts-from-the-richest-pile/) 1277
-   [2336\. 无限集中的最小数字](https://leetcode.cn/problems/smallest-number-in-infinite-set/) 1375
-   [2530\. 执行 K 次操作后的最大分数](https://leetcode.cn/problems/maximal-score-after-applying-k-operations/) 1386
-   [3066\. 超过阈值的最少操作数 II](https://leetcode.cn/problems/minimum-operations-to-exceed-threshold-value-ii/) 1400
-   [1962\. 移除石子使总数最小](https://leetcode.cn/problems/remove-stones-to-minimize-the-total/) 1419
-   [1845\. 座位预约管理系统](https://leetcode.cn/problems/seat-reservation-manager/) 1429
-   [2208\. 将数组和减半的最少操作次数](https://leetcode.cn/problems/minimum-operations-to-halve-array-sum/) 1550
-   [2233\. K 次增加后的最大乘积](https://leetcode.cn/problems/maximum-product-after-k-increments/) 1686
-   [1942\. 最小未被占据椅子的编号](https://leetcode.cn/problems/the-number-of-the-smallest-unoccupied-chair/) 1695
-   [1801\. 积压订单中的订单总数](https://leetcode.cn/problems/number-of-orders-in-the-backlog/) 1711
-   [2406\. 将区间分为最少组数](https://leetcode.cn/problems/divide-intervals-into-minimum-number-of-groups/) 1713
-   [2462\. 雇佣 K 位工人的总代价](https://leetcode.cn/problems/total-cost-to-hire-k-workers/) 1764
-   [1834\. 单线程 CPU](https://leetcode.cn/problems/single-threaded-cpu/) 1798
-   [1792\. 最大平均通过率](https://leetcode.cn/problems/maximum-average-pass-ratio/) 1818
-   [2931\. 购买物品的最大开销](https://leetcode.cn/problems/maximum-spending-after-buying-items/) 1822
-   [1882\. 使用服务器处理任务](https://leetcode.cn/problems/process-tasks-using-servers/) 1979
-   [2402\. 会议室 III](https://leetcode.cn/problems/meeting-rooms-iii/) 2093
-   [253\. 会议室 II](https://leetcode.cn/problems/meeting-rooms-ii/)（会员题）
-   [1167\. 连接木棍的最低费用](https://leetcode.cn/problems/minimum-cost-to-connect-sticks/)（会员题）

### [](#§52-进阶)§5.2 进阶

-   [23\. 合并 K 个升序链表](https://leetcode.cn/problems/merge-k-sorted-lists/)
-   [355\. 设计推特](https://leetcode.cn/problems/design-twitter/)
-   [502\. IPO](https://leetcode.cn/problems/ipo/)
-   [1705\. 吃苹果的最大数目](https://leetcode.cn/problems/maximum-number-of-eaten-apples/) 1930
-   [778\. 水位上升的泳池中游泳](https://leetcode.cn/problems/swim-in-rising-water/)
-   [1631\. 最小体力消耗路径](https://leetcode.cn/problems/path-with-minimum-effort/) 1948
-   [1354\. 多次求和构造目标数组](https://leetcode.cn/problems/construct-target-array-with-multiple-sums/) 2015
-   [1353\. 最多可以参加的会议数目](https://leetcode.cn/problems/maximum-number-of-events-that-can-be-attended/) 2016
-   [1235\. 规划兼职工作](https://leetcode.cn/problems/maximum-profit-in-job-scheduling/) 2023 也可以 DP
-   [632\. 最小区间](https://leetcode.cn/problems/smallest-range-covering-elements-from-k-lists/)
-   [2542\. 最大子序列的分数](https://leetcode.cn/problems/maximum-subsequence-score/) 2056
-   [1383\. 最大的团队表现值](https://leetcode.cn/problems/maximum-performance-of-a-team/) 2091
-   [2503\. 矩阵查询可获得的最大分数](https://leetcode.cn/problems/maximum-number-of-points-from-grid-queries/) 2196
-   [2163\. 删除元素后和的最小差值](https://leetcode.cn/problems/minimum-difference-in-sums-after-removal-of-elements/) 2225
-   [857\. 雇佣 K 名工人的最低成本](https://leetcode.cn/problems/minimum-cost-to-hire-k-workers/) 2260
-   [1606\. 找到处理最多请求的服务器](https://leetcode.cn/problems/find-servers-that-handled-most-number-of-requests/) 2276
-   [1851\. 包含每个查询的最小区间](https://leetcode.cn/problems/minimum-interval-to-include-each-query/) 2286
-   [218\. 天际线问题](https://leetcode.cn/problems/the-skyline-problem/)
-   [407\. 接雨水 II](https://leetcode.cn/problems/trapping-rain-water-ii/)
-   [2940\. 找到 Alice 和 Bob 可以相遇的建筑](https://leetcode.cn/problems/find-building-where-alice-and-bob-can-meet/) 2327
-   [2589\. 完成所有任务的最少时间](https://leetcode.cn/problems/minimum-time-to-complete-all-tasks/) 2381 做法不止一种
-   [1675\. 数组的最小偏移量](https://leetcode.cn/problems/minimize-deviation-in-array/) 2533
-   [2617\. 网格图中最少访问的格子数](https://leetcode.cn/problems/minimum-number-of-visited-cells-in-a-grid/) 2582
-   [2532\. 过桥的时间](https://leetcode.cn/problems/time-to-cross-a-bridge/) 2589
-   [LCP 33. 蓄水](https://leetcode.cn/problems/o8SXZn/) 思考：更快的做法
-   [1199\. 建造街区的最短时间](https://leetcode.cn/problems/minimum-time-to-build-blocks/)（会员题）
-   [2263\. 数组变为有序的最小操作次数](https://leetcode.cn/problems/make-array-non-decreasing-or-non-increasing/)（会员题）

### [](#§53-重排yuan素)§5.3 重排元素

-   [984\. 不含 AAA 或 BBB 的字符串](https://leetcode.cn/problems/string-without-aaa-or-bbb/) 1474 不需要堆，选这题的目的是引入贪心思想
-   [767\. 重构字符串](https://leetcode.cn/problems/reorganize-string/) 1681
-   [1054\. 距离相等的条形码](https://leetcode.cn/problems/distant-barcodes/) 1702
-   [1953\. 你可以工作的最大周数](https://leetcode.cn/problems/maximum-number-of-weeks-for-which-you-can-work/) 1804
-   [1405\. 最长快乐字符串](https://leetcode.cn/problems/longest-happy-string/) 1821
-   [3081\. 替换字符串中的问号使分数最小](https://leetcode.cn/problems/replace-question-marks-in-string-to-minimize-its-value/) 1905
-   [621\. 任务调度器](https://leetcode.cn/problems/task-scheduler/)
-   [358\. K 距离间隔重排字符串](https://leetcode.cn/problems/rearrange-string-k-distance-apart/)（会员题）

### [](#§54-第-k-小/大)§5.4 第 K 小/大

部分题目也可以用二分解决。

-   [264\. 丑数 II](https://leetcode.cn/problems/ugly-number-ii/)
-   [378\. 有序矩阵中第 K 小的元素](https://leetcode.cn/problems/kth-smallest-element-in-a-sorted-matrix/)
-   [373\. 查找和最小的 K 对数字](https://leetcode.cn/problems/find-k-pairs-with-smallest-sums/)
-   [1439\. 有序矩阵中的第 k 个最小数组和](https://leetcode.cn/problems/find-the-kth-smallest-sum-of-a-matrix-with-sorted-rows/) 2134
-   [786\. 第 K 个最小的素数分数](https://leetcode.cn/problems/k-th-smallest-prime-fraction/) 2169
-   [2386\. 找出数组的第 K 大和](https://leetcode.cn/problems/find-the-k-sum-of-an-array/) 2648

### [](#§55-反悔堆)§5.5 反悔堆

基于堆的反悔贪心。

-   [LCP 30. 魔塔游戏](https://leetcode.cn/problems/p0NxJO/)
-   [1642\. 可以到达的最远建筑](https://leetcode.cn/problems/furthest-building-you-can-reach/) 1962
-   [630\. 课程表 III](https://leetcode.cn/problems/course-schedule-iii/)
-   [871\. 最低加油次数](https://leetcode.cn/problems/minimum-number-of-refueling-stops/) 2074
-   [2813\. 子序列最大优雅度](https://leetcode.cn/problems/maximum-elegance-of-a-k-length-subsequence/) 2582
-   [3049\. 标记所有下标的最早秒数 II](https://leetcode.cn/problems/earliest-second-to-mark-indices-ii/) 3111
-   [2599\. 使前缀和数组非负](https://leetcode.cn/problems/make-the-prefix-sum-non-negative/)（会员题）

### [](#§56-懒删除堆)§5.6 懒删除堆

-   [2349\. 设计数字容器系统](https://leetcode.cn/problems/design-a-number-container-system/) 1540
-   [2353\. 设计食物评分系统](https://leetcode.cn/problems/design-a-food-rating-system/) 1782
-   [3092\. 最高频率的 ID](https://leetcode.cn/problems/most-frequent-ids/) 1793
-   [2034\. 股票价格波动](https://leetcode.cn/problems/stock-price-fluctuation/) 1832
-   [1172\. 餐盘栈](https://leetcode.cn/problems/dinner-plate-stacks/) 2110

### [](#§57-对顶堆)§5.7 对顶堆

-   [2102\. 序列顺序查询](https://leetcode.cn/problems/sequentially-ordinal-rank-tracker/) 2159
-   [295\. 数据流的中位数](https://leetcode.cn/problems/find-median-from-data-stream/)
-   [480\. 滑动窗口中位数](https://leetcode.cn/problems/sliding-window-median/)
-   [703\. 数据流中的第 K 大元素](https://leetcode.cn/problems/kth-largest-element-in-a-stream/)
-   [3013\. 将数组分成最小总代价的子数组 II](https://leetcode.cn/problems/divide-an-array-into-subarrays-with-minimum-cost-ii/) 2540
-   [LCP 24. 数字游戏](https://leetcode.cn/problems/5TxKeK/)

另见 [图论题单](https://leetcode.cn/circle/discuss/01LUak/) 中的 Dijkstra 算法。

## [](#六、字典树（trie）)六、字典树（trie）

### [](#§61-基础)§6.1 基础

-   [208\. 实现 Trie (前缀树)](https://leetcode.cn/problems/implement-trie-prefix-tree/)
-   [211\. 添加与搜索单词 - 数据结构设计](https://leetcode.cn/problems/design-add-and-search-words-data-structure/)
-   [14\. 最长公共前缀](https://leetcode.cn/problems/longest-common-prefix/)
-   [648\. 单词替换](https://leetcode.cn/problems/replace-words/)
-   [677\. 键值映射](https://leetcode.cn/problems/map-sum-pairs/)
-   [720\. 词典中最长的单词](https://leetcode.cn/problems/longest-word-in-dictionary/)
-   [1268\. 搜索推荐系统](https://leetcode.cn/problems/search-suggestions-system/) 1573
-   [1233\. 删除子文件夹](https://leetcode.cn/problems/remove-sub-folders-from-the-filesystem/)
-   [820\. 单词的压缩编码](https://leetcode.cn/problems/short-encoding-of-words/) 1632
-   [2416\. 字符串的前缀分数和](https://leetcode.cn/problems/sum-of-prefix-scores-of-strings/) 1725
-   [1804\. 实现 Trie （前缀树） II](https://leetcode.cn/problems/implement-trie-ii-prefix-tree/)（会员题）

### [](#§62-进阶)§6.2 进阶

-   [676\. 实现一个魔法字典](https://leetcode.cn/problems/implement-magic-dictionary/)
-   [212\. 单词搜索 II](https://leetcode.cn/problems/word-search-ii/)
-   [336\. 回文对](https://leetcode.cn/problems/palindrome-pairs/)
-   [745\. 前缀和后缀搜索](https://leetcode.cn/problems/prefix-and-suffix-search/)
-   [3093\. 最长公共后缀查询](https://leetcode.cn/problems/longest-common-suffix-queries/) 2118
-   [3045\. 统计前后缀下标对 II](https://leetcode.cn/problems/count-prefix-and-suffix-pairs-ii/) 2328
-   [1948\. 删除系统中的重复文件夹](https://leetcode.cn/problems/delete-duplicate-folders-in-system/) 2534
-   [425\. 单词方块](https://leetcode.cn/problems/word-squares/)（会员题）
-   [527\. 单词缩写](https://leetcode.cn/problems/word-abbreviation/)（会员题）
-   [588\. 设计内存文件系统](https://leetcode.cn/problems/design-in-memory-file-system/)（会员题）
-   [616\. 给字符串添加加粗标签](https://leetcode.cn/problems/add-bold-tag-in-string/)（会员题）
-   [758\. 字符串中的加粗单词](https://leetcode.cn/problems/bold-words-in-string/)（会员题）
-   [642\. 设计搜索自动补全系统](https://leetcode.cn/problems/design-search-autocomplete-system/)（会员题）
-   [1065\. 字符串的索引对](https://leetcode.cn/problems/index-pairs-of-a-string/)（会员题）
-   [1166\. 设计文件系统](https://leetcode.cn/problems/design-file-system/)（会员题）
-   [1858\. 包含所有前缀的最长单词](https://leetcode.cn/problems/longest-word-with-all-prefixes/)（会员题）

### [](#§63-字典树优化-dp)§6.3 字典树优化 DP

-   [139\. 单词拆分](https://leetcode.cn/problems/word-break/)
-   [140\. 单词拆分 II](https://leetcode.cn/problems/word-break-ii/)
-   [472\. 连接词](https://leetcode.cn/problems/concatenated-words/) ~2300
-   [2977\. 转换字符串的最小成本 II](https://leetcode.cn/problems/minimum-cost-to-convert-string-ii/) 2696

### [](#§64-01-字典树（异或字典树）)§6.4 0-1 字典树（异或字典树）

部分题目也可以用试填法解决。

-   [421\. 数组中两个数的最大异或值](https://leetcode.cn/problems/maximum-xor-of-two-numbers-in-an-array/) ~2000
-   [2935\. 找出强数对的最大异或值 II](https://leetcode.cn/problems/maximum-strong-pair-xor-ii/) 2349
-   [1707\. 与数组中元素的最大异或值](https://leetcode.cn/problems/maximum-xor-with-an-element-from-array/) 2359
-   [1803\. 统计异或值在范围内的数对有多少](https://leetcode.cn/problems/count-pairs-with-xor-in-a-range/) 2479
-   [1938\. 查询最大基因差](https://leetcode.cn/problems/maximum-genetic-difference-query/) 2503
-   [2479\. 两个不重叠子树的最大异或值](https://leetcode.cn/problems/maximum-xor-of-two-non-overlapping-subtrees/)（会员题）

## [](#七、并查集)七、并查集

### [](#§71-基础)§7.1 基础

见 [网格图题单](https://leetcode.cn/circle/discuss/YiXPXW/) 中的 DFS 和 [图论题单](https://leetcode.cn/circle/discuss/01LUak/) 中的 DFS，其中大部分题目可以用并查集实现。

-   [990\. 等式方程的可满足性](https://leetcode.cn/problems/satisfiability-of-equality-equations/) 1638
-   [721\. 账户合并](https://leetcode.cn/problems/accounts-merge/)
-   [737\. 句子相似性 II](https://leetcode.cn/problems/sentence-similarity-ii/)（会员题）
-   [1101\. 彼此熟识的最早时间](https://leetcode.cn/problems/the-earliest-moment-when-everyone-become-friends/)（会员题）
-   [1258\. 近义词句子](https://leetcode.cn/problems/synonymous-sentences/)（会员题）

### [](#§72-进阶)§7.2 进阶

-   [1202\. 交换字符串中的元素](https://leetcode.cn/problems/smallest-string-with-swaps/) 1855
-   [1061\. 按字典序排列最小的等效字符串](https://leetcode.cn/problems/lexicographically-smallest-equivalent-string/)
-   [1722\. 执行交换操作后的最小汉明距离](https://leetcode.cn/problems/minimize-hamming-distance-after-swap-operations/) 1892
-   [765\. 情侣牵手](https://leetcode.cn/problems/couples-holding-hands/) 1999
-   [947\. 移除最多的同行或同列石头](https://leetcode.cn/problems/most-stones-removed-with-same-row-or-column/) 2035
-   [839\. 相似字符串组](https://leetcode.cn/problems/similar-string-groups/) 2054
-   [1970\. 你能穿过矩阵的最后一天](https://leetcode.cn/problems/last-day-where-you-can-still-cross/) 2124
-   [2076\. 处理含限制条件的好友请求](https://leetcode.cn/problems/process-restricted-friend-requests/) 2131
-   [1579\. 保证图可完全遍历](https://leetcode.cn/problems/remove-max-number-of-edges-to-keep-graph-fully-traversable/) 2132
-   [959\. 由斜杠划分区域](https://leetcode.cn/problems/regions-cut-by-slashes/) 2136
-   [2812\. 找出最安全路径](https://leetcode.cn/problems/find-the-safest-path-in-a-grid/) 2154
-   [2503\. 矩阵查询可获得的最大分数](https://leetcode.cn/problems/maximum-number-of-points-from-grid-queries/) 2196
-   [2867\. 统计树中的合法路径数目](https://leetcode.cn/problems/count-valid-paths-in-a-tree/) 2428
-   [2421\. 好路径的数目](https://leetcode.cn/problems/number-of-good-paths/) 2445
-   [2157\. 字符串分组](https://leetcode.cn/problems/groups-of-strings/) 2499
-   [1632\. 矩阵转换后的秩](https://leetcode.cn/problems/rank-transform-of-a-matrix/) 2530
-   [803\. 打砖块](https://leetcode.cn/problems/bricks-falling-when-hit/) 2765
-   [1569\. 将子数组重新排序得到同一个二叉搜索树的方案数](https://leetcode.cn/problems/number-of-ways-to-reorder-array-to-get-same-bst/) 思考：更快的做法
-   [LCP 71. 集水器](https://leetcode.cn/problems/kskhHQ/)
-   [2371\. 最小化网格中的最大值](https://leetcode.cn/problems/minimize-maximum-value-in-a-grid/)（会员题）同 1632 题

### [](#§73-公因数并查集)§7.3 公因数并查集

-   [2709\. 最大公约数遍历](https://leetcode.cn/problems/greatest-common-divisor-traversal/) 2172
-   [1627\. 带阈值的图连通性](https://leetcode.cn/problems/graph-connectivity-with-threshold/) 2221
-   [952\. 按公因数计算最大组件大小](https://leetcode.cn/problems/largest-component-size-by-common-factor/) 2272
-   [1998\. 数组的最大公因数排序](https://leetcode.cn/problems/gcd-sort-of-an-array/) 2429

### [](#§74-数组上的并查集)§7.4 数组上的并查集

-   [1562\. 查找大小为 M 的最新分组](https://leetcode.cn/problems/find-latest-group-of-size-m/) 1928
-   [1488\. 避免洪水泛滥](https://leetcode.cn/problems/avoid-flood-in-the-city/) 1974
-   [2382\. 删除操作后的最大子段和](https://leetcode.cn/problems/maximum-segment-sum-after-removals/) 2136
-   [2334\. 元素值大于变化阈值的子数组](https://leetcode.cn/problems/subarray-with-elements-greater-than-varying-threshold/) 2381

### [](#§75-区间并查集)§7.5 区间并查集

-   [1851\. 包含每个查询的最小区间](https://leetcode.cn/problems/minimum-interval-to-include-each-query/) 2286
-   [2158\. 每天绘制新区域的数量](https://leetcode.cn/problems/amount-of-new-area-painted-each-day/)（会员题）

### [](#§76-边权并查集)§7.6 边权并查集

-   [399\. 除法求值](https://leetcode.cn/problems/evaluate-division/)
-   [2307\. 检查方程中的矛盾之处](https://leetcode.cn/problems/check-for-contradictions-in-equations/)（会员题）

## [](#八、树状数组和线段树)八、树状数组和线段树

能用树状数组解决的题目，也能用线段树解决（反过来不一定）。但树状数组实现简单，代码短。

为方便大家练习，我把适合用树状数组解决的题目分到树状数组中，其余分到线段树中。

### [](#§81-树状数组)§8.1 树状数组

讲解：[带你发明树状数组！附数学证明](https://leetcode.cn/problems/range-sum-query-mutable/solution/dai-ni-fa-ming-shu-zhuang-shu-zu-fu-shu-lyfll/)

-   [307\. 区域和检索 - 数组可修改](https://leetcode.cn/problems/range-sum-query-mutable/)
-   [3072\. 将元素分配到两个数组中 II](https://leetcode.cn/problems/distribute-elements-into-two-arrays-ii/) 2053
-   [1649\. 通过指令创建有序数组](https://leetcode.cn/problems/create-sorted-array-through-instructions/) 2208
-   [1626\. 无矛盾的最佳球队](https://leetcode.cn/problems/best-team-with-no-conflicts/)
-   [1409\. 查询带键的排列](https://leetcode.cn/problems/queries-on-a-permutation-with-key/)
-   [2250\. 统计包含每个点的矩形数目](https://leetcode.cn/problems/count-number-of-rectangles-containing-each-point/)
-   [2179\. 统计数组中好三元组数目](https://leetcode.cn/problems/count-good-triplets-in-an-array/) 2272
-   [1395\. 统计作战单位数](https://leetcode.cn/problems/count-number-of-teams/)
-   [2659\. 将数组清空](https://leetcode.cn/problems/make-array-empty/) 2282
-   [2653\. 滑动子数组的美丽值](https://leetcode.cn/problems/sliding-subarray-beauty/) 树状数组二分
-   [1505\. 最多 K 次交换相邻数位后得到的最小整数](https://leetcode.cn/problems/minimum-possible-integer-after-at-most-k-adjacent-swaps-on-digits/) 2337
-   [2926\. 平衡子序列的最大和](https://leetcode.cn/problems/maximum-balanced-subsequence-sum/) 2448
-   [2736\. 最大和查询](https://leetcode.cn/problems/maximum-sum-queries/) 2533
-   [2519\. 统计 K-Big 索引的数量](https://leetcode.cn/problems/count-the-number-of-k-big-indices/)（会员题）
-   [2613\. 美数对](https://leetcode.cn/problems/beautiful-pairs/)（会员题）曼哈顿最近点对
-   [2921\. 价格递增的最大利润三元组 II](https://leetcode.cn/problems/maximum-profitable-triplets-with-increasing-prices-ii/)（会员题）
-   [308\. 二维区域和检索 - 可变](https://leetcode.cn/problems/range-sum-query-2d-mutable/)（会员题）

### [](#§82-逆序对)§8.2 逆序对

也可以在**归并排序**的同时计算。

-   [LCR 170. 交易逆序对的总数](https://leetcode.cn/problems/shu-zu-zhong-de-ni-xu-dui-lcof/)
-   [315\. 计算右侧小于当前元素的个数](https://leetcode.cn/problems/count-of-smaller-numbers-after-self/)
-   [493\. 翻转对](https://leetcode.cn/problems/reverse-pairs/)
-   [327\. 区间和的个数](https://leetcode.cn/problems/count-of-range-sum/)
-   [2426\. 满足不等式的数对数目](https://leetcode.cn/problems/number-of-pairs-satisfying-inequality/) 2030
-   [1850\. 邻位交换的最小次数](https://leetcode.cn/problems/minimum-adjacent-swaps-to-reach-the-kth-smallest-number/)
-   [2193\. 得到回文串的最少操作次数](https://leetcode.cn/problems/minimum-number-of-moves-to-make-palindrome/)
-   [1885\. 统计数对](https://leetcode.cn/problems/count-pairs-in-two-arrays/)（会员题）

### [](#§83-线段树（无区间更新）)§8.3 线段树（无区间更新）

-   [1157\. 子数组中占绝大多数的元素](https://leetcode.cn/problems/online-majority-element-in-subarray/) 2205
-   [2407\. 最长递增子序列 II](https://leetcode.cn/problems/longest-increasing-subsequence-ii/) 2280
-   [2940\. 找到 Alice 和 Bob 可以相遇的建筑](https://leetcode.cn/problems/find-building-where-alice-and-bob-can-meet/) 2327 线段树二分
-   [2286\. 以组为单位订音乐会的门票](https://leetcode.cn/problems/booking-concert-tickets-in-groups/) 2470 线段树二分
-   [2213\. 由单个字符重复的最长子字符串](https://leetcode.cn/problems/longest-substring-of-one-repeating-character/) 2629

### [](#§84-lazy-线段树（有区间更新）)§8.4 Lazy 线段树（有区间更新）

-   [2589\. 完成所有任务的最少时间](https://leetcode.cn/problems/minimum-time-to-complete-all-tasks/) 2381
-   [2569\. 更新数组后处理求和查询](https://leetcode.cn/problems/handling-sum-queries-after-update/) 2398
-   [1622\. 奇妙序列](https://leetcode.cn/problems/fancy-sequence/) 2476 有更简单的做法
-   [2916\. 子数组不同元素数目的平方和 II](https://leetcode.cn/problems/subarrays-distinct-element-sum-of-squares-ii/) 2816
-   [LCP 05. 发 LeetCoin](https://leetcode.cn/problems/coin-bonus/)
-   [LCP 27. 黑盒光线反射](https://leetcode.cn/problems/IQvJ9i/)

### [](#§85-动态开点线段树)§8.5 动态开点线段树

部分题目也可以用**珂朵莉树**解决。

-   [699\. 掉落的方块](https://leetcode.cn/problems/falling-squares/)
-   [715\. Range 模块](https://leetcode.cn/problems/range-module/)
-   [729\. 我的日程安排表 I](https://leetcode.cn/problems/my-calendar-i/)
-   [731\. 我的日程安排表 II](https://leetcode.cn/problems/my-calendar-ii/)
-   [732\. 我的日程安排表 III](https://leetcode.cn/problems/my-calendar-iii/)
-   [850\. 矩形面积 II](https://leetcode.cn/problems/rectangle-area-ii/)
-   [2276\. 统计区间中的整数数目](https://leetcode.cn/problems/count-integers-in-intervals/) 2222
-   [2770\. 达到末尾下标所需的最大跳跃次数](https://leetcode.cn/problems/maximum-number-of-jumps-to-reach-the-last-index/)

## [](#专题：离线算法)专题：离线算法

对询问**排序**，通过改变回答询问的顺序，使问题更容易处理。

> 相应的，在线算法就是按照输入的顺序处理，来一个处理一个。

-   [2343\. 裁剪数字后查询第 K 小的数字](https://leetcode.cn/problems/query-kth-smallest-trimmed-number/) 1652
-   [2070\. 每一个查询的最大美丽值](https://leetcode.cn/problems/most-beautiful-item-for-each-query/) 1724
-   [1847\. 最近的房间](https://leetcode.cn/problems/closest-room/) 2082
-   [2503\. 矩阵查询可获得的最大分数](https://leetcode.cn/problems/maximum-number-of-points-from-grid-queries/) 2196
-   [1851\. 包含每个查询的最小区间](https://leetcode.cn/problems/minimum-interval-to-include-each-query/) 2286
-   [1697\. 检查边长度限制的路径是否存在](https://leetcode.cn/problems/checking-existence-of-edge-length-limited-paths/) 2300
-   [2940\. 找到 Alice 和 Bob 可以相遇的建筑](https://leetcode.cn/problems/find-building-where-alice-and-bob-can-meet/) 2327
-   [2747\. 统计没有收到请求的服务器数目](https://leetcode.cn/problems/count-zero-request-servers/) 2405
-   [1938\. 查询最大基因差](https://leetcode.cn/problems/maximum-genetic-difference-query/) 2503
-   [2736\. 最大和查询](https://leetcode.cn/problems/maximum-sum-queries/) 2533