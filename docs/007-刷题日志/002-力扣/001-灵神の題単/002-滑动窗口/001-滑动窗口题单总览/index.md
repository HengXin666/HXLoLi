# [分享丨【题单】滑动窗口（定长/不定长/多指针）](https://leetcode.cn/circle/discuss/0viNMK/)

> [!TIP]
> 题单已老, 请点击链接, 查看最新版!


> By <span style="color:rgb(255, 161, 22)">灵茶山艾府</span>

右边数字为难度分。

### 定长滑动窗口

- [x] [1456. 定长子串中元音的最大数目](https://leetcode.cn/problems/maximum-number-of-vowels-in-a-substring-of-given-length/) 1263
- [x] [2269. 找到一个数字的 K 美丽值](https://leetcode.cn/problems/find-the-k-beauty-of-a-number/) 1280
- [x] [1984. 学生分数的最小差值](https://leetcode.cn/problems/minimum-difference-between-highest-and-lowest-of-k-scores/) 1306
- [x] [643. 子数组最大平均数 I](https://leetcode.cn/problems/maximum-average-subarray-i/)
- [x] [1343. 大小为 K 且平均值大于等于阈值的子数组数目](https://leetcode.cn/problems/number-of-sub-arrays-of-size-k-and-average-greater-than-or-equal-to-threshold/) 1317
- [x] [2090. 半径为 k 的子数组平均值](https://leetcode.cn/problems/k-radius-subarray-averages/) 1358
- [x] [2379. 得到 K 个黑块的最少涂色次数](https://leetcode.cn/problems/minimum-recolors-to-get-k-consecutive-black-blocks/) 1360
- [x] [1052. 爱生气的书店老板](https://leetcode.cn/problems/grumpy-bookstore-owner/) 1418
- [x] [2841. 几乎唯一子数组的最大和](https://leetcode.cn/problems/maximum-sum-of-almost-unique-subarray/) 1546
- [x] [2461. 长度为 K 子数组中的最大和](https://leetcode.cn/problems/maximum-sum-of-distinct-subarrays-with-length-k/) 1553 (哈希表 + 滑窗)
- [x] [1423. 可获得的最大点数](https://leetcode.cn/problems/maximum-points-you-can-obtain-from-cards/) 1574
- [x] [2134. 最少交换次数来组合所有的 1 II](https://leetcode.cn/problems/minimum-swaps-to-group-all-1s-together-ii/) 1748
- [x] [2653. 滑动子数组的美丽值](https://leetcode.cn/problems/sliding-subarray-beauty/) 1786 (计数排序(暴力) (数据范围很大应该是 懒删除大小堆 为正解...))
- [x] [567. 字符串的排列](https://leetcode.cn/problems/permutation-in-string/)
- [x] [438. 找到字符串中所有字母异位词](https://leetcode.cn/problems/find-all-anagrams-in-a-string/)
- [x] [2156. 查找给定哈希值的子串](https://leetcode.cn/problems/find-substring-with-given-hash-value/) 2063 (数学)
-   [2953\. 统计完全子字符串](https://leetcode.cn/problems/count-complete-substrings/) 2449
-   [346\. 数据流中的移动平均值](https://leetcode.cn/problems/moving-average-from-data-stream/)（会员题）
-   [1100\. 长度为 K 的无重复字符子串](https://leetcode.cn/problems/find-k-length-substrings-with-no-repeated-characters/)（会员题）
-   [1852\. 每个子数组的数字种类数](https://leetcode.cn/problems/distinct-numbers-in-each-subarray/)（会员题）
-   [2067\. 等计数子串的数量](https://leetcode.cn/problems/number-of-equal-count-substrings/)（会员题）
-   [2107\. 分享 K 个糖果后独特口味的数量](https://leetcode.cn/problems/number-of-unique-flavors-after-sharing-k-candies/)（会员题）

### 不定长滑动窗口（求最长/最大）

- [x] [3. 无重复字符的最长子串](https://leetcode.cn/problems/longest-substring-without-repeating-characters/)
- [x] [1493. 删掉一个元素以后全为 1 的最长子数组](https://leetcode.cn/problems/longest-subarray-of-1s-after-deleting-one-element/) 1423
- [x] [2730. 找到最长的半重复子字符串](https://leetcode.cn/problems/find-the-longest-semi-repetitive-substring/) 1502 (语文: 至多一个 <=> 可能为0个)
- [x] [904. 水果成篮](https://leetcode.cn/problems/fruit-into-baskets/) 1516
- [x] [1695. 删除子数组的最大得分](https://leetcode.cn/problems/maximum-erasure-value/) 1529
- [x] [2958. 最多 K 个重复元素的最长子数组](https://leetcode.cn/problems/length-of-longest-subarray-with-at-most-k-frequency/) 1535
- [x] [2024. 考试的最大困扰度](https://leetcode.cn/problems/maximize-the-confusion-of-an-exam/) 1643 (窗口内出现次数最少的`T`/`F`的次数需要小于k)
- [x] [1004. 最大连续1的个数 III](https://leetcode.cn/problems/max-consecutive-ones-iii/) 1656
- [x] [1438. 绝对差不超过限制的最长连续子数组](https://leetcode.cn/problems/longest-continuous-subarray-with-absolute-diff-less-than-or-equal-to-limit/) 1672
- [x] [2401. 最长优雅子数组](https://leetcode.cn/problems/longest-nice-subarray/) 1750
- [x] [1658. 将 x 减到 0 的最小操作数](https://leetcode.cn/problems/minimum-operations-to-reduce-x-to-zero/) 1817 (正难则反)
- [x] [1838. 最高频元素的频数](https://leetcode.cn/problems/frequency-of-the-most-frequent-element/) 1876 (排序 + 滑窗)
- [x] [2516. 每种字符至少取 K 个](https://leetcode.cn/problems/take-k-of-each-character-from-left-and-right/) 1948 (问两端搞中间)
- [x] [2831. 找出最长等值子数组](https://leetcode.cn/problems/find-the-longest-equal-subarray/) 1976 (分组循环)
- [x] [2106. 摘水果](https://leetcode.cn/problems/maximum-fruits-harvested-after-at-most-k-steps/) 2062 (左到右 != 右到左)
-   [1610\. 可见点的最大数目](https://leetcode.cn/problems/maximum-number-of-visible-points/) 2147
-   [2781\. 最长合法子字符串的长度](https://leetcode.cn/problems/length-of-the-longest-valid-substring/) 2204
-   [2968\. 执行操作使频率分数最大](https://leetcode.cn/problems/apply-operations-to-maximize-frequency-score/) 2444
-   [395\. 至少有 K 个重复字符的最长子串](https://leetcode.cn/problems/longest-substring-with-at-least-k-repeating-characters/)
-   [1763\. 最长的美好子字符串](https://leetcode.cn/problems/longest-nice-substring/)
-   [159\. 至多包含两个不同字符的最长子串](https://leetcode.cn/problems/longest-substring-with-at-most-two-distinct-characters/)（会员题）
-   [340\. 至多包含 K 个不同字符的最长子串](https://leetcode.cn/problems/longest-substring-with-at-most-k-distinct-characters/)（会员题）

### 不定长滑动窗口（求最短/最小）

- [x] [209. 长度最小的子数组](https://leetcode.cn/problems/minimum-size-subarray-sum/)
- [x] [1234. 替换子串得到平衡字符串](https://leetcode.cn/problems/replace-the-substring-for-balanced-string/) 1878 `(难)`
- [x] [1574. 删除最短的子数组使剩余数组有序](https://leetcode.cn/problems/shortest-subarray-to-be-removed-to-make-array-sorted/) 1932 `(难)` (不太会这种: 移除子数组)
-   [76\. 最小覆盖子串](https://leetcode.cn/problems/minimum-window-substring/)
-   [面试题 17.18. 最短超串](https://leetcode.cn/problems/shortest-supersequence-lcci/)

### 不定长滑动窗口（求子数组个数）

- [x] [2799. 统计完全子数组的数目](https://leetcode.cn/problems/count-complete-subarrays-in-an-array/) 1398 (子数组数量怎么算qwq?!)
- [x] [713. 乘积小于 K 的子数组](https://leetcode.cn/problems/subarray-product-less-than-k/)
- [x] [1358. 包含所有三种字符的子字符串数目](../002-包含所有三种字符的子字符串数目/index.md) 1646 (子数组数量怎么算qwq?! 这里你就理解了)
- [x] [2962. 统计最大元素出现至少 K 次的子数组](https://leetcode.cn/problems/count-subarrays-where-max-element-appears-at-least-k-times/) 1701 (同上的子数组计算方法)
- [x] [2302. 统计得分小于 K 的子数组数目](https://leetcode.cn/problems/count-subarrays-with-score-less-than-k/) 1808 (同上的子数组计算方法, 但有不同)
- [x] [2537. 统计好子数组的数目](https://leetcode.cn/problems/count-the-number-of-good-subarrays/) 1892 (同上的子数组计算方法, 但是使用哈希统计`arr[i] == arr[j] (i < j)`的`对`数)
- [x] [2762. 不间断子数组](https://leetcode.cn/problems/continuous-subarrays/) 1940 (有序列表)
- [x] [2972. 统计移除递增子数组的数目 II](https://leetcode.cn/problems/count-the-number-of-incremovable-subarrays-ii/) 2153 (不太会这种: 移除子数组)
-   [2743\. 计算没有重复字符的子字符串数量](https://leetcode.cn/problems/count-substrings-without-repeating-character/)（会员题）

### 多指针滑动窗口

计算可排除前后也可作为子数组的, 问子数组数目, 模版:

0, 1 数组, 问和为`goal`的子数组数目. 因为可能存在前后缀 [0,0,0,1,0,0], 0 可以使得窗口移动, 因此我们使用两个指针l1, l2作为前面, (l1)一个负责`goal`窗口, (l2)一个负责`goal - 1`窗口, 那么就可以得到排除的`0`的部分, 最后`res += l2 - l1`即可.

```C++
class Solution { // 930. 和相同的二元子数组 code
public:
    int numSubarraysWithSum(vector<int>& nums, int goal) {
        int n = nums.size(), res = 0;
        for (int i = 0, l1 = 0, l2 = 0, sum1 = 0, sum2 = 0; i < n; ++i) {
            sum1 += nums[i];
            while (l1 <= i && sum1 > goal)
                sum1 -= nums[l1++];

            sum2 += nums[i];
            while (l2 <= i && sum2 >= goal)
                sum2 -= nums[l2++];

            res += l2 - l1;
        }
        return res;
    }
};
```

- [x] [930. 和相同的二元子数组](https://leetcode.cn/problems/binary-subarrays-with-sum/) 1592
- [x] [1248. 统计「优美子数组」](https://leetcode.cn/problems/count-number-of-nice-subarrays/) 1624
- [x] [2563. 统计公平数对的数目](https://leetcode.cn/problems/count-the-number-of-fair-pairs/) 1721 (难, 得从后面写起 | 二分)
- [x] [1712. 将数组分成三个子数组的方案数](https://leetcode.cn/problems/ways-to-split-array-into-three-subarrays/) 2079 (二分)
-   [2444\. 统计定界子数组的数目](https://leetcode.cn/problems/count-subarrays-with-fixed-bounds/) 2093
-   [992\. K 个不同整数的子数组](https://leetcode.cn/problems/subarrays-with-k-different-integers/) 2210
-   [1989\. 捉迷藏中可捕获的最大人数](https://leetcode.cn/problems/maximum-number-of-people-that-can-be-caught-in-tag/)（会员题）