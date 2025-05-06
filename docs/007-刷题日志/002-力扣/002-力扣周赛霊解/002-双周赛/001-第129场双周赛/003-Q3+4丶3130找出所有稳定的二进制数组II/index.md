# 3129. 找出所有稳定的二进制数组 I
链接: [3129. 找出所有稳定的二进制数组 I](https://leetcode.cn/problems/find-all-possible-stable-binary-arrays-i/)

数据范围: `1 <= zero, one, limit <= 200`

# 3130. 找出所有稳定的二进制数组 II

链接: [3130. 找出所有稳定的二进制数组 II](https://leetcode.cn/problems/find-all-possible-stable-binary-arrays-ii/)

给你 3 个正整数 zero ，one 和 limit 。

一个 二进制数组 arr 如果满足以下条件，那么我们称它是 稳定的 ：

- 0 在 arr 中出现次数 恰好 为 zero 。
- 1 在 arr 中出现次数 恰好 为 one 。
- arr 中每个长度超过 limit 的 子数组 都 同时 包含 0 和 1 。

请你返回 稳定 二进制数组的 总 数目。

由于答案可能很大，将它对 $10^9 + 7$ 取余 后返回。

数据范围: `1 <= zero, one, limit <= 1000`

# 题解
> 0x3f: [两种方法：动态规划 / 组合数学（Python/Java/C++/Go）](https://leetcode.cn/problems/find-all-possible-stable-binary-arrays-ii/solutions/2758868/dong-tai-gui-hua-cong-ji-yi-hua-sou-suo-37jdi)
>
> 这个还不是我可以学的qwq..

## 记忆化 / dp

## 组合数学
我也想到这里, 但是已经忘光光了(高中的使用隔板法解决的问题, 然后减去不满足的情况)(但是我不会)