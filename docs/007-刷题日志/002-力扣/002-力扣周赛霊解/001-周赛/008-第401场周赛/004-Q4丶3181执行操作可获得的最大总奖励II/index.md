# 3181. 执行操作可获得的最大总奖励 II
链接: [3181. 执行操作可获得的最大总奖励 II](https://leetcode.cn/problems/maximum-total-reward-using-operations-ii/)

给你一个整数数组 rewardValues，长度为 n，代表奖励的值。

最初，你的总奖励 x 为 0，所有下标都是 未标记 的。你可以执行以下操作 任意次 ：

- 从区间 [0, n - 1] 中选择一个 未标记 的下标 i。

- 如果 rewardValues[i] 大于 你当前的总奖励 x，则将 rewardValues[i] 加到 x 上（即 x = x + rewardValues[i]），并 标记 下标 i。

以整数形式返回执行最优操作能够获得的 最大 总奖励。

提示:
- 1 <= rewardValues.length <= 5 * 10^4
- 1 <= rewardValues[i] <= 5 * 10^4

# 题解
## bitset/bigint 优化 0-1 背包 + 两数之和优化
- [bitset/bigint 优化 0-1 背包 + 两数之和优化（Python/Java/C++/Go）](https://leetcode.cn/problems/maximum-total-reward-using-operations-ii/solutions/2805413/bitset-you-hua-0-1-bei-bao-by-endlessche-m1xn)

- 01背包 - 使用二进制代表: 是否可以选择从而得到值 $x$, 可以则 第 $x$ 位二进制为 1, 否则 0. (具体看视频qwq)

```C++
class Solution {
public:
    int maxTotalReward(vector<int>& rewardValues) {
        ranges::sort(rewardValues);
        rewardValues.erase(unique(rewardValues.begin(), rewardValues.end()), rewardValues.end());

        bitset<100000> f{1};
        for (int v : rewardValues) {
            int shift = f.size() - v;
            // 左移 shift 再右移 shift，把所有 >= v 的比特位置 0
            // f |= f << shift >> shift << v;
            f |= f << shift >> (shift - v); // 简化上式
        }
        for (int i = rewardValues.back() * 2 - 1; ; i--) {
            if (f.test(i)) {
                return i;
            }
        }
    }
};
```
