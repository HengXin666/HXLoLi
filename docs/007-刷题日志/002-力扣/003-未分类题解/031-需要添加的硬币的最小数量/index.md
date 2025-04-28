# 2952. 需要添加的硬币的最小数量
链接: [2952. 需要添加的硬币的最小数量](https://leetcode.cn/problems/minimum-number-of-coins-to-be-added/description/)

第 374 场周赛 Q2 1784

给你一个下标从 0 开始的整数数组 coins，表示可用的硬币的面值，以及一个整数 target 。

如果存在某个 coins 的子序列总和为 x，那么整数 x 就是一个 可取得的金额 。

返回需要添加到数组中的 任意面值 硬币的 最小数量 ，使范围 [1, target] 内的每个整数都属于 可取得的金额 。

数组的 子序列 是通过删除原始数组的一些（可能不删除）元素而形成的新的 非空 数组，删除过程不会改变剩余元素的相对位置。

```
示例 1：

输入：coins = [1,4,10], target = 19
输出：2
解释：需要添加面值为 2 和 8 的硬币各一枚，得到硬币数组 [1,2,4,8,10] 。
可以证明从 1 到 19 的所有整数都可由数组中的硬币组合得到，且需要添加到数组中的硬币数目最小为 2 。
示例 2：

输入：coins = [1,4,10,5,7,19], target = 19
输出：1
解释：只需要添加一枚面值为 2 的硬币，得到硬币数组 [1,2,4,5,7,10,19] 。
可以证明从 1 到 19 的所有整数都可由数组中的硬币组合得到，且需要添加到数组中的硬币数目最小为 1 。
示例 3：

输入：coins = [1,1,1], target = 20
输出：3
解释：
需要添加面值为 4 、8 和 16 的硬币各一枚，得到硬币数组 [1,1,1,4,8,16] 。 
可以证明从 1 到 20 的所有整数都可由数组中的硬币组合得到，且需要添加到数组中的硬币数目最小为 3 。
```

提示: 

$ 1 <= target <= 10^5 \\ 1 <= coins.length <= 10^5 \\ 1 <= coins[i] <= target$

# 题解

学习: [滑动窗口 组合数学【力扣周赛 374】](https://www.bilibili.com/video/BV1og4y1Z7SZ/)

```C++
class Solution {
public:
    int minimumAddedCoins(vector<int>& coins, int target) {
/*
[0, s - 1] 添加一个数 c[i] (c从小到大排序)
为 [c[i], s + c[i] - 1], 
如果 c[i] <= s, 那么说明可以合并出区间 [0, s + c[i] - 1]
如果 c[i] > s, 那么说明 可以得到区间 [0, s - 1] 和 [c[i], s + c[i] - 1]
但是永远缺少了, (s - 1, c[i]) 的数, 因为 + c[i + k] >= c[i]
所以需要添加 (s - 1, c[i]) 的数 即 [s, c[i] - 1] 需要添加, 
那么添加的个数就是 ((c[i] - 1) - s) + 1

r = s - 1 => s = r + 1
c[i] > r + 1

((c[i] - 1) - (r + 1)) + 1
c[i] - 1 - r
*/
        sort(coins.begin(), coins.end());
        int l = 0;          // 左区间
        int r = 1;          // 右区间
        int res = 0;        // 添加数的个数
        int i = 0;

        while (r <= target) {
            if (i < coins.size() && coins[i] <= r) {
                r += coins[i++];
            } else {
                ++res;
                r *= 2; // 尝试全部 + r
            }
        }
        return res;
    }
};

/*
[1, t] 的每个整数都可以 从 coins 中的子序列和表示出来

添加的都是二进制数(1 << x)?

枚举出所有可能的和 ? 然后再枚举二进制的 ? len ~ 10^5 不行

动态规划吗? 01背包 ? 如果 v[i] != i 那么需要点手段?

n 或者 nlogn

二进制拆分后, 所有的 |

0001 - 1
0100 - 4
1010 - 10
0010 - 2  +
0100 - 8  +

00010 - 2 +
00001 - 1
00100 - 4
01010 - 10
00101 - 5
00111 - 7
01000 - 8 ~
01011 - 19

也不是二进制, tm的思维题
*/
```

思维题, 没什么好说的, 这里还有相似的题目: [1798. 你能构造出连续值的最大数目](https://leetcode.cn/problems/maximum-number-of-consecutive-values-you-can-make/description/)


```C++
class Solution {
public:
    int getMaximumConsecutive(vector<int>& coins) {
        sort(coins.begin(), coins.end());
        int r = 0;
/*
对于 [L, R] 区间加上 x => [L + x, R + x] 为可以构造出来的数
如果 R >= L + x - 1, 那么 即可以构造出 [L, R + x] 即 (R + x - L + 1) 个数
如果 R < L + x - 1, 那么 可以构造 出 [L + x, R + x]
*/
        for (int i = 0; i < coins.size() && r >= coins[i] - 1; ++i) {
            r += coins[i];
        }

        return r + 1;
    }
};
```
