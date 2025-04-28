# 2369. 检查数组是否存在有效划分
题目: [2369. 检查数组是否存在有效划分](https://leetcode.cn/problems/check-if-there-is-a-valid-partition-for-the-array/description/)

中等

给你一个下标从 0 开始的整数数组 nums ，你必须将数组划分为一个或多个 连续 子数组。

如果获得的这些子数组中每个都能满足下述条件 之一 ，则可以称其为数组的一种 有效 划分：

1. 子数组 恰 由 2 个相等元素组成，例如，子数组 [2,2] 。
2. 子数组 恰 由 3 个相等元素组成，例如，子数组 [4,4,4] 。
3. 子数组 恰 由 3 个连续递增元素组成，并且相邻元素之间的差值为 1 。例如，子数组 [3,4,5] ，但是子数组 [1,3,5] 不符合要求。

如果数组 至少 存在一种有效划分，返回 true ，否则，返回 false 。

## 示例 1：

输入：nums = [4,4,4,5,6]<br>
输出：true<br>
解释：数组可以划分成子数组 [4,4] 和 [4,5,6] 。<br>
这是一种有效划分，所以返回 true 。

## 示例 2：

输入：nums = [1,1,1,2] <br>
输出：false<br>
解释：该数组不存在有效划分。

## 提示：

$
2 <= nums.length <= 10^5 \\
1 <= nums[i] <= 10^6
$

# 题解
如何想出状态定义？

如果 $\textit{nums}$ 的最后两个数相等，那么去掉这两个数，问题变成剩下 $n-2$ 个数能否有效划分。<br>
如果 $\textit{nums}$ 的最后三个数相等，那么去掉这三个数，问题变成剩下 $n-3$ 个数能否有效划分。<br>
如果 $\textit{nums}$ 的最后三个数是连续递增的，那么去掉这三个数，问题变成剩下 $n-3$ 个数能否有效划分。<br>

我们要解决的问题都形如「 $\textit{nums}$ 的前 $i$ 个数能否有效划分」。

于是定义 $ \texttt{f[0] = true}$ , $f[i+1]$ 表示能否有效划分 $\textit{nums[0]}$ 到 $\textit{nums[i]}$。

根据有效划分的定义，有

$$
f[i+1]=\vee\left\{\begin{aligned}
f[i-1] \wedge \text { nums }[i]=n u m s[i-1], & & i>0 \\
f[i-2] \wedge \text { nums }[i]=\text { nums }[i-1]=n u m s[i-2], & & i>1 \\
f[i-2] \wedge \text { nums }[i]=n u m s[i-1]+1=n u m s[i-2]+2, & & i>1
\end{aligned}\right.
$$

```C++
class Solution {
public:
    bool validPartition(vector<int> &nums) {
        int n = nums.size();
        vector<int> f(n + 1);
        f[0] = true;
        for (int i = 1; i < n; i++) {
            if (f[i - 1] && nums[i] == nums[i - 1] ||
                i > 1 && f[i - 2] && (nums[i] == nums[i - 1] && nums[i] == nums[i - 2] ||
                                      nums[i] == nums[i - 1] + 1 && nums[i] == nums[i - 2] + 2)) {
                f[i + 1] = true;
            }
        }
        return f[n];
    }
};

/*
作者：灵茶山艾府
链接：https://leetcode.cn/problems/check-if-there-is-a-valid-partition-for-the-array/solutions/1728735/by-endlesscheng-8y73/
来源：力扣（LeetCode）
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。*/
```