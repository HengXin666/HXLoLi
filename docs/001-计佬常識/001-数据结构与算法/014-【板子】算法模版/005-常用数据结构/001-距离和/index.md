# 距离和 (数学化简后 前缀和)

问题可能是多样的, 但是核心是一个类似于以下形态的式子:

$$
\sum_{j = 0}^{n - 1}{\left | i - j \right | }
$$

这个是`距离和`相关题目的特征 *(个人认为)*, 做题时候只需要展开后慢慢推导即可qwq...

---

例如:

> [1685. 有序数组中差绝对值之和](https://leetcode.cn/problems/sum-of-absolute-differences-in-a-sorted-array/)
>
> 输入: 一个 **非递减** 有序整数数组`num`
> 
> ```C++
> vector<int> getSumAbsoluteDifferences(vector<int>& nums)
> ```
>
> 返回一个整数数组`res`, 它跟`nums`长度相同, 且`res[i]`等于`nums[i]`与数组中所有其他元素差的绝对值之和.
>
> 换句话说, `res[i]`等于`sum(|nums[i]-nums[j]|)`, 其中`0 <= j < nums.length`且`j != i`(下标从 0 开始).

此时得到的式子是:

$$
res[i] = \sum_{j = 0}^{n - 1}{\left | nums[i] - nums[j] \right | }
$$

我们可以将其展开:

$$
res[i] = |nums[i] - nums[0]| + |nums[i] - nums[1]| + ... + |nums[i] - nums[i - 1]| \\ \ \\ + |nums[i] - nums[i]| \\ \ \\ + |nums[i] - nums[i + 1]| + ... + |nums[i] - nums[n - 1]|
$$

由于数组是非递减的, 我们可以把绝对值拆开:

$$
res[i] = (nums[i] - nums[0]) + (nums[i] - nums[1]) + ... + (nums[i] - nums[i - 1]) \\ \ \\ + (nums[i] - nums[i]) \\ \ \\ + (nums[i + 1] - nums[i]) + ... + (nums[n - 1] - nums[i])
$$

然后去掉括号+移项, 亦有:

$$
res[i] = [i \times nums[i] - (nums[0] + nums[1] + ... + nums[i - 1])] \\ \ \\ + 0 \\ \ \\ + [(nums[i + 1] + ... + nums[n - 1]) - (n - i) \times nums[i]]
$$

即

$$
res[i] = \sum_{x = i + 1}^{n - 1}nums[x] - \sum_{x = 0}^{i - 1} nums[x] + nums[i] \times [i - (n - i)]
$$

而对于求解 $\sum_{x = i + 1}^{n - 1}$ 与 $\sum_{x = 0}^{i - 1}$ 显然我们可以使用前缀和来计算.

故有:

```C++
class Solution {
public:
    vector<int> getSumAbsoluteDifferences(vector<int>& nums) {
        int n = nums.size();
        vector<int> sum(n + 1);
        for (int i = 0; i < n; ++i)
            sum[i + 1] = sum[i] + nums[i];
        vector<int> res(n);
        for (int i = 0; i < n; ++i) {
            res[i] = (sum[n] - sum[i]) - (sum[i] - sum[0]) 
                   + nums[i] * (i - (n - i)); 
        }
        return res;
    }
};
```

> [!TIP]
> 为何是 $(i - (n - i))$ 而不是 $(i - ((n - 1) - i))$ ?
>
> 答: $n - 1$ 是换算到索引的, 而这里表示的是个数, 因此使用 $n = nums.size()$ 即可! *(小心不要和前缀和的长度搞混了!)*

## 相关题目

- [x] 本题 [1685. 有序数组中差绝对值之和](https://leetcode.cn/problems/sum-of-absolute-differences-in-a-sorted-array/) 1496
- [2615. 等值距离和](https://leetcode.cn/problems/sum-of-distances/) 1793
- [2602. 使数组元素全部相等的最少操作次数](https://leetcode.cn/problems/minimum-operations-to-make-all-array-elements-equal/) 1903

更多可以参考灵神题单: 
- [本题相关] [分享丨【题单】常用数据结构（前缀和/差分/栈/队列/堆/字典树/并查集/树状数组/线段树）](https://leetcode.cn/circle/discuss/mOr1u6/)
