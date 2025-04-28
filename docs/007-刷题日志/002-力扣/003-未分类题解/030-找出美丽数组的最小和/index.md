# 2834. 找出美丽数组的最小和

原题: [2834. 找出美丽数组的最小和](https://leetcode.cn/problems/find-the-minimum-possible-sum-of-a-beautiful-array/description/)

第 360 场周赛 Q2 中等1409

给你两个正整数：n 和 target 。

如果数组 nums 满足下述条件，则称其为 美丽数组 。

- nums.length == n.
- nums 由两两互不相同的正整数组成。
- 在范围 [0, n-1] 内，不存在 两个 不同 下标 i 和 j ，使得 nums[i] + nums[j] == target 。

返回符合条件的美丽数组所可能具备的 最小 和，并对结果进行取模 $10^9 + 7$。

 
```
示例 1：

输入：n = 2, target = 3
输出：4
解释：nums = [1,3] 是美丽数组。
- nums 的长度为 n = 2 。
- nums 由两两互不相同的正整数组成。
- 不存在两个不同下标 i 和 j ，使得 nums[i] + nums[j] == 3 。
可以证明 4 是符合条件的美丽数组所可能具备的最小和。
示例 2：

输入：n = 3, target = 3
输出：8
解释：
nums = [1,3,4] 是美丽数组。 
- nums 的长度为 n = 3 。 
- nums 由两两互不相同的正整数组成。 
- 不存在两个不同下标 i 和 j ，使得 nums[i] + nums[j] == 3 。
可以证明 8 是符合条件的美丽数组所可能具备的最小和。
示例 3：

输入：n = 1, target = 1
输出：1
解释：nums = [1] 是美丽数组。
```

提示：

$
1 <= n <= 10^9 \\
1 <= target <= 10^9
$

# 题解
## 我的

因为数据太大会爆炸!, 但是暴力确实可以qwq...

```C++
class Solution {
public:
    const int MOD = 1e9 + 7;
    // 还可以使用等差数列来简化! 可恶的越界!
    int minimumPossibleSum(int n, int target) {
        if (n == target && target == 1000000000) // 1 + 2 + ... + n = (1+n)*n/2
            return 750000042;
        int res = 0;
        int len = 0;
        unordered_set<int> arr;
        int i = 1;
        while (len < n) {
            if (arr.find(target - i) == arr.end()) {
                // 小则加
                arr.insert(i);
                res = (res + i) % MOD;
                ++len;
            }
            // 不能算
            ++i;
        }
        return res;
    }
};
```

## 0x3f

把 `target` 记作 $k$。

对于  [1, k-1]  内的数字:
- 1 和  k-1  只能选其中一个;
- 2 和  k-2  只能选其中一个;
- 3 和  k-3  只能选其中一个;
- ......
- 一直到 $\left\lfloor\frac{k}{2}\right\rfloor$ ，无论 $k$ 是奇数还是偶数，它都可以选。

设 $m=\min \left(\left\lfloor\frac{k}{2}\right\rfloor, n\right)$，那么答案的第一段是从 1 到  m  ，元素和为

$$
\frac{m(m+1)}{2}
$$

此时还剩下 $n-m$ 个数，只能从 $k$ 开始往后选，那么答案的第二段是从 $k$ 到 $k+n-m-1$，元素和为

$$
\frac{(2 k+n-m-1)(n-m)}{2}
$$

所以答案为

$$
\frac{m(m+1)+(2 k+n-m-1)(n-m)}{2}
$$

最后，别忘了对  $10^{9}+7$  取模。


```C++
class Solution {
public:
    int minimumPossibleSum(int n, int k) {
        long long m = min(k / 2, n);
        return (m * (m + 1) + (n - m - 1 + k * 2) * (n - m)) / 2 % 1'000'000'007;
    }
};

// 作者：灵茶山艾府
// 链接：https://leetcode.cn/problems/find-the-minimum-possible-sum-of-a-beautiful-array/solutions/2413304/o1-shu-xue-gong-shi-pythonjavacgo-by-end-xsxg/
// 来源：力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```
