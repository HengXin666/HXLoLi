# 2588. 统计美丽子数组数目
链接: [2588. 统计美丽子数组数目](https://leetcode.cn/problems/count-the-number-of-beautiful-subarrays/)

第 336 场周赛 Q3 `1697`

给你一个下标从 `0` 开始的整数数组nums 。每次操作中，你可以：

- 选择两个满足 `0 <= i, j < nums.length` 的不同下标 $i$ 和 $j$ 。
- 选择一个非负整数 $k$ ，满足`nums[i]`和`nums[j]`在二进制下的第 $k$ 位（下标编号从 0 开始）是`1`。
- 将`nums[i]`和`nums[j]`都减去 $2^k$。

如果一个子数组内执行上述操作若干次后，该子数组可以变成一个全为`0`的数组，那么我们称它是一个 **美丽** 的子数组。

请你返回数组`nums`中 美丽子数组 的数目。

子数组是一个数组中一段连续 **非空** 的元素序列。

# 题解
## 套路: [异或前缀和 + 哈希表]
1. 首先我们需要知道「将`nums[i]`和`nums[j]`都减去 $2^k$ 」这句话的意思, 就是去掉 这两个数的相同位数上的`1`; 因为最终目标是子数组全部变成`0`, 也就是要保证所有数的每一个位的1的个数都是偶数个(1^1=0), 即 子数组异或和为0, 那么可以通过`异或前缀和`快速求解.

$$偶数个1变成0, 就需要想到 XOR!$$

2. 而哈希表, 可以帮助我们计算子数组个数: 对于 子数组 $[i, j)$, 如果它是美丽的, 则需要满足`子数组异或和为0`, 即`sumArr[j] ^ sumArr[i] == 0`即`sumArr[j] == sumArr[i]`; 因此我们只需要统计`sumArr[x]`的个数, 即可知道所有美丽子数组的个数:

```C++
// 对于异或前缀和:
0 1 3 4 6 4 0 4 1

// 0(后面) ~ 0(前面) : 1
  
// 4(中间) ~ 4(前面) : 1

// 4(后面) ~ 4(中间) : 1
// 4(后面) ~ 4(前面) : 1
```

代码:
```C++
class Solution {
public:
    long long beautifulSubarrays(vector<int>& nums) {
        long long res = 0;
        vector<int> sumArr(nums.size() + 1);
        for (int i = 0; i < nums.size(); ++i)
            sumArr[i + 1] = sumArr[i] ^ nums[i];

        unordered_map<int, int> cnt;
        for (int it : sumArr)
            res += cnt[it]++;
        
        return res;
    }
};
```

更可以一边统计一般计算:

```C++
class Solution {
public:
    long long beautifulSubarrays(vector<int>& nums) {
        long long res = 0;
        unordered_map<int, int> cnt;
        cnt[0] = 1;
        int sum = 0;
        for (int it : nums) {
            sum ^= it;
            res += cnt[sum]++;
        }
        
        return res;
    }
};
```