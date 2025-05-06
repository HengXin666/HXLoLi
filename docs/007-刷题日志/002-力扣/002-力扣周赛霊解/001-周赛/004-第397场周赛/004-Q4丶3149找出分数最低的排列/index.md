# 3149. 找出分数最低的排列
链接: [3149. 找出分数最低的排列](https://leetcode.cn/problems/find-the-minimum-cost-array-permutation/)

给你一个数组 nums ，它是 [0, 1, 2, ..., n - 1] 的一个 **排列**。对于任意一个 [0, 1, 2, ..., n - 1] 的排列 perm ，其 **分数** 定义为：

$score(perm) = |perm[0] - nums[perm[1]]| + |perm[1] - nums[perm[2]]| + ... + |perm[n - 1] - nums[perm[0]]|$

返回具有 **最低** 分数的排列 perm 。如果存在多个满足题意且分数相等的排列，则返回其中 **字典序最小** 的一个。

提示:
- 2 <= n == nums.length <= 14
- nums 是 [0, 1, 2, ..., n - 1] 的一个排列。

# 题解
## 暴力

我不会

```C++
class Solution {
    // const int maxx = 5e7;
public:
    vector<int> findPermutation(vector<int>& nums) {
        vector<int> res, cp;
        cp = nums;
        sort(nums.begin(), nums.end());
        int fs = 1e9;
        int cs = 0;
        do {
            if (nums[0]) // 剪枝
                break;
            int tmp = abs(nums[nums.size() - 1] - cp[nums[0]]);
            for (int i = 1; i < nums.size(); ++i) {
                tmp += abs(nums[i - 1] - cp[nums[i]]);
            }
            
            // cout << tmp << '\n';
            if (tmp < fs) {
                fs = tmp;
                res = nums;
                // cout << fs << "!\n";
            }
            
            // if (++cs > maxx)
            //     break;
            
        } while (next_permutation(nums.begin(), nums.end()));
        // cout << fs << "!\n";
        return res;
    }
};
```

## 0x3f正解: 状压DP

> 如果数据范围很小, 就可能是状压dp (对于Q4)

- [状压 DP：从记忆化搜索到递推（Python/Java/C++/Go）](https://leetcode.cn/problems/find-the-minimum-cost-array-permutation/solutions/2775272/zhuang-ya-dpcong-ji-yi-hua-sou-suo-dao-d-s9t5)

不会, 先睡了...