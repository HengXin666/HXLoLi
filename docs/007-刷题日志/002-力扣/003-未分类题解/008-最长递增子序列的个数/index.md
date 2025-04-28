# 673. 最长递增子序列的个数

原题链接:  [673. 最长递增子序列的个数](https://leetcode.cn/problems/number-of-longest-increasing-subsequence/description/)

中等

给定一个未排序的整数数组 `nums` ， 返回最长递增子序列的个数 。

注意 这个数列必须是 **严格** 递增的。

 

## 示例 1:

输入: [1,3,5,4,7]<br>
输出: 2<br>
解释: 有两个最长递增子序列，分别是 [1, 3, 4, 7] 和 [1, 3, 5, 7]。
## 示例 2:

输入: [2,2,2,2,2]<br>
输出: 5<br>
解释: 最长递增子序列的长度是1，并且存在5个子序列的长度为1，因此输出5。
 

## 提示: 

$1 <= nums.length <= 2000$<br>
$-10^6 <= nums[i] <= 10^6$

# 代码

## 我的记忆化搜索

```C++
class Solution {
public:
    int BFS(int now_i, vector<int>& nums, vector<int>& dp, vector<int>& dp_g) {
        if (dp[now_i] != -1)
            return dp[now_i];
        
        int res = 1;
        for (int i = now_i - 1; i >= 0; --i) {
            if (nums[now_i] > nums[i]) {
                int tmp = BFS(i, nums, dp,dp_g) + 1;
                if (tmp == res) {
                    dp_g[now_i] += dp_g[i];
                } else if (tmp > res) {
                    res = tmp;
                    dp_g[now_i] = dp_g[i];
                }
            }
        }

        dp[now_i] = res;
        return dp[now_i];
    }

    int findNumberOfLIS(vector<int>& nums) {
        const int len = nums.size();
        // 定义: dp[i] 为到[i, len) 的递增子序列长度
        vector<int> dp(len, -1);
        vector<int> dp_g(len, 1);
        for (int i = 0; i < len; ++i)
            BFS(i, nums, dp, dp_g);

        int res = 0, maxRes = 0;

        for (int i = 0; i < len; ++i) {
            if (dp[i] > maxRes) {
                maxRes = dp[i];
                res = dp_g[i];
            } else if (dp[i] == maxRes){
                res += dp_g[i];
            }
        }

        return res;
    }
};
```

## 力扣官方题解dp

### 思路

第一步: 思路同[300. 最长递增子序列](../005-最长递增子序列/index.md)

第二步: 由于我们需要求解的是最长上升子序列的个数，因此需要额外定义 `g[i]` 为考虑以 `nums[i]` 结尾的最长上升子序列的个数。

- 结合 第一步的过程 我们可以想到这样转移:
    - 由于每个数都能独自一个成为子序列，因此起始必然有 `g[i] = 1`
    - 枚举`[0, i)` 所有的`nums[j]`, 如果满足 `nums[j] < nums[i]` 则说明 `nums[i]` 可以在 `nums[j]` 后面形成上升子序列. 这时候对 `dp[i]` 与 `dp[j] + 1` 讨论:
        - 满足 `dp[i] < dp[j] + 1`: 说明有更长的子序列, `dp[i]` 会被更新, 此时同步更新 `g[i] = g[j]` 即可.
        - 满足 `dp[i] == dp[j] + 1`: 说明形成了**等***当前***最长**的子序列, 所以 `dp[i]` 不变, 但 `g[i] += g[j]` (注意: *不一定是加一*!, 因为可能原本`g[j]`就可以形成多条当时最长子序列!)

### 代码
```C++
class Solution {
public:
    int findNumberOfLIS(vector<int> &nums) {
        int n = nums.size(), maxLen = 0, ans = 0;
        vector<int> dp(n), cnt(n);
        for (int i = 0; i < n; ++i) {
            dp[i] = 1;
            cnt[i] = 1;
            for (int j = 0; j < i; ++j) {
                if (nums[i] > nums[j]) {
                    if (dp[j] + 1 > dp[i]) {
                        dp[i] = dp[j] + 1;
                        cnt[i] = cnt[j]; // 重置计数
                    } else if (dp[j] + 1 == dp[i]) {
                        cnt[i] += cnt[j];
                    }
                }
            }
            if (dp[i] > maxLen) {
                maxLen = dp[i];
                ans = cnt[i]; // 重置计数
            } else if (dp[i] == maxLen) {
                ans += cnt[i];
            }
        }
        return ans;
    }
};

/*
作者：力扣官方题解
链接：https://leetcode.cn/problems/number-of-longest-increasing-subsequence/solutions/1007075/zui-chang-di-zeng-zi-xu-lie-de-ge-shu-by-w12f/
来源：力扣（LeetCode）
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。*/
```


# 题解

```C++
class Solution {
public:
    int resArrDP[2001] = {0};

    int BFS(int now_i, vector<int>& nums, vector<int>& dp) {
        if (dp[now_i] != -1)
            return dp[now_i];
        
        int res = 1;
        for (int i = now_i - 1; i >= 0; --i) {
            if (nums[now_i] > nums[i]) {
                int tmp = BFS(i, nums, dp) + 1;
                res = max(res, tmp);

                if (res == tmp)
                    ++resArrDP[tmp];
            }
        }

        dp[now_i] = res;
        return dp[now_i];
    }

    int findNumberOfLIS(vector<int>& nums) {
        const int len = nums.size();
        // 定义: dp[i] 为到[i, len) 的递增子序列长度
        vector<int> dp(len, -1);
        for (int i = 0; i < len; ++i)
            BFS(i, nums, dp);

        int res = 0;
        for (int i = len; i >= 0; --i) {
            if (resArrDP[i]) {
                res = resArrDP[i] + 1;
                for (int j = 0; j <= i; ++j) {
                    printf("%d ", resArrDP[j] + 1);
                }
                break;
            }
        }

        return res;
    }
};
```
起初思考到: 如果在dp的时候也记录数量, 不就OK了吗?
但是使用一个数组, 但是没有办法溯源 (指如果出现更大的没有办法回溯回去(因为当时设计的是使用这个数组来记录长度为x的序列有多少条(类似于哈希表)))