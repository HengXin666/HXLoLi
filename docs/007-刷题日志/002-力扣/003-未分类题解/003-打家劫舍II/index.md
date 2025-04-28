# 213. 打家劫舍 II

原题链接: [213. 打家劫舍 II](https://leetcode.cn/problems/house-robber-ii/description/) 

已解答

中等

你是一个专业的小偷，计划偷窃沿街的房屋，每间房内都藏有一定的现金。这个地方所有的房屋都 围成一圈 ，这意味着第一个房屋和最后一个房屋是紧挨着的。同时，相邻的房屋装有相互连通的防盗系统，如果两间相邻的房屋在同一晚上被小偷闯入，系统会自动报警 。

给定一个代表每个房屋存放金额的非负整数数组，计算你 在不触动警报装置的情况下 ，今晚能够偷窃到的最高金额。

### 示例 1：

输入：nums = [2,3,2]

输出：3

解释：你不能先偷窃 1 号房屋（金额 = 2），然后偷窃 3 号房屋（金额 = 2）, 因为他们是相邻的。
### 示例 2：

输入：nums = [1,2,3,1]

输出：4

解释：你可以先偷窃 1 号房屋（金额 = 1），然后偷窃 3 号房屋（金额 = 3）。
     偷窃到的最高金额 = 1 + 3 = 4 。
### 示例 3：

输入：nums = [1,2,3]

输出：3
 

### 提示：

$1 <= nums.length <= 100$

$0 <= nums[i] <= 1000$

# 解答
## 我的代码

思路: 使用 `打家劫舍I` 的dp 思路, 然后分别dp两条线路, 取max值

```C++
class Solution {
public:
    int DP[1001] = {0};   // 记录左1
    int DP_2[1001] = {0}; // 记录非左1
    int rob(vector<int>& nums) {
        // if (nums.size() <= 3) {
        //     int res = 0;
        //     for (auto& it : nums) {
        //         if (res < it)
        //             res = it;
        //     }

        //     return res;
        // }


        DP[0] = nums[0];
        if (nums.size() >= 2) {
            DP[1] = DP[0];
            DP_2[1] = nums[1];
            if (nums.size() >= 3) {
                DP_2[2] = nums[2];
            }
        }

        for (int i = 2, j = 2; i < nums.size(); ++i, ++j) {
            DP_2[j] = max(DP_2[j - 1], DP_2[j - 2] + ((j < nums.size()) ? nums[j] : 0));

            if (i < nums.size() - 1) {
                DP[i] = max(DP[i - 1], DP[i - 2] + ((i < nums.size()) ? nums[i] : 0));
            }
            else {
                DP[i] = max(DP[i - 1], DP[i - 2]);
            }
        }

        // printf("%d %d", DP[nums.size() - 1], DP_2[nums.size() - 1]);
        return max(DP[nums.size() - 1], DP_2[nums.size() - 1]);
    }
};
```