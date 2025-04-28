# 918. 环形子数组的最大和
链接: [918. 环形子数组的最大和](https://leetcode.cn/problems/maximum-sum-circular-subarray/)

中等`1777` 第 105 场周赛 Q2

给定一个长度为 n 的环形整数数组 nums ，返回 nums 的非空 子数组 的最大可能和 。

环形数组 意味着数组的末端将会与开头相连呈环状。形式上， nums[i] 的下一个元素是 nums[(i + 1) % n] ， nums[i] 的前一个元素是 nums[(i - 1 + n) % n] 。

子数组 最多只能包含固定缓冲区 nums 中的每个元素一次。形式上，对于子数组 nums[i], nums[i + 1], ..., nums[j] ，不存在 i <= k1, k2 <= j 其中 k1 % n == k2 % n 。

# 题解
- 学习: [没有思路？一张图秒懂！（Python/Java/C++/Go/JS）](https://leetcode.cn/problems/maximum-sum-circular-subarray/solutions/2351107/mei-you-si-lu-yi-zhang-tu-miao-dong-pyth-ilqh)

<img src="https://pic.leetcode.cn/1689750394-drKSAI-lc918-c.png">

```C++
class Solution {
public:
    int maxSubarraySumCircular(vector<int>& arr) {
        int maxArr = arr[0], minArr = 0, n = arr.size(), sum = 0;
        for (int i = 0, tmpMax = 0, tmpMin = 0; i < n; ++i) {
            tmpMax = max(tmpMax, 0) + arr[i];
            tmpMin = min(tmpMin, 0) + arr[i];
            maxArr = max(maxArr, tmpMax);
            minArr = min(minArr, tmpMin);
            sum += arr[i];
        }

        if (sum == minArr)
            return maxArr;
        return max(maxArr, sum - minArr);
    }
};
```
