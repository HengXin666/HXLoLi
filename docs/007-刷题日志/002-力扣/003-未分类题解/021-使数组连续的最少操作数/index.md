# 2009. 使数组连续的最少操作数

链接: [2009. 使数组连续的最少操作数](https://leetcode.cn/problems/minimum-number-of-operations-to-make-array-continuous/description/)

第 61 场双周赛 Q4 2084

---

给你一个整数数组 nums 。每一次操作中，你可以将 nums 中 任意 一个元素替换成 任意 整数。

如果 nums 满足以下条件，那么它是 连续的 ：

- nums 中所有元素都是 互不相同 的。
- nums 中 最大 元素与 最小 元素的差等于 nums.length - 1 。

比方说，nums = [4, 2, 5, 3] 是 连续的 ，但是 nums = [1, 2, 3, 5, 6] 不是连续的 。

请你返回使 nums 连续 的 最少 操作次数。

### 示例 1：
```
输入：nums = [4,2,5,3]
输出：0
解释：nums 已经是连续的了。
```

### 示例 2：
```
输入：nums = [1,2,3,5,6]
输出：1
解释：一个可能的解是将最后一个元素变为 4 。
结果数组为 [1,2,3,5,4] ，是连续数组。
```

### 示例 3：
```
输入：nums = [1,10,100,1000]
输出：3
解释：一个可能的解是：
- 将第二个元素变为 2 。
- 将第三个元素变为 3 。
- 将第四个元素变为 4 。
结果数组为 [1,2,3,4] ，是连续数组。
```

提示:

$
1 <= nums.length <= 10^5\\
1 <= nums[i] <= 10^9
$

# 代码
## 我的思考

如下, 然而连`测试用例2`都不满足...

```C++
class Solution {
public:
    int minOperations(vector<int>& nums) {
        int res = 0;
        int len = nums.size() - 1;
        vector<int> fk(nums.size()); // 通解数组
        for (int i = 0; i < nums.size(); ++i)
            fk[i] = i + 1;
        sort(nums.begin(), nums.end());
        // 显然是存在如下解:
        // [1, 2, 3, 4, n - 1] + k == len - 1 (k为整数)
        //  0  1  2  3  n - 2
        // 也就是寻找 nums 中最多元素符合的 这个数组

        // 最优的解决方案就是, 原数组的值尽量不变
        // [1, 2, 3, 4, n - 1] + k == len - 1
        // [3, 3, 7, 8, 9] == 4 确定k
        // 即 排序后的数组中最长的严格递增且差为一子数组, 其起始元素
        // 映射到 解决方案数组的 对应索引
        // 两元素之差
        // 然后就可以遍历修改了

        // 寻找最长严格递增相差1的子数组
        int left = 0;
        int addIndex = 0, addLen = 1;
        for (int right = left + 1; right < nums.size(); ++right) {
            while (right < nums.size() && nums[right] - nums[right - 1] == 1) {
                ++right;
            }
            
            if (addLen < right - left) {
                addIndex = left;
                addLen = right - left;
            }
            left = right;
        }

        // 还有一种情况: 就是如果 修改少数(a) 可以使得(b)连续
        // a + b > c(原本的连续)
        // 如何求?

        // 1 2 3 4 + k
        // 2 3 4 5 <-
        // cout << addIndex << " ";
        int k = nums[addIndex] - fk[addIndex];
        // cout << k << '\n';

        for (int i = 0; i < nums.size(); ++i) {
            if (nums[i] != fk[i] + k)
                ++res;
        }

        // 时间复杂度 nlogn
        return res; // 0 <= res <= len - 1
    }
};
```

## 03xf
正难则反，考虑 **最多保留** 多少个元素不变。

我写的去重太慢了
```C++
class Solution {
public:
    int minOperations(vector<int>& nums) {
        // 变res个元素 == 不变 n - res 个元素
        // 显然元素需要在 [k, k + n - 1] 区间连续
        // 即 [k - n + 1, k] k 是 连续数字的最大值, n 是原数组nums的长度
        int n = nums.size();
        {   // 去重 + 排序
            set<int> tmp(nums.begin(), nums.end());
            nums = vector<int>(tmp.begin(), tmp.end());
        }
        int res = 0; // 不变的元素个数

        int left = 0;
        for (int right = 0; right < nums.size(); ++right) {
            while (nums[left] < nums[right] - n + 1) {
                ++left;
            }
            res = max(res, right - left + 1);
        }

        return n - res;
    }
}; // By Heng_Xin
```

这个快的离谱:
```C++
class Solution {
public:
    int minOperations(vector<int> &nums) {
        int n = nums.size();
        ranges::sort(nums);
        nums.erase(unique(nums.begin(), nums.end()), nums.end()); // 原地去重
        int ans = 0, left = 0;
        for (int i = 0; i < nums.size(); i++) {
            while (nums[left] < nums[i] - n + 1) { // nums[left] 不在窗口内
                left++;
            }
            ans = max(ans, i - left + 1);
        }
        return n - ans;
    }
};

// 作者：灵茶山艾府
// 链接：https://leetcode.cn/problems/minimum-number-of-operations-to-make-array-continuous/solutions/1005398/on-zuo-fa-by-endlesscheng-l7yi/
// 来源：力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```
