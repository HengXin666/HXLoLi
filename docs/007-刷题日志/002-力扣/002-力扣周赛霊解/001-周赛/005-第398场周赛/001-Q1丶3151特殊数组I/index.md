# 3151. 特殊数组 I

链接: [3151. 特殊数组 I](https://leetcode.cn/problems/special-array-i/)

如果数组的每一对相邻元素都是两个奇偶性不同的数字，则该数组被认为是一个 特殊数组 。

Aging 有一个整数数组 nums。如果 nums 是一个 特殊数组 ，返回 true，否则返回 false。

# 题解

```C++
class Solution {
public:
    bool isArraySpecial(vector<int>& nums) {
        for (int i = 1; i < nums.size(); ++i)
            if ((nums[i] & 1) == (nums[i - 1] & 1))
                return false;
        return true;
    }
};
```
