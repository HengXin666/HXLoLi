# 3159. 查询数组中元素的出现位置
链接: [3159. 查询数组中元素的出现位置](https://leetcode.cn/problems/find-occurrences-of-an-element-in-an-array/)

给你一个整数数组 nums ，一个整数数组 queries 和一个整数 x 。

对于每个查询 queries[i] ，你需要找到 nums 中第 queries[i] 个 x 的位置，并返回它的下标。如果数组中 x 的出现次数少于 queries[i] ，该查询的答案为 -1 。

请你返回一个整数数组 answer ，包含所有查询的答案。

# 题解
## 记录所有等于 x 的元素下标

```C++
class Solution {
public:
    vector<int> occurrencesOfElement(vector<int>& nums, vector<int>& res, int x) {
        vector<int> arr;
        for (int i = 0; i < nums.size(); ++i)
            if (nums[i] == x)
                arr.push_back(i);
        
        for (int& it : res) {
            if (it <= arr.size())
                it = arr[it - 1];
            else
                it = -1;
        }
        return res;
    }
};
```
