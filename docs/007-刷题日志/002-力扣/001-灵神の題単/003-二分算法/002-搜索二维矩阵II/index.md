# 240. 搜索二维矩阵 II

链接: [240. 搜索二维矩阵 II](https://leetcode.cn/problems/search-a-2d-matrix-ii/)

斜着看, 是二叉搜索树!

```C++
class Solution {
public:
    bool searchMatrix(vector<vector<int>>& matrix, int target) {
        int n = matrix.size(), m = matrix[0].size();
        int i = n - 1, j = 0;
        while (i >= 0 && j < m) {
            if (matrix[i][j] < target) {
                ++j;
            } else if (matrix[i][j] > target) {
                --i;
            } else {
                return true;
            }
        }
        return false;
    }
};
```
