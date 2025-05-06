# 3127. 构造相同颜色的正方形
链接: [3127. 构造相同颜色的正方形](https://leetcode.cn/problems/make-a-square-with-the-same-color/)

给你一个二维 3 x 3 的矩阵 grid ，每个格子都是一个字符，要么是 'B' ，要么是 'W' 。字符 'W' 表示白色，字符 'B' 表示黑色。

你的任务是改变 至多一个 格子的颜色，使得矩阵中存在一个 2 x 2 颜色完全相同的正方形。

如果可以得到一个相同颜色的 2 x 2 正方形，那么返回 true ，否则返回 false 。

# 题解

```C++
class Solution {
    const int fx[4][2] = {
        {1, 0}, {0, 1}, {1, 1}, {0, 0}
    };
public:
    bool canMakeSquare(vector<vector<char>>& grid) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                int b = 0;
                for (auto& it : fx)
                    if (grid[i + it[0]][j + it[1]] == 'B')
                        ++b;
                if (b != 2)
                    return true;
            }
        }
        return false;
    }
};
```

## 计数(思路同上)
```C++
class Solution {
public:
    bool canMakeSquare(vector<vector<char>>& grid) {
        auto check = [&](int i, int j) {
            int cnt[2]{};
            cnt[grid[i][j] & 1]++;
            cnt[grid[i][j + 1] & 1]++;
            cnt[grid[i + 1][j] & 1]++;
            cnt[grid[i + 1][j + 1] & 1]++;
            return cnt[0] >= 3 || cnt[1] >= 3;
        };
        return check(0, 0) || check(0, 1) || check(1, 0) || check(1, 1);
    }
};
```
