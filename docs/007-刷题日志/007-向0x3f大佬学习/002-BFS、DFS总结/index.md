
## 二叉树 vs 网格图 vs 一般图
|  | 重复访问 | 邻居个数 | DFS | BFS |
| --- | --- | --- | --- | --- |
| 二叉树 | 否 | $\le 3$ | 前中后序 | 层序 |
| 网格图 | 是 | $\le 8$ | 连通块 | 最短路 |
| 一般图 | 是 | 任意 | 连通块、判环等 | 最短路等 |

> 注1：通常情况下，网格图是四方向的，每个格子的邻居个数不超过 $4$。本题是八方向的，每个格子的邻居个数不超过 $8$。
>
> 注2：BFS 也可以判断连通块，但要手动用队列保存待访问节点；而 DFS 是计算机帮你创建了一个栈，自动保存递归路径上的节点，不需要手动处理。所以代码上 DFS 通常比 BFS 要简短。

## BFS 模版

**BFS 可以用来求`最短路径问题`。BFS 先搜索到的结点，一定是距离最近的结点。**

BFS 使用队列，把每个还没有搜索到的点依次放入队列，然后再弹出队列的头部元素当做当前遍历点。BFS 总共有两个模板：

1. 如果不需要确定当前遍历到了哪一层，BFS 模板如下。

```c++ py
while queue 不空：
    cur = queue.pop()
    for 节点 in cur的所有相邻节点：
        if 该节点有效且未访问过：
            标记该节点为已经访问
            queue.push(该节点)
```

2. 如果要确定当前遍历到了哪一层，BFS 模板如下。

    这里增加了`level`表示当前遍历到二叉树中的哪一层了，也可以理解为在一个图中，现在已经走了多少步了。`size`表示在当前遍历层有多少个元素，也就是队列中的元素数，我们把这些元素一次性遍历完，即把当前层的所有元素都向外走了一步。


```C++ py
level = 0
while queue 不空：
    size = queue.size()
    while (size --) {
        cur = queue.pop()
        for 节点 in cur的所有相邻节点：
            if 该节点有效且未被访问过：
                标记该节点为已经访问
                queue.push(该节点)
    }
    level ++;
```

<b style="color:red">血的教训: 最好不要在`for`外面进行标记是否已经访问, 

- 因为可能你入队还没有来得及出队(标记)
- 就又有一个元素又tm访问到你之前的位置, 然后因为没有来得及标记, 就导致重复入队了!!!
- *下面前几个代码都有这种错误, 我还纳闷了, 所以在`for`外面写了一些奇形怪状的`if`*
</b>

### 各类题的写法
<style>
    /* 自定义滚动条样式 */
    .hx::-webkit-scrollbar {
        height: 10px; /* 设置滚动条宽度 */
    }

    /* 轨道 */
    .hx::-webkit-scrollbar-track {
        background-color: #650056; /* 设置轨道背景色 */
    }

    /* 滑块 */
    .hx::-webkit-scrollbar-thumb {
        background-color: #990099; /* 设置滑块背景色 */
        border-radius: 5px; /* 设置滑块圆角 */
    }

    /* 当鼠标悬停在滑块上时 */
    .hx::-webkit-scrollbar-thumb:hover {
        background-color: red; /* 设置滑块悬停时的背景色 */
    }
</style>

<div class="hx" style="display: flex; overflow-x: auto;">
<div style="flex: 1; border: 1px solid black; padding: 10px;">

[695. 岛屿的最大面积](https://leetcode.cn/problems/max-area-of-island/description/)
```C++
class Solution {
    int fx[4][2] = { // 方向向量
        {0, 1}, {0, -1}, {1, 0}, {-1, 0}
    };
public:
    int maxAreaOfIsland(vector<vector<int>>& grid) {
        int res = 0;
        int now_s = 0;
        function<void(int, int)> dfs = [&](int i, int j) {
            if (i < 0 || j < 0 || i >= grid.size() || j >= grid[0].size() || grid[i][j] != 1)
                return;

            grid[i][j] = 2; // 标记为已经走过
            ++now_s;

            for (auto& it : fx) {
                dfs(i + it[0], j + it[1]);
            }
        };

        for (int i = 0; i < grid.size(); ++i) {
            for (int j = 0; j < grid[i].size(); ++j) {
                if (grid[i][j] == 1) {
                    now_s = 0;
                    dfs(i, j);
                    res = max(res, now_s);
                }
            }
        }

        return res;
    }
};
```

</div>
<div style="flex: 1; border: 1px solid black; padding: 10px;">

[面试题 16.19. 水域大小](https://leetcode.cn/problems/pond-sizes-lcci/description/)
```C++
class Solution {
    const int fx[8][2] = { // 方向向量
        {0, 1}, {0, -1},
        {1, 0}, {-1, 0},
        {1, 1}, {-1, -1},
        {1, -1}, {-1, 1}
    };
public:
    vector<int> pondSizes(vector<vector<int>>& land) {
        vector<int> tmp;
        int now_s = 0;
        function<void(int, int)> dfs = [&](int i, int j) {
            if (i < 0 || j < 0 || i >= land.size() || j >= land[0].size() || land[i][j])
                return;
            land[i][j] = -1; // 标记已经访问的
            ++now_s;
            for (auto& it : fx) {
                dfs(i + it[0], j + it[1]);
            }
        };

        for (int i = 0; i < land.size(); ++i) {
            for (int j = 0; j < land[0].size(); ++j) {
                if (!land[i][j]) {
                    now_s = 0;
                    dfs(i, j);
                    tmp.push_back(now_s);
                }
            }
        }

        sort(tmp.begin(), tmp.end());
        return tmp;
    }
};
```

</div>

<div style="flex: 1; border: 1px solid black; padding: 10px;">

[463. 岛屿的周长](https://leetcode.cn/problems/island-perimeter/description/)
```C++
class Solution {
    const int fx[4][2] = {
        {0, 1}, {0, -1},
        {-1, 0}, {1, 0}
    };
public:
    int islandPerimeter(vector<vector<int>>& grid) {
        int res = 0;
        int n = grid.size(), m = grid[0].size();
        // 恰好有一个岛
        function<bool(int, int)> dfs = [&](int i, int j) {
            if (i < 0 || j < 0 || i >= n || j >= m || !grid[i][j])
                return 1;
            if (grid[i][j] == 2)
                return 0;
            grid[i][j] = 2; // 标记
            for (auto& it : fx) {
                if (dfs(i + it[0], j + it[1]))
                    ++res;
            }
            return 0;
        };

        for (int i = 0; i < n; ++i)
            for (int j = 0; j < m; ++j)
                if (grid[i][j] == 1)
                    dfs(i, j);
        return res;
    }
};
```

</div>

<div style="flex: 1; border: 1px solid black; padding: 10px;">

[2658. 网格图中鱼的最大数目](https://leetcode.cn/problems/maximum-number-of-fish-in-a-grid/description/)
```C++
class Solution {
    const int fx[4][2] = {
        {0, 1}, {1, 0},
        {0, -1}, {-1, 0}
    };
public:
    int findMaxFish(vector<vector<int>>& grid) {
        int res = 0;
        int now_s = 0;
        int n = grid.size(), m = grid[0].size();
        function<void(int, int)> dfs = [&](int i, int j) {
            if (i < 0 || j < 0 || i >= n || j >= m || grid[i][j] <= 0)
                return;
            
            now_s += grid[i][j];
            grid[i][j] = -1;

            for (auto& it : fx) {
                dfs(i + it[0], j + it[1]);
            }
        };

        for (int i = 0; i < n; ++i)
            for (int j = 0; j < m; ++j)
                if (grid[i][j] > 0) {
                    now_s = 0;
                    dfs(i, j);
                    res = max(res, now_s);
                }
        return res;
    }
};
```

</div>

<div style="flex: 1; border: 1px solid black; padding: 10px;">

[1034. 边界着色](https://leetcode.cn/problems/coloring-a-border/description/)

```C++
class Solution {
    const int fx[4][2] = {
        {0, 1}, 
        {1, 0},
        {0, -1},
        {-1, 0}
    };
public:
    vector<vector<int>> colorBorder(vector<vector<int>>& grid, int row, int col, int color) {
        vector<vector<char>> vis(grid.size(), vector<char>(grid[0].size(), 0)); 
        // 对 相同颜色 [相同连通分量]
        // 不同颜色进行染色 [相同连通分量边界->数组边界/]
        function<bool(int, int, int)> dfs = [&](int i, int j, int paColor) {
            if (i < 0 
            || j < 0 
            || i >= grid.size() 
            || j >= grid[i].size())
                return 1;
            
            if (vis[i][j])
                return 0;

            if (grid[i][j] != paColor) {
                return 1;
            }
            
            vis[i][j] = 1; // 标记

            for (auto& it : fx) {
                if (dfs(i + it[0], j + it[1], paColor)) {
                    grid[i][j] = color;
                }
            }

            return 0;
        };

        dfs(row, col, grid[row][col]);

        return grid;
    }
};
```
</div>

<div style="flex: 1; border: 1px solid black; padding: 10px;">

[542. 01 矩阵](https://leetcode.cn/problems/01-matrix/description/)

```C++
class Solution {
    const int fx[4][2] = {
        {0, 1}, {0, -1}, {1, 0}, {-1, 0}
    };
public:
    vector<vector<int>> updateMatrix(vector<vector<int>>& mat) {
        int n = mat.size();
        int m = mat[0].size();
        vector<vector<char>> vis(n, vector<char>(m));
        queue<pair<pair<int, int>, int>> pq;

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                if (!mat[i][j]) {
                    pq.push({{i, j}, 0});
                } else
                    mat[i][j] = -1;
            }
        }

        while (pq.size()) {
            auto [zb, jl] = pq.front();
            auto [i, j] = zb;
            pq.pop();

            if (i < 0 || j < 0 || i >= n || j >= m || vis[i][j])
                continue;
            vis[i][j] = 1;

            if (mat[i][j] == -1)
                mat[i][j] = jl;

            for (auto& it : fx) {
                pq.push({{i + it[0], j + it[1]}, jl + 1});
            }
        }

        return mat;
    }
};
```

</div>

<div style="flex: 1; border: 1px solid black; padding: 10px;">

[994. 腐烂的橘子](https://leetcode.cn/problems/rotting-oranges/description/)

```C++
class Solution {
    const int fx[4][2] = {
        {0, 1}, {1, 0},
        {-1, 0}, {0, -1}
    };
public:
    int orangesRotting(vector<vector<int>>& grid) {
        int res = 0; // 分钟数
        int good = 0;
        int n = grid.size();
        int m = grid[0].size();
        queue<pair<pair<int, int>, int>> pq;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                if (grid[i][j] == 2) {
                    pq.push({{i, j}, 0});
                } else if (grid[i][j] == 1)
                    ++good;
            }
        }

        while (pq.size()) {
            auto [zb, t] = pq.front();
            auto [i, j] = zb;
            pq.pop();
            if (grid[i][j] == 3)
                continue;
            res = max(res, t);
            if (grid[i][j] == 1 && --good == 0)
                return res;
            grid[i][j] = 3;
            for (auto& it : fx) {
                int y = i + it[0], x = j + it[1];
                if (y >= 0 && y < n && x >= 0 && x < m && grid[y][x] == 1) {
                    pq.push({{y, x}, t + 1});
                }
            }
        }

        return good ? -1 : res;
    }
};
```
  
</div>

<div style="flex: 1; border: 1px solid black; padding: 10px;">

[1926. 迷宫中离入口最近的出口](https://leetcode.cn/problems/nearest-exit-from-entrance-in-maze/description/)


```C++
class Solution {
    const int fx[4][2] = {
        {1, 0}, {0, 1}, {0, -1}, {-1, 0}
    };
public:
    int nearestExit(
        vector<vector<char>>& maze, vector<int>& entrance) {
            int n = maze.size();
            int m = maze[0].size();
            queue<pair<int, int>> pq;
            pq.push({entrance[0], entrance[1]}); // [i][j]
            int res = 0;
            while (pq.size()) {
                int k = pq.size();
                while (k--) {
                    auto [i, j] = pq.front();
                    pq.pop();

                    if (!(i == entrance[0] && j == entrance[1]) // 找到出口
                    && (i == 0 || j == 0 || i == n - 1 || j == m - 1))
                        return res;

                    for (auto& it : fx) {
                        int y = i + it[0], x = j + it[1];
                        if (y >= 0 && y < n && x >= 0 && x < m && maze[y][x] == '.') {
                            maze[y][x] = '-';
                            pq.push({y, x});
                        }
                    }
                }
                ++res;
            }

            return -1;
    }
};
```
</div>

<div style="flex: 1; border: 1px solid black; padding: 10px;">

[1162. 地图分析](https://leetcode.cn/problems/as-far-from-land-as-possible/description/)

```C++
class Solution {
    const int fx[4][2] = {
        {1, 0}, {-1, 0}, {0, -1}, {0, 1}
    };
public:
    int maxDistance(vector<vector<int>>& grid) {
        // 找出所有的 0
        // 要距离它最近的 1
        queue<pair<int, int>> pq;
        int n = grid.size(), m = grid[0].size();
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                if (grid[i][j]) {
                    grid[i][j] = -1; // 已经访问过了
                    pq.push({i, j});
                }
            }
        }

        if (!pq.size() || pq.size() == m * n)
            return -1;

        int res = 0;
        while (pq.size()) {
            int k = pq.size();
            while (k--) {
                auto [i, j] = pq.front();
                pq.pop();

                for (auto& it : fx) {
                    int y = i + it[0], x = j + it[1];
                    if (y >= 0 && y < n && x >= 0 && x < m && grid[y][x] != -1) {
                        grid[y][x] = -1;
                        pq.push({y, x});
                    }
                }
            }
            ++res;
        }

        return res - 1;
    }
};
```
</div>

<div style="flex: 1; border: 1px solid black; padding: 10px;">

[934. 最短的桥](https://leetcode.cn/problems/shortest-bridge/description/)

```C++
class Solution {
    const int fx[4][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };
public:
    int shortestBridge(vector<vector<int>>& grid) {
        int n = grid.size();
        int m = grid[0].size();
        queue<pair<int, int>> pq;

        function<void(int, int)> dfs = [&](int i, int j) {
            if (i < 0 || j < 0 || i >= n || j >= m || grid[i][j] != 1)
                return;
            grid[i][j] = 2;
            pq.push({i, j});
            for (auto& it : fx) {
                dfs(i + it[0], j + it[1]);
            }
        };

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                if (grid[i][j]) {
                    dfs(i, j);
                    goto FXXK;
                }
            }
        }

        FXXK:
        int res = 0;
        while (pq.size()) {
            int k = pq.size();
            while (k--) {
                // 最先找到1的是赢家
                auto [i, j] = pq.front();
                pq.pop();

                for (auto& it : fx) {
                    int y = i + it[0], x = j + it[1];
                    if (y >= 0 && y < n && x >= 0 && x < m && grid[y][x] != 2) {
                        if (grid[y][x] == 1)
                            return res;
                        grid[y][x] = 2;
                        pq.push({y, x});
                    }
                }
            }
            ++res;
        }
        return -1;
    }
};
```
</div>

<div style="flex: 1; border: 1px solid black; padding: 10px;">

[2146. 价格范围内最高排名的 K 样物品](https://leetcode.cn/problems/k-highest-ranked-items-within-a-price-range/description/)

```C++
class Solution {
    const int fx[4][2] = {
        {-1, 0}, {0, -1}, {1, 0}, {0, 1}
    };
public:
    vector<vector<int>> highestRankedKItems(
        vector<vector<int>>& grid, 
        vector<int>& pricing, 
        vector<int>& start, int k) {
        int n = grid.size();
        int m = grid[0].size();
        int row = start[0], col = start[1];
        int low = pricing[0], high = pricing[1];
        // 距离 - 价格 - 坐标[i][j]
        vector<tuple<int, int, int, int>> tmp;
        queue<tuple<int, int, int>> pq; // 当前坐标[i][j] 距离
        if (low <= grid[row][col] && grid[row][col] <= high)
            tmp.push_back({0, grid[row][col], row, col});
        grid[row][col] = -grid[row][col];
        pq.push({row, col, 0});

        while (pq.size()) {
            auto [i, j, jl] = pq.front();
            pq.pop();

            ++jl;
            for (auto& it : fx) {
                int y = i + it[0], x = j + it[1];
                if (y >= 0 && y < n && x >= 0 && x < m && grid[y][x] > 0) {
                    if (low <= grid[y][x] && grid[y][x] <= high) {
                        tmp.push_back({jl, grid[y][x], y, x});
                    }
                    grid[y][x] = -grid[y][x];
                    pq.push({y, x, jl});
                }
            }
        }
        
        sort(tmp.begin(), tmp.end()); // 利用 tiple 从左到右 从小到大排序
        vector<vector<int>> res;
        for (int i = 0; i < k && i < tmp.size(); ++i) {
            auto [jl, jg, y, x] = tmp[i];
            res.push_back({y, x});
        }

        return res;
    }
};
```

</div>

<div style="flex: 1; border: 1px solid black; padding: 10px;">

[1765. 地图中的最高点](https://leetcode.cn/problems/map-of-highest-peak/description/) (*符合C++11的写法*) 我感觉已经非常的模版(标准)了

```C++
class Solution {
    const int fx[4][2] = {
        {0, 1}, {0, -1}, {1, 0}, {-1, 0}
    };
public:
    vector<vector<int>> highestPeak(
        vector<vector<int>>& isWater) {
        int n = isWater.size(), m = isWater[0].size();
        vector<vector<char>> vis(n, vector<char>(m)); // 标记
        // [i][j] - 高度
        queue<tuple<int, int, int>> pq;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                if (isWater[i][j]) {
                    vis[i][j] = 1;
                    isWater[i][j] = 0;
                    pq.push(make_tuple(i, j, 0));
                }
            }
        }

        while (pq.size()) {
            int i, j, h;
            tie(i, j, h) = pq.front();
            pq.pop();

            ++h;
            for (auto& it : fx) {
                int y = i + it[0], x = j + it[1];
                if (y >= 0 && y < n && x >= 0 && x < m && !vis[y][x]) {
                    isWater[y][x] = h;
                    vis[y][x] = 1;
                    pq.push(make_tuple(y, x, h));
                }
            }
        }

        return isWater;
    }
};
```

</div>

</div>
