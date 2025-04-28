# 岛屿个数
题目: [岛屿个数](https://dashoj.com/d/lqbproblem/p/4)

## 题解

1. 先 DFS 把整个岛入队
2. 把入队的进行 BFS, 尝试寻找最外层(出地图边界), 这样说明没有被包围

注: BFS的方向是8个方向, DFS是4个方向

*特别注意DFS与BFS需要分别使用不同的标记数组!*

例如, 这样是 3 个岛屿, 而不是 2 个; 所以我们可以通过斜边跑路!
```C++
111111
100001
020301
100001
111111
```

代码:
```C++
#include <cstdio>
#include <vector>
#include <tuple>
#include <queue>
#include <utility>
#include <functional>

using namespace std;

const int x_fx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
const int y_fx[8] = {0, 1, 0, -1, 1, 1, -1, -1};

void hx() {
    int n, m;
    scanf("%d %d", &n, &m);
    vector<vector<char>> tizi(n, vector<char>(m));
    for (int i = 0; i < n; ++i) {
        char tmp[128];
        scanf("%s", tmp);
        for (int j = 0; j < m; ++j) {
            tizi[i][j] = tmp[j];
        }
    }
    
    int res = 0;
    vector<vector<char>> vis(n, vector<char>(m));
    queue<pair<int, int>> pq;
    
    function<void(int, int)> dfs = [&](int i, int j) {
        if (i < 0 || j < 0 || i >= n || j >= m || vis[i][j] || tizi[i][j] == '0')
            return;
        
        vis[i][j] = 1;
        pq.push(make_pair(i, j));
        
        for (int k = 0; k < 4; ++k) {
            dfs(i + y_fx[k], j + x_fx[k]);
        }
    };
    
    function<void()> bfs = [&]() {
        vector<vector<char>> kairyu(n, vector<char>(m));
        
        while (pq.size()) {
            int i, j;
            tie(i, j) = pq.front();
            kairyu[i][j] = 1;
            pq.pop();
            
            // 向八方向跑
            for (int k = 0; k < 8; ++k) {
                int y = i + y_fx[k];
                int x = j + x_fx[k];
                if (x < 0 || y < 0 || y >= n || x >= m) { // 逃出去海了, 直接返回! 
                    ++res;
                    pq = queue<pair<int, int>>(); // 清空
                    return;
                }
                
                if (!kairyu[y][x] && tizi[y][x] == '0') {
                    kairyu[y][x] = 1;
                    pq.push(make_pair(y, x));
                }
            }
        }
    };
    
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            if (tizi[i][j] == '1') {
                dfs(i, j); // dfs 把整个岛放进去
                bfs();     // bfs 岛上的每一个点尝试跑出地图外 (如果可以跑出去, 说明没有被岛包围) 
            }
        }
    }
    
    printf("%d\n", res);
}

int main() {
    int t;
    scanf("%d", &t);
    while (t--)
        hx();
        
    return 0;
}
```