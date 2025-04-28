# 3123. 最短路径中的边
链接: [3123. 最短路径中的边](https://leetcode.cn/problems/find-edges-in-shortest-paths/)

给你一个`n`个节点的无向带权图，节点编号为`0`到`n - 1`。图中总共有`m`条边，用二维数组`edges`表示，其中 $edges[i] = [a_i, b_i, w_i]$ 表示节点 $a_i$ 和 $b_i$ 之间有一条边权为 $w_i$ 的边。

对于节点`0`为出发点，节点`n - 1`为结束点的所有最短路，你需要返回一个长度为`m`的`boolean`数组`answer`，如果`edges[i]`至少 在其中一条最短路上，那么`answer[i]`为`true`，否则`answer[i]`为`false`。

请你返回数组`answer`。

注意，图可能不连通。

提示:
- 2 <= n <= 5 * $10^4$
- m == edges.length
- 1 <= m <= min(5 * $10^4$, n * (n - 1) / 2)
- 0 <= $a_i, b_i$ < n
- $a_i$ != $b_i$
- 1 <= $w_i$ <= $10^5$
- 图中没有重边。

# 题解
## 双向 $Dijkstra$
看了题解后: 我只能独立推出这个: $$dis[a] + w[x] + rdis[b] == dis[n - 1]$$ 具体代码注释有描述

特别注意的是, 我踩坑了qwq... 忘记把堆设置为**小根堆**了 (小数据没问题, 大数据会爆炸); 其次, 权忘记加之前的了(debug发现了qwq...)

模版还是不熟悉!

```C++
class Solution {
    const int inf = 1e9;
public:
    vector<bool> findAnswer(int n, vector<vector<int>>& edges) {
        // 如果 0 -> a --x--> b -> n
        // 那么一定是 dis[a] + rdis[b] + w[x] = dis[n]
        vector<vector<pair<int, int>>> G(n);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq; // 权, 点
        vector<int> dis(n, inf);
        vector<int> rdis(n, inf);
        vector<bool> res(edges.size());
        for (auto& it : edges) {
            G[it[0]].push_back({it[1], it[2]});
            G[it[1]].push_back({it[0], it[2]});
        }

        dis[0] = 0;
        pq.push({0, 0});

        while (pq.size()) {
            auto [w, u] = pq.top();
            pq.pop();
            if (w > dis[u])
                continue;
            
            for (auto& it : G[u]) {
                auto [v, dw] = it;
                dw += w;
                if (dw < dis[v]) {
                    dis[v] = dw;
                    pq.push({dw, v});
                }
            }
        }

        if (dis[n - 1] == inf) // 剪枝
            return res;

        rdis[n - 1] = 0;
        pq.push({0, n - 1});
        while (pq.size()) {
            auto [w, u] = pq.top();
            pq.pop();
            if (w > rdis[u])
                continue;
            
            for (auto& it : G[u]) {
                auto [v, dw] = it;
                dw += w;
                if (dw < rdis[v]) {
                    rdis[v] = dw;
                    pq.push({dw, v});
                }
            }
        }

        // dis[a] + rdis[b] + w[x] = dis[n]
        int i = 0;
        for (auto& it : edges) {
            if (dis[it[0]] > dis[it[1]]) {
                swap(it[0], it[1]);
            }
            
            if (dis[it[0]] + rdis[it[1]] + it[2] == dis[n - 1])
                res[i] = true;
            
            ++i;
        }

        return res;
    }
};
```

## $Dijkstra$ 最短路 + DFS/BFS 找边（Python/Java/C++/Go）

首先用 $Dijkstra$ 算法（堆优化版本）计算出起点 $0$ 到所有节点的最短路长度 $dis$。

如果 $dis[n−1]=∞$，说明无法从起点 $0$ 到终点 $n−1$，答案全为 $false$。

否则，我们可以从终点 $n−1$ 出发，倒着 DFS 或 BFS，设当前在点 $y$，邻居为 $x$，边权为 $w$，如果满足 $$dis[x]+w=dis[y]$$ 则说明 $x-y$ 这条边在从 $0$ 到 $n−1$ 的最短路上。

> 问: 为什么在求出最短路后，不能从起点 0 出发去寻找在最短路上的边?
>
> 答: 从起点 0 出发，当发现 $dis[x]+w=dis[y]$ 时，这仅仅意味着 $x-y$ 这条边在从 0 出发的最短路上，但这条最短路不一定通向终点 $n−1$ (比如图是一棵树，这条边不通往 $n−1$ )。而从终点 $n−1$ 出发倒着寻找，就能保证符合等式的边在通向终点的最短路上。

```C++
class Solution {
public:
    vector<bool> findAnswer(int n, vector<vector<int>>& edges) {
        vector<vector<tuple<int, int, int>>> g(n);
        for (int i = 0; i < edges.size(); i++) {
            auto& e = edges[i];
            int x = e[0], y = e[1], w = e[2];
            g[x].emplace_back(y, w, i);
            g[y].emplace_back(x, w, i);
        }

        vector<long long> dis(n, LLONG_MAX);
        dis[0] = 0;
        priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<>> pq;
        pq.emplace(0, 0);
        while (!pq.empty()) {
            auto [dx, x] = pq.top();
            pq.pop();
            if (dx > dis[x]) {
                continue;
            }
            for (auto [y, w, _] : g[x]) {
                int new_dis = dx + w;
                if (new_dis < dis[y]) {
                    dis[y] = new_dis;
                    pq.emplace(new_dis, y);
                }
            }
        }

        vector<bool> ans(edges.size());
        // 图不连通
        if (dis[n - 1] == LLONG_MAX) {
            return ans;
        }

        // 从终点出发 DFS
        vector<int> vis(n);
        function<void(int)> dfs = [&](int y) {
            vis[y] = true;
            for (auto [x, w, i] : g[y]) {
                if (dis[x] + w != dis[y]) {
                    continue;
                }
                ans[i] = true;
                if (!vis[x]) {
                    dfs(x);
                }
            }
        };
        dfs(n - 1);
        return ans;
    }
};
// 作者：灵茶山艾府
// 链接：https://leetcode.cn/problems/find-edges-in-shortest-paths/solutions/2749274/dijkstra-zui-duan-lu-dfsbfs-zhao-bian-py-yf48/
// 来源：力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```