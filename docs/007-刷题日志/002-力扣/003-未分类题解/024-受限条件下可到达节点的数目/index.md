# 2368. 受限条件下可到达节点的数目
原题: [2368. 受限条件下可到达节点的数目](https://leetcode.cn/problems/reachable-nodes-with-restrictions/description/)

第 305 场周赛 Q2 - $1477$ - 中等

现有一棵由 n 个节点组成的无向树，节点编号从 0 到 n - 1 ，共有 n - 1 条边。

给你一个二维整数数组 edges ，长度为 n - 1 ，其中 edges[i] = [ai, bi] 表示树中节点 ai 和 bi 之间存在一条边。另给你一个整数数组 restricted 表示 受限 节点。

在不访问受限节点的前提下，返回你可以从节点 0 到达的 最多 节点数目。

注意，节点 0 不 会标记为受限节点。

## 示例 1：

输入：n = 7, edges = [[0,1],[1,2],[3,1],[4,0],[0,5],[5,6]], restricted = [4,5] <br>
输出：4 <br>
解释：上图所示正是这棵树。 <br>
在不访问受限节点的前提下，只有节点 [0,1,2,3] 可以从节点 0 到达。

## 示例 2：

输入：n = 7, edges = [[0,1],[0,2],[0,5],[0,4],[3,2],[6,5]], restricted = [4,2,1] <br>
输出：3 <br>
解释：上图所示正是这棵树。 <br>
在不访问受限节点的前提下，只有节点 [0,5,6] 可以从节点 0 到达。

提示：

$
2 <= n <= 10^5\\
edges.length == n - 1\\
edges[i].length == 2\\
0 <= ai, bi < n\\
ai != bi\\
edges 表示一棵有效的树\\
1 <= restricted.length < n\\
1 <= restricted[i] < n\\
restricted 中的所有值 互不相同
$

## 题解
可以使用 并查集/dfs/bfs

### 树上dfs

注意, 要标记 `baba`父结点(保证访问是从0往下的), 不然会产生回路 (不需要标记那些是访问过的)

```图示 By Heng_Xin
    baba
     | ↑   it_1
     ↓ |  ↗
   now_i -> it_2
          ↘
            it_n
```

```C++
class Solution {
public:
    int dfs(int now_i, int baba, unordered_set<int>& nodes, vector<vector<int>>& G) {
        int res = 1;
        for (auto& it : G[now_i]) {
            if (it != baba)
                res += dfs(it, now_i, nodes, G);
        }

        return res;
    }

    int reachableNodes(int n, vector<vector<int>>& edges, vector<int>& restricted) {
        unordered_set<int> nodes(restricted.begin(), restricted.end());
        vector<vector<int>> G(n);
        for (auto& it : edges) { // 在这里阻断
            if (nodes.find(it[0]) == nodes.end() &&
            nodes.find(it[1]) == nodes.end()) {
                G[it[0]].push_back(it[1]);
                G[it[1]].push_back(it[0]);
            }
        }

        return dfs(0, -1, nodes, G);
    }
};
```
