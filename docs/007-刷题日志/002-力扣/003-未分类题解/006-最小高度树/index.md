# 310. 最小高度树
原题: [310. 最小高度树](https://leetcode.cn/problems/minimum-height-trees/description/)

树是一个无向图，其中任何两个顶点只通过一条路径连接。 换句话说，一个任何没有简单环路的连通图都是一棵树。

给你一棵包含 n 个节点的树，标记为 0 到 n - 1 。给定数字 n 和一个有 n - 1 条无向边的 edges 列表（每一个边都是一对标签），其中 edges[i] = [$a_i$, $b_i$ ] 表示树中节点 $a_i$ 和 $b_i$ 之间存在一条无向边。

可选择树中任何一个节点作为根。当选择节点 x 作为根节点时，设结果树的高度为 h 。在所有可能的树中，具有最小高度的树（即，min(h)）被称为 最小高度树 。

请你找到所有的 最小高度树 并按 任意顺序 返回它们的根节点标签列表。

树的 高度 是指根节点和叶子节点之间最长向下路径上边的数量。
 
```c in
示例 1：

输入：n = 4, edges = [[1,0],[1,2],[1,3]]
输出：[1]
解释：如图所示，当根是标签为 1 的节点时，树的高度是 1 ，这是唯一的最小高度树。

示例 2：

输入：n = 6, edges = [[3,0],[3,1],[3,2],[3,4],[5,4]]
输出：[3,4]
```

提示:

$
1 <= n <= 2 * 10^4\\
edges.length == n - 1\\
0 <= a_i, b_i < n\\
a_i != b_i\\
所有 (ai, bi) 互不相同\\
给定的输入 保证 是一棵树，并且 不会有重复的边
$

# 题解
## 我的代码

直接弗洛伊德算法, 得出所有点到所有点的距离, 然后使得点到叶子的距离最小即可

时间复杂度: $O(N^3)$ 显然爆炸了

```C++
class Solution {
    
public:
    vector<int> findMinHeightTrees(int n, vector<vector<int>>& edges) {
        // 弗洛伊德算法 得出所有点到所有点的最短路径
        static const int INF = 1e7;
        vector<int> res;
        vector<vector<int>> forld(n, vector<int>(n, INF));
        for (auto& it : edges) {
            forld[it[0]][it[1]] = 1;
            forld[it[1]][it[0]] = 1;
        }
        for (int k = 0; k < n; ++k) {
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    if (i == j || i == k || j == k)
                        continue;

                    if (forld[i][j] > forld[i][k] + forld[k][j]) {
                        forld[i][j] = forld[i][k] + forld[k][j];
                    }
                }
            }
        }

        int maxH = INF;
        // int minH = INF;
        for (int i = 0; i < n; ++i) {
            int tmpMaxH = 0;
            int tmpMinH = INF;
            for (int j = 0; j < n; ++j) {
                if (forld[i][j] == INF)
                    continue;
                
                tmpMaxH = max(tmpMaxH, forld[i][j]);
                // tmpMinH = min(tmpMinH, forld[i][j]);
            }

            if (tmpMaxH < maxH) {
                maxH = tmpMaxH;
            }
        }

        for (int i = 0; i < n; ++i) {
            int tmpMaxH = 0;
            for (int j = 0; j < n; ++j) {
                if (forld[i][j] == INF)
                    continue;
                
                tmpMaxH = max(tmpMaxH, forld[i][j]);
            }

            if (tmpMaxH == maxH)
                res.push_back(i);
        }

        return res;
    }
};
```

## 正解
通过观察可以知道:
- 答案要么只有一个节点, 要么是对称的有两个节点.
- 并且是在树的深处(靠中间的位置)

所以, 我们可以使用拓扑排序, 像削苹果一样, 把它一点一点的从外往内剥开.

这里处理的非常巧妙! 因为平常的拓扑排序你是不知道一层是多少的, 结束的时候就已经是全部剥完了, 但是我们需要的是最后一层(里面的核), 所以我们可以记录每一层的数量`Q.size()`, 然后在循环里面剥, *如果还可以剥, 就清空之前的记录, 继续剥(见AC代码)*.

### 我的代码
还是tm的超时...

```C++
class Solution {
public:
    vector<int> findMinHeightTrees(int n, vector<vector<int>>& edges) {
        if (n == 1)
            return {0};
        
        // 拓扑排序, 对于无向图, 那么就是度 >= 2 则是非叶子节点
        // 邻接表
        vector<list<int>> GTbale(n, list<int>());
        vector<int> d(n); // 度
        for (auto& it: edges) {
            ++d[it[0]];
            ++d[it[1]];
            GTbale[it[0]].push_back(it[1]);
            GTbale[it[1]].push_back(it[0]);
        }

        // 进行拓扑排序
        queue<int> Q;
        for (int i = 0; i < n; ++i) {
            if (d[i] == 1) {
                Q.push(i);
            }
        }

        vector<int> res;
        while (Q.size()) {
            res.clear(); // 清空
            int now_n = Q.size(); // 当前层有多少结点

            for (int i = 0; i < now_n; ++i) {
                int x = Q.front();
                Q.pop();

                // 出度清除
                GTbale[x] = list<int>();
                --d[x];
                res.push_back(x);

                // 入度清除
                for (int j = 0; j < n; ++j) {
                    for (auto& it : GTbale[j]) {
                        if (it == x) {
                            --d[j];
                            if (d[j] == 1)
                                Q.push(j);
                            break;
                        }
                    }
                }
            }
        }

        return res;
    }
};
```

### AC

对于为什么下面代码这样写, 也可以的原因是:

- 原本上面的代码有效的实际上只有for循环中`j == x`时候才有用, 所以直接写成下面就OK!

```C++
class Solution {
public:
    vector<int> findMinHeightTrees(int n, vector<vector<int>>& edges) {
        if (n == 1)
            return {0};
        
        // 拓扑排序, 对于无向图, 那么就是度 >= 2 则是非叶子节点
        // 邻接表
        vector<list<int>> GTbale(n, list<int>());
        vector<int> d(n); // 度
        for (auto& it: edges) {
            ++d[it[0]];
            ++d[it[1]];
            GTbale[it[0]].push_back(it[1]);
            GTbale[it[1]].push_back(it[0]);
        }

        // 进行拓扑排序
        queue<int> Q;
        for (int i = 0; i < n; ++i) {
            if (d[i] == 1) {
                Q.push(i);
            }
        }

        vector<int> res;
        while (Q.size()) {
            res.clear(); // 清空
            int now_n = Q.size(); // 当前层有多少结点

            for (int i = 0; i < now_n; ++i) {
                int x = Q.front();
                Q.pop();

                // --d[x]; // 不是必须的
                res.push_back(x);

                // 出度清除
                // 为什么不用清除入度呢? 因为之前已经访问了, 以后都访问不到了
                for (auto& it : GTbale[x]) {
                    --d[it];
                    if (d[it] == 1)
                        Q.push(it);
                }
            }
        }

        return res;
    }
};
```
