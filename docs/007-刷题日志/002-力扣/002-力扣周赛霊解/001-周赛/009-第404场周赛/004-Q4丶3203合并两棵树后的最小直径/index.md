# 3203. 合并两棵树后的最小直径

链接: [3203. 合并两棵树后的最小直径](https://leetcode.cn/problems/find-minimum-diameter-after-merging-two-trees/)

给你两棵 无向 树，分别有 n 和 m 个节点，节点编号分别为 0 到 n - 1 和 0 到 m - 1 。给你两个二维整数数组 edges1 和 edges2 ，长度分别为 n - 1 和 m - 1 ，其中 edges1[i] = [ai, bi] 表示在第一棵树中节点 ai 和 bi 之间有一条边，edges2[i] = [ui, vi] 表示在第二棵树中节点 ui 和 vi 之间有一条边。

你必须在第一棵树和第二棵树中分别选一个节点，并用一条边连接它们。

请你返回添加边后得到的树中，最小直径 为多少。

一棵树的 直径 指的是树中任意两个节点之间的最长路径长度。

# 题解
## 剥洋葱大法: 找树心
实际就是剥洋葱，对于是无向图降边后的树，只要是无方向的搜索都建议用这个思路，又快又简单。

具体原理就是找到这个树最外层的节点，也就是出入度只有 1 的节点，然后全部删掉，再重新搜索外层循环步骤，那么最后肯定只有两个情况：剩一个节点，剩两个节点。前者就是整个树的最中心的节点，后者就是数中心有两个节点。

本来不需要理会这个结果，因为正常都是用作枚举，但这题需要找出树的直径，那么剥了多少层等于半径，如果数心只有一个节点，那么`直径 == 半径 * 2`，如果树心是两个节点，等于中间不是节点是一条边，那么`直径 = 半径 * 2 - 1`。

那么直接写就可以写函数输出需要有两个参数半径与树心数，但由于 * 2 必然是偶数再-1 必然奇数，所以实际可以单输出当前树的直径即可，反过来通过奇偶判断数心树恢复半径；

(比赛的时候思路类似, 但是死活调不出来, 本篇思路: [【详解】剥洋葱大法好](https://leetcode.cn/problems/find-minimum-diameter-after-merging-two-trees/solutions/2827577/xiang-jie-bo-yang-cong-da-fa-hao-by-l00-yp6l))
```C++
class Solution {
        int fk(vector<vector<int>>& edges) { // 拓扑排序洋葱
            if (!edges.size()) // 可以不写
                return 0;
            int n = edges.size() + 1;
            vector<vector<int>> G(n);
            vector<int> du(n);
            for (auto& it : edges) {
                G[it[0]].push_back(it[1]);
                G[it[1]].push_back(it[0]);
                ++du[it[0]], ++du[it[1]];
            }

            // 拓扑排序
            queue<int> Q; // 度为 1 则入 
            for (int i = 0; i < n; ++i)
                if (du[i] == 1)
                    Q.push(i);

            int res = 0, tmp = 0;
            while (Q.size() > 1) { // 剩下心
                ++res;
                int now_n = tmp = Q.size(); // 当前层有多少结点
                for (int i = 0; i < now_n; ++i) {
                    int x = Q.front();
                    Q.pop();
                    // 出度清除
                    for (auto& it : G[x])
                        if (--du[it] == 1)
                            Q.push(it);
                }
            }
            return res * 2 - (Q.size() ^ 1);
        }
public:
    int minimumDiameterAfterMerge(
        vector<vector<int>>& edges1, vector<vector<int>>& edges2) {
        int res1 = fk(edges1), res2 = fk(edges2);
        return max({res1, res2, (res1 + 1) / 2 + (res2 + 1) / 2 + 1});
    }
};
```

我的八嘎代码: wa于: `[[0,1],[2,0],[3,2],[3,6],[8,7],[4,8],[5,4],[3,5],[3,9]]` | `[[0,1],[0,2],[0,3]]` | 输出: `6` | 答案: `7`

```C++
class Solution {
        int fk(vector<vector<int>>& edges) {
            if (edges.size() == 0)
                return 1;
            int n = edges.size() + 1;
            vector<vector<int>> G(n);
            vector<int> du(n);
            for (auto& it : edges) {
                G[it[0]].push_back(it[1]);
                G[it[1]].push_back(it[0]);
                ++du[it[0]], ++du[it[1]];
            }

            // 拓扑排序
            queue<int> Q; // 度为 1 则入 
            
            for (int i = 0; i < n; ++i) {
                if (du[i] == 1) {
                    Q.push(i);
                }
            }

            int res = 0, tmp = 0;
            while (Q.size()) {
                ++res;
                int now_n = tmp = Q.size(); // 当前层有多少结点

                for (int i = 0; i < now_n; ++i) {
                    int x = Q.front();
                    Q.pop();

                    // 出度清除
                    // 为什么不用清除入度呢? 因为之前已经访问了, 以后都访问不到了
                    for (auto& it : G[x]) {
                        --du[it];
                        if (du[it] == 1)
                            Q.push(it);
                    }
                }
            }
            cout << res << '|' << tmp << '\n';
            return res + tmp - 1;
        }
public:
    int minimumDiameterAfterMerge(
        vector<vector<int>>& edges1, 
        vector<vector<int>>& edges2) {
        if (edges1.size() == 0 && edges2.size() == 0)
            return 1;
        int res1 = fk(edges1), res2 = fk(edges2);
        return res1 + res2 - 1;
    }
};
```



## 树形DP
不会