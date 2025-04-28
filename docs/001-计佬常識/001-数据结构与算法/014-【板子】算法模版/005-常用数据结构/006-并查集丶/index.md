# 并查集
## 一、板子合集
### 1.1 基础板子 ( cp $0\times3F$ )

```C++
class UnionFind {
    vector<int> fa; // 代表元
    vector<int> sz; // 集合大小

public:
    int cc; // 连通块个数

    UnionFind(int n) : fa(n), sz(n, 1), cc(n) {
        // 一开始有 n 个集合 {0}, {1}, ..., {n-1}
        // 集合 i 的代表元是自己，大小为 1
        ranges::iota(fa, 0); // iota(fa.begin(), fa.end(), 0);
    }

    // 返回 x 所在集合的代表元
    // 同时做路径压缩，也就是把 x 所在集合中的所有元素的 fa 都改成代表元
    int find(int x) {
        // 如果 fa[x] == x，则表示 x 是代表元
        if (fa[x] != x) {
            fa[x] = find(fa[x]); // fa 改成代表元
        }
        return fa[x];
    }

    // 判断 x 和 y 是否在同一个集合
    bool is_same(int x, int y) {
        // 如果 x 的代表元和 y 的代表元相同，那么 x 和 y 就在同一个集合
        // 这就是代表元的作用：用来快速判断两个元素是否在同一个集合
        return find(x) == find(y);
    }

    // 把 from 所在集合合并到 to 所在集合中
    // 返回是否合并成功
    bool merge(int from, int to) {
        int x = find(from), y = find(to);
        if (x == y) { // from 和 to 在同一个集合，不做合并
            return false;
        }
        fa[x] = y; // 合并集合。修改后就可以认为 from 和 to 在同一个集合了
        sz[y] += sz[x]; // 更新集合大小（注意集合大小保存在代表元上）
        // 无需更新 sz[x]，因为我们不用 sz[x] 而是用 sz[find(x)] 获取集合大小，但 find(x) == y，我们不会再访问 sz[x]
        cc--; // 成功合并，连通块个数减一
        return true;
    }

    // 返回 x 所在集合的大小
    int get_size(int x) {
        return sz[find(x)]; // 集合大小保存在代表元上
    }
};
```

### 1.2 极简板子

简单板子, 不需要计算集合大小、连通块数的

```C++
int n = /* 元素个数 */;
vector<int> fa(n);
iota(fa.begin(), fa.end(), 0);
auto find = [&](this auto&& find, int x) -> int {
    return x != fa[x] ? fa[x] = find(fa[x]) : x;
};
auto isSame = [&](int x, int y) {
    return find(x) == find(y);
};
auto merge = [&](int from, int to) {
    int x = find(from);
    int y = find(to);
    if (x == y)
        return;
    fa[x] = y;
};
```

## 二、题型
### 2.0 裸题

- 略

### 2.1 对于点

- [947. 移除最多的同行或同列石头](https://leetcode.cn/problems/most-stones-removed-with-same-row-or-column/)

对于`横坐标`**或**`纵坐标`相等即可合并的情况, 我们可以通过数据范围, 然后把两个点合并为一个点:

```C++
class UnSet {
    unordered_map<int, int> fa;
    int cnt = 0;
public:
    int find(int x) {
        if (!fa.count(x)) {
            ++cnt; // 连通块数 + 1
            return fa[x] = x;
        }
        return [this](this auto&& _find, int x) -> int {
            return x == fa[x] ? x : fa[x] = _find(fa[x]);
        }(x);
    }

    void merag(int from, int to) {
        int x = find(from);
        int y = find(to);
        if (x == y)
            return;
        fa[x] = y;
        --cnt; // 合并了, 连通块数 - 1
    }

    bool isSame(int x, int y) {
        return find(x) == find(y);
    }

    int getCnt() const {
        return cnt;
    }
};

class Solution {
public:
    int removeStones(vector<vector<int>>& stones) {
        UnSet unset;
        for (auto const& p : stones) {
            int x = p[0], y = p[1];
            // 该行和该列 都属于同一个并查集
            unset.merag(x + 10001, y);
            // 如果 行 或者 列 相同, 那就是一个连通块的
        }
        // 总数 - 连通块数(一个连通块只需要剩下一个石头) = 最多可以移除的石子的数量
        return stones.size() - unset.getCnt();
    }
};
```
