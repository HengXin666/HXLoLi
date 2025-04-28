# 景区导游

题目: [景区导游](https://dashoj.com/d/lqbproblem/p/68)

树上倍增 (最近公共祖先问题 `pa[][]`的推导可以尝试这题: [1483. 树节点的第 K 个祖先 - 力扣（LeetCode）](https://leetcode.cn/problems/kth-ancestor-of-a-tree-node/description/) (再不济可以看看[【模板讲解】树上倍增算法（以及最近公共祖先）Python/Java/C++/Go](https://leetcode.cn/problems/kth-ancestor-of-a-tree-node/solutions/2305895/mo-ban-jiang-jie-shu-shang-bei-zeng-suan-v3rw/)))

然后就是把 单纯的 `(祖先)` 拓展为 `(祖先, 本节点到祖先的距离)`即可

不知道为什么这里提交不过, 我在[蓝桥杯2023年第十四届省赛真题-景区导游 - C语言网 (dotcpp.com)](https://www.dotcpp.com/oj/problem3156.html)提交是没问题的!😄

```cpp
#include <cstdio>
#include <vector>
#include <tuple>
#include <queue>
#include <utility>
#include <functional>

using namespace std;

using ll = long long;

// 获取二进制位数 
int getBit(int x) {
    int res = 0;
    while (x) {
        ++res;
        x >>= 1;
    }
    return res;
}

int main() {
    int n, k;
    scanf("%d %d", &n, &k);
    int nBit = getBit(n);
    vector<vector<pair<int, int>>> pa(n, vector<pair<int, int>>(nBit, make_pair(-1, -1)));
    vector<int> hArr(n, 0); // 深度 
    vector<vector<pair<int, int>>> G(n); // [u][v, w]
    for (int i = 1; i < n; ++i) {
        int u, v, w;
        scanf("%d %d %d", &u, &v, &w);
        --u, --v; // 映射到索引 
        G[u].push_back(make_pair(v, w));
        G[v].push_back(make_pair(u, w));
    }
    
    // 预处理
    function<void(int, int, int)> dfs = [&](int i, int p, int pw) {
        pa[i][0] = make_pair(p, pw);
        
        for (auto& it : G[i]) {
            int v, w;
            tie(v, w) = it;
            if (v != p) {
                hArr[v] = hArr[i] + 1;
                dfs(v, i, w);
            }
        }
    };
    
    // tmd 根是tm的誰?! (先假设是0) (好像是誰无所谓?)
    dfs(0, -1, -1);
    
    // dp 计算所有节点的2^b个祖先结点 以及到达他们的权 
    for (int b = 1; b < nBit; ++b) {
        for (int i = 0; i < n; ++i) {
            int v1, v2, w1, w2;
            tie(v1, w1) = pa[i][b - 1];
            if (v1 != -1) {
                tie(v2, w2) = pa[v1][b - 1];
                pa[i][b] = make_pair(v2, w1 + w2);
            }
        }
    }
    
    function<ll(int, int)> lac = [&](int a, int b) {
        if (hArr[a] < hArr[b]) { // 保证 a 是根深的结点 
            int tmp = a;
            a = b;
            b = tmp;
        }
        
        ll res = 0;
        int h = hArr[a] - hArr[b], bitH = 0;

        // 现在 把 a 向上移动 hArr[a] - hArr[b] 个距离, 保证 ab 处于同一层
        while (h) {
            if (h & 1) {
                int new_a, new_w;
                tie(new_a, new_w) = pa[a][bitH];
                if (new_a == -1)
                    break;
                res += new_w;
                a = new_a;
            }
            ++bitH;
            h >>= 1;
        }

        if (a == b) { // a 的最近祖先是 b ! 
            return res;
        }
        
        // 二分跳
        for (int t = pa[a].size() - 1; t >= 0; --t) {
            int av, aw, bv, bw;
            tie(av, aw) = pa[a][t];
            tie(bv, bw) = pa[b][t];
            if (av != bv) {
                a = av;
                b = bv;
                res += (aw + bw);
            }
        }
        int av, aw, bv, bw;
        tie(av, aw) = pa[a][0];
        tie(bv, bw) = pa[b][0];
        return res + aw + bw;
    };
    
    // 我进行询问
    ll sum = 0; // 总距离 
    vector<int> arr(k);
    
    for (int i = 0; i < k; ++i) {
        scanf("%d", &arr[i]);
        --arr[i];
        if (i > 0)
            sum += lac(arr[i - 1], arr[i]);
    }
    
    for (int i = 0; i < k; ++i) {
        if (i == 0) {
            printf("%lld ", sum - lac(arr[0], arr[1]));
        } else if (i == k - 1) {
            printf("%lld ", sum - lac(arr[i - 1], arr[i]));
        } else { 
            // a -> b -> c -> d
            // a -> c -> d (减去了-> b ->, 加上了 a->c)
            printf("%lld ", sum - lac(arr[i - 1], arr[i]) 
                                - lac(arr[i], arr[i + 1]) 
                                + lac(arr[i - 1], arr[i + 1])
            );
        }
    }
    
    return 0;
}
```
