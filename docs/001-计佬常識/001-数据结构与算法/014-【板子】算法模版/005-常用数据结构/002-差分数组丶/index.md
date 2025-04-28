# 差分数组

- 理论知识: [差分数组](https://blog.HXLoLi.com/blog/#/articles?articleId=20605 "##20605##")

## 一、一维有序集合上差分 (或者是哈希表+排序)
- 例题: [2251. 花期内花的数目](https://leetcode.cn/problems/number-of-flowers-in-full-bloom/)

```cpp
class Solution {
public:
    vector<int> fullBloomFlowers(
        vector<vector<int>>& flowers, 
        vector<int>& people
    ) {
        map<int, int> dArr;
        for (auto& q : flowers) {
            ++dArr[q[0]];
            --dArr[q[1] + 1];
        }
        int n = people.size();
        vector<tuple<int, int>> pp(n);
        for (int i = 0; i < n; ++i)
            pp[i] = {people[i], i};
        // 排序一下询问数组, 经典的处理离线问题的手段
        sort(pp.begin(), pp.end());
        vector<int> res(n);
        auto it = dArr.begin();
        for (int i = 0, s = 0; i < n; ++i) {
            auto [t, idx] = pp[i];
            while (it != dArr.end() && it->first <= t) {
                s += it->second;
                ++it;
            }
            res[idx] = s;
        }
        return res;
    }
};
```

## 二、二维差分的原地模版

- 例题: [2536. 子矩阵元素加 1](https://leetcode.cn/problems/increment-submatrices-by-one/)

> [!TIP]
> `二维差分`用`二维前缀和`恢复!

- 提升: [2132. 用邮票贴满网格图](https://leetcode.cn/problems/stamping-the-grid/)

```cpp
class Solution {
public:
    vector<vector<int>> rangeAddQueries(
        int n,
        vector<vector<int>>& queries
    ) {
        // + 1 是差分防止后项讨论
        // 再 + 1 是 前缀和防止对 idx = 0 的讨论
        vector<vector<int>> dArr(n + 2, vector<int>(n + 2));
        for (auto& q : queries) {
            ++dArr[q[0] + 1][q[1] + 1];
            --dArr[q[2] + 2][q[1] + 1];
            --dArr[q[0] + 1][q[3] + 2];
            ++dArr[q[2] + 2][q[3] + 2];
        }
        // 前缀和 恢复
        for (int i = 1; i <= n; ++i) {
            for (int j = 1; j <= n; ++j) {
                dArr[i][j] += dArr[i][j - 1] + dArr[i - 1][j] - dArr[i - 1][j - 1];
            }
        }
        dArr.pop_back();
        dArr.erase(dArr.begin());
        for (auto& it : dArr) {
            it.pop_back();
            it.erase(it.begin());
        }
        return dArr;
    }
};
```

## 三、二维离线差分

- [LCP 74. 最强祝福力场](https://leetcode.cn/problems/xepqZ5/)

- [850. 矩形面积 II](https://leetcode.cn/problems/rectangle-area-ii/) (不确定)

> 暂时没有学习

