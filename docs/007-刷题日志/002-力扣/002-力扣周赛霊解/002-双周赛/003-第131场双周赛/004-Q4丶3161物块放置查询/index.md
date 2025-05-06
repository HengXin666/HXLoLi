# 3161. 物块放置查询
链接: [3161. 物块放置查询](https://leetcode.cn/problems/block-placement-queries/)

有一条无限长的数轴，原点在 0 处，沿着 x 轴 正 方向无限延伸。

给你一个二维数组 queries ，它包含两种操作：

- 操作类型 1 ：queries[i] = [1, x] 。在距离原点 x 处建一个障碍物。数据保证当操作执行的时候，位置 x 处 没有 任何障碍物。

- 操作类型 2 ：queries[i] = [2, x, sz] 。判断在数轴范围 [0, x] 内是否可以放置一个长度为 sz 的物块，这个物块需要 完全 放置在范围 [0, x] 内。如果物块与任何障碍物有重合，那么这个物块 不能 被放置，但物块可以与障碍物刚好接触。注意，你只是进行查询，并 不是 真的放置这个物块。每个查询都是相互独立的。

请你返回一个 boolean 数组results ，如果第 i 个操作类型 2 的操作你可以放置物块，那么 results[i] 为 true ，否则为 false 。

提示:
- 1 <= queries.length <= 15 * 10^4
- 2 <= queries[i].length <= 3
- 1 <= queries[i][0] <= 2
- 1 <= x, sz <= min(5 * 10^4, 3 * queries.length)
- 输入保证操作 1 中，x 处不会有障碍物。
- 输入保证至少有一个操作类型 2 。

# 题解
> 我不会...

## 三种方法：线段树/树状数组/并查集

0x3f: [三种方法：线段树/树状数组/并查集（Python/Java/C++/Go）](https://leetcode.cn/problems/block-placement-queries/solutions/2790395/ping-heng-shu-xian-duan-shu-pythonjavacg-8klz)