# [蓝桥杯 2017 国 B] 发现环

## 题目描述

小明的实验室有 $N$ 台电脑，编号 $1 \sim N$。原本这 $N$ 台电脑之间有 $N-1$ 条数据链接相连，恰好构成一个树形网络。在树形网络上，任意两台电脑之间有唯一的路径相连。

不过在最近一次维护网络时，管理员误操作使得某两台电脑之间增加了一条数据链接，于是网络中出现了环路。环路上的电脑由于两两之间不再是只有一条路径，使得这些电脑上的数据传输出现了 BUG。

为了恢复正常传输。小明需要找到所有在环路上的电脑，你能帮助他吗？

## 输入格式

第一行包含一个整数 $N$。

以下 $N$ 行每行两个整数 $a$ 和 $b$，表示 $a$ 和 $b$ 之间有一条数据链接相连。

输入保证合法。

## 输出格式

按从小到大的顺序输出在环路上的电脑的编号，中间由一个空格分隔。

## 样例 #1

### 样例输入 #1

```
5
1 2
3 1
2 4
2 5
5 3
```

### 样例输出 #1

```
1 2 3 5
```

## 提示

对于 $30\%$ 的数据， $1 \le N \le 1000$。

对于 $100\%$ 的数据， $1 \le N \le 10^5$，$1 \le a,b \le N$。

时限 1 秒, 256M。蓝桥杯 2017 年第八届国赛

# 题解
显然拓扑排序

## 八嘎1

```C++
#include <cstdio>
#include <vector>
#include <queue>

using namespace std;

int main() {
    int n;
    scanf("%d", &n);
    vector<vector<int>> G(n);
    vector<int> du(n); // 度 数组 
    
    for (int i = 0, v, u; i < n; ++i) {
        scanf("%d %d", &v, &u);
        G[--v].push_back(--u); // v -> u 有边
        G[u].push_back(v);     // u -> v (无向图)
        ++du[v];
        ++du[u];
    }
    
    // 拓扑排序
    queue<int> Q; // 度为 1 则入 
    
    for (int i = 0; i < n; ++i) {
        if (du[i] == 1) {
            Q.push(i);
            --du[i];
            
            for (int& it : G[i]) {
                --du[it];
            }
        }
    }
    
    vector<bool> arr(n);
    
    while (Q.size()) {
        int now_i = Q.front();
        Q.pop();
        arr[now_i] = 1;
        
        for (int& it : G[now_i]) {
            if (du[it] == 1) {
                Q.push(it);
            }
        }
    }
    
    for (int i = 0; i < n; ++i)
        if (!arr[i])
            printf("%d ", i + 1);
            
    return 0;
}
```

## AC

> 还是那句话, 度为1的, 就永远不会遇到了, 所以对自己不用减度, 而对自己的出度进行减度
>
> 对于 a <-> b <-> c <-> d
>
> 有
|a|b|c|d|
|:-:|:-:|:-:|:-:|
|1*|2|2|1*|
>
> (外部for) 队列为: 入 -> [d, a] -> 出
>
> >对于`a`点, 进行一轮for后: a <- b <-> c <-> d
|a|b|c|d|
|:-:|:-:|:-:|:-:|
|1*|1*|2|1*|
> >
> > 队列为: 入 -> [b, d] -> 出
> 
> > **同理**, 对于`d`点, 进行一轮for后: a <- b <-> c -> d
|a|b|c|d|
|:-:|:-:|:-:|:-:|
|1*|1*|1*|1*|
> >
> > 队列为: 入 -> [c, b] -> 出
>
> > 此时, 我们需要的来啦: <b style=color:red>为什么只需要在 内for, `--度[it]`?</b>
> >
> > 对于`b`点, 进行一轮for后: a | b <- c -> d
|a|b|c|d|
|:-:|:-:|:-:|:-:|
|0|1*|0|1*|
> >
> > 这就是为什么它不需要对`a`点处理, 但也不会回去的原因: 如果之前为`1`, 那么再次选到它的时候已经是`--1`即`0`了

```C++
#include <cstdio>
#include <vector>
#include <queue>

using namespace std;

int main() {
    int n;
    scanf("%d", &n);
    vector<vector<int>> G(n);
    vector<int> du(n); // 度 数组 
    
    for (int i = 0, v, u; i < n; ++i) {
        scanf("%d %d", &v, &u);
        G[--v].push_back(--u); // v -> u 有边
        G[u].push_back(v);     // u -> v (无向图)
        ++du[v];
        ++du[u];
    }
    
    // 拓扑排序
    queue<int> Q; // 度为 1 则入 
    
    for (int i = 0; i < n; ++i) {
        if (du[i] == 1) {
            Q.push(i);
        }
    }
    
    vector<bool> arr(n);
    
    while (Q.size()) {
        int now_i = Q.front();
        Q.pop();
        arr[now_i] = 1;
        
        for (int& it : G[now_i]) {
            if (--du[it] == 1) {
                Q.push(it);
            }
        }
    }
    
    for (int i = 0; i < n; ++i) // 输出
        if (!arr[i])
            printf("%d ", i + 1);
            
    return 0;
}
```
