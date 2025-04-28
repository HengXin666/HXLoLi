# 整数删除（蓝桥杯C/C++2023B组省赛）

题目描述
给定一个长度为 N的整数数列: $A_1,A_2,...,A_n$。

你要重复以下操作 $K$ 次：

每次选择数列中最小的整数（如果最小值不止一个，选择最前的），将其删除。并把与它相邻的整数加上被删除的数值。输出K次操作后的序列。

## 输入格式
第一行包含两个整数N和K。第二行包含 N 个整数, $A_1,A_2,...,A_n$。

## 输出格式
输出 n－k 个整数，中间用一个空格隔开，代表 K 次操作后的序列。

# AC

```C++
#include <cstdio>
#include <queue>
#include <vector>
#include <utility>
#include <functional>
#include <list>

using namespace std;

using ll = long long;

int main() {
    int n, k;
    scanf("%d %d", &n, &k);

    auto cmp = [](const pair<ll, pair<int, list<ll>::iterator>>& a, 
                  const pair<ll, pair<int, list<ll>::iterator>>& b) {
        if (a.first == b.first)
            return a.second.first > b.second.first;
        return a.first > b.first;
    }; // key 从小到大, 相同 则 val 从小到大 
    
    list<ll> arr;
    priority_queue<
        pair<ll, pair<int, list<ll>::iterator>>, 
        vector<pair<ll, pair<int, list<ll>::iterator>>>,
        decltype(cmp)> pq(cmp);
    
    for (int i = 0; i < n; ++i) {
        ll j;
        scanf("%lld", &j);
        arr.push_back(j);
        pq.push(make_pair(j, make_pair(i, --arr.end())));
    }
    
    while (k--) {
        auto x = pq.top();
        pq.pop();
        
        if (x.first != *(x.second.second)) {
            pq.push(make_pair(*(x.second.second), make_pair(x.second.first, x.second.second)));
            ++k;
            continue;
        }
        
        // f - s
        // 对 arr 进行修改:
        // 删除了 x
        // 获取 这个数的前一个数 
        auto mae = x.second.second;
        if (arr.begin() != mae) { // 给前面一个数进行加 
            --mae;
            *mae += *(x.second.second);
        }
        
        auto next = x.second.second;
        ++next;
        if (next != arr.end()) { // 给后面一个数进行加 
            *next += *(x.second.second);
        }
        
        // 移除自己 
        arr.erase(x.second.second);
//        printf("%lld [%d, %p]\n", x.first, x.second.first, x.second.first);
    }
    
    for (ll& it : arr)
        printf("%lld ", it);
    
    return 0;
}
```
