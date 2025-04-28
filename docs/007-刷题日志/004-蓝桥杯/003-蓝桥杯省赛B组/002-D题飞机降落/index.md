# P9241 [蓝桥杯 2023 省 B] 飞机降落
链接: [P9241 [蓝桥杯 2023 省 B D题] 飞机降落](https://www.luogu.com.cn/problem/P9241)

# [蓝桥杯 2023 省 B] 飞机降落

## 题目描述

$N$ 架飞机准备降落到某个只有一条跑道的机场。其中第 $i$ 架飞机在 $T_{i}$ 时刻到达机场上空，到达时它的剩余油料还可以继续盘旋 $D_{i}$ 个单位时间，即它最早可以于 $T_{i}$ 时刻开始降落，最晩可以于 $T_{i}+D_{i}$ 时刻开始降落。降落过程需要 $L_{i}$ 个单位时间。

一架飞机降落完毕时，另一架飞机可以立即在同一时刻开始降落，但是不能在前一架飞机完成降落前开始降落。

请你判断 $N$ 架飞机是否可以全部安全降落。

## 输入格式

输入包含多组数据。

第一行包含一个整数 $T$，代表测试数据的组数。

对于每组数据，第一行包含一个整数 $N$。

以下 $N$ 行，每行包含三个整数 $T_{i},D_{i},L_{i}$。

## 输出格式

对于每组数据，输出 `YES` 或者 `NO`，代表是否可以全部安全降落。

## 样例 #1

### 样例输入 #1

```
2
3
0 100 10
10 10 10
0 2 20
3
0 10 20
10 10 20
20 10 20
```

### 样例输出 #1

```
YES
NO
```

## 提示

**【样例说明】**

对于第一组数据，可以安排第 3 架飞机于 0 时刻开始降落，20 时刻完成降落。安排第 2 架飞机于 20 时刻开始降落，30 时刻完成降落。安排第 1 架飞机于 30 时刻开始降落，40 时刻完成降落。

对于第二组数据，无论如何安排，都会有飞机不能及时降落。

**【评测用例规模与约定】**

对于 $30 \%$ 的数据， $N \leq 2$。

对于 $100 \%$ 的数据， $1 \leq T \leq 10$，$1 \leq N \leq 10$，$0 \leq T_{i},D_{i},L_{i} \leq 10^{5}$。 

蓝桥杯 2023 省赛 B 组 D 题。

# 题解
我贪心不出来, 所以直接bfs回溯暴力即可(看了范围也差不多 $A_{10}^{10} * 10 * 10 * 2$ )

AC代码
```C++
#include <cstdio>
#include <vector>
#include <algorithm>

using namespace std;

// 降落不算在盘旋的时间内 
// 暴力回溯 A{10}{10} = 10! = 3628800 * 100 = 勉强? 
bool res = false;
void huisu(int now_i, int now_time, vector<vector<int>>& arr, vector<bool>& arrUp) {
    bool tag = 1;
    for (int i = 0; i < arr.size(); ++i) {
        if (arrUp[i])
            continue;
        if (now_time - arr[i][0] <= arr[i][1]) { // 当前时间 - 到达时间 <= 盘旋时间 
            arrUp[i] = 1;
            if (now_time - arr[i][0] < 0) // 当前时间 - 到达时间  < 0
                huisu(i, arr[i][0] + arr[i][2], arr, arrUp); // 到达时间 + 降落时间 
            else
                huisu(i, now_time + arr[i][2], arr, arrUp); // 当前时间 + 降落时间 
            if (res == 1)
                break;
            arrUp[i] = 0;
        }
        tag = 0;
    }
    
    if (res == 1 || tag == 1) {
        res = 1;
    }
}


bool fun(vector<vector<int>>& arr) {
//    int now_time = 0;
    int n = arr.size();
    res = false;
    vector<bool> arrUp(n); // 是否已经遍历过 
//    for (int i = 0; i < n; ) {
//        if (now_time >= arr[i][0]) {
//            if (arr[i][1] - (now_time - arr[i][0]) >= 0)
//                now_time += arr[i][2];
//            else
//                return 0;
//            ++i;
//        }
//        else
//            now_time = arr[i][0];
//    } // 贪心不出, 调半天
    huisu(0, 0, arr, arrUp);
    return res;
}

// 背包问题 ?  暴力回溯 ! 
int main() {
    int T;
    scanf("%d", &T);
    for (int i = 0; i < T; ++i) {
        int n;
        scanf("%d", &n);
        vector<vector<int>> arr(n, vector<int>(3));
        for (int j = 0; j < n; ++j) {
            scanf("%d %d %d", &arr[j][0], &arr[j][1], &arr[j][2]);
        }
        sort(arr.begin(), arr.end(), [](const vector<int>& a, const vector<int>& b){
            if (a[0] > b[0]) // 到达时间最早 
                return 0;
//            if (a[0] == b[0] && a[1] > b[1]) // 可以盘旋更久
//                return 0;
//            if (a[0] == b[0] && a[1] == b[1] && a[2] > b[2]) // 滑翔更快 
//                return 0;
            return 1;
        });
        if (fun(arr))
            printf("YES\n");
        else
            printf("NO\n");
    }

    return 0;
}
```

我看题解也几乎是暴力的qwq...

另外有一个艹蛋的是, 我把`YES`理所当然的输出成`Yes`等了!!! $艹^艹_艹$