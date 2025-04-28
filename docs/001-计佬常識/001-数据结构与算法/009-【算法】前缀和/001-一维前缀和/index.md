# 一维前缀和

如果我们需要 $O(1)$ 的得到数组 $[a, b]$ 下标的和 / 积, 朴素的想法是 直接 $\sum_{i=a}^{b}arr[i]$ 或者 $\prod_{i=a}^{b}arr[i]$ 但是这样的时间复杂度是 $O(n)$ 的. 对于某些题目, 它可能只是题目最终结果的一个部分, 比如: [[中位数贪心]货仓寻址问题](../../012-【算法】贪心/001-【中位数贪心】货仓寻址问题/index.md) 等等.

因此我们需要一个 $O(1)$ 的算法, 这样可以使得原来的时间复杂度除上一个 $O(n)$ 是个定胜负的优化! (当然你也可以使用大炮打蚊子, 比如使用[普通线段树](../../010-【数据结构】线段树/001-普通线段树/index.md)来搞(**如果题目要求这个求和区间是动态变化的话纯前缀和就不行了**))

我们可以看看代码:

```C++
using ll = long long;

int n;
cin >> n;
vector<int> arr(n);
for (int i = 0; i < n; ++i)
    cin >> arr[i];

vector<ll> sumArr(n + 1); // 前缀和数组, 注意需要开 n + 1 个位置, 最好使用 long long, 不然见祖宗

for (int i = 0; i < n; ++i)
    sumArr[i + 1] = arr[i] + sumArr[i]; // 求前缀和

// 获取 arr[a] ~ arr[b] 的和
ll getSum(int index, int jndex) {
    return sumArr[jndex + 1] - sumArr[index];
}
```
