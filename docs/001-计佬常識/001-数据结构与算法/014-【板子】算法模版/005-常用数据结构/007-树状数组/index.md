# 树状数组

要求维护的数据可逆

>  **结合律** 且 **可差分**, 如加法、乘法、异或等

> gcd、max这种, 就不能使用普通树状数组了

什么是树状数组, 请看灵神~

```cpp
template <typename T>
class TreeArr {
    vector<T> tree;
public:
    TreeArr(int n)
        : tree(n + 1)
    {}

    // i 从 1 开始, 到 n
    // 单点修改
    void add(int i, T val) {
        for (; i < tree.size(); i += i & -i)
            tree[i] += val;
    }

    // i 从 1 开始, 到 n
    // 获取前缀和
    T getSum(int i) const {
        T res = 0;
        for (; i; i &= i - 1) // i -= i & -i
            res += tree[i];
        return res;
    }

    // l, r 从 1 开始, 到 n
    // 计算区间和 [l, r] 的
    T sumRange(int l, int r) const {
        if (l > r)
            return T{};
        return getSum(r) - getSum(l - 1);
    }
};
```