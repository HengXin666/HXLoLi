# 质数
## 零、何为质数

如果一个数 $x$ **只能** 被 $1$ **或者** $x$ 整除, 那么 $x$ 就是质数 (素数).

特别的, $1$ **不是** 质数.

---

冷知识: 如果当前数字长度为 8, 那么可以跳过检查, 因为不存在 8 长度的素数.

- **偶数** 长度的 **回文数** 中, 只有`11`是素数.

## 一、判断质数

```C++
auto isFk = [](int x) {
    for (int i = 2; i * i <= x; ++i)
        if (!(x % i))
            return false;
    return x >= 2;
};
```

- 时间复杂度: $O(\sqrt{n})$

## 二、质数筛
### 2.1 埃氏筛

```C++
constexpr int N = 5e6 + 1;

vector<int> fkNum;

auto __init__ = [] {
    vector<char> isUnFk(N);
    for (int i = 2; i < N; ++i) {
        if (isUnFk[i])
            continue;
        fkNum.push_back(i);
        for (int j = i + i; j < N; j += i)
            isUnFk[j] = true;
    }
    return 0;
}();
```

- 时间复杂度: $O(n \log{\log{n}})$ 证明: 略

### 2.2 线性筛

- 待更新
