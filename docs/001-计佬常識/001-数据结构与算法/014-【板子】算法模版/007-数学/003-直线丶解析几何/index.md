# 直线
## 一、计算直线解析式
### 1.1 斜截式 y = kx + b

```C++
bool get_line_slope_intercept(int x1, int y1, 
                              int x2, int y2, 
                              double &k, double &b) {
    if (x1 == x2)
        return false; // 垂直线, 无法表示为斜截式
    k = (y2 - y1) * 1.0 / (x2 - x1);
    b = y1 - k * x1;
    return true;
}
```

因为需要讨论斜率不存在, 以及精度损失. 因此我们一般 **不使用** 这个刁毛.

### 1.2 一般式 Ax + By + C = 0

```C++
// 返回直线 Ax + By + C = 0 中的 A, B, C
void get_line_general_form(int x1, int y1, 
                           int x2, int y2, 
                           int &A, int &B, int &C) {
    A = y2 - y1;
    B = x1 - x2;
    C = x2 * y1 - x1 * y2;
}
```

#### 1.2.1 推导: 向量叉乘法

核心思想:

> 一条直线上任意一点，与给定的两个点构成的向量，它们是**共线**的，那共线的两个向量叉积就为 0。

我们用这条直线上任意一点 $(x, y)$ 来构造向量，考虑：

- 向量1: $\vec{v_1} = (x_2 - x_1, y_2 - y_1)$, 即 AB 向量
- 向量2: $\vec{v_2} = (x - x_1, y - y_1)$, 即 AM 向量（任意点）

如果点 $(x, y)$ 在直线上, 那么

$$
\vec{v_1} \times \vec{v_2} = 0
$$

叉乘公式(二维):

$$
(x_2 - x_1)(y - y_1) - (y_2 - y_1)(x - x_1) = 0
$$

展开整理:

$$
(x_2 - x_1)y - (x_2 - x_1)y_1 - (y_2 - y_1)x + (y_2 - y_1)x_1 = 0
$$

我们把它整理成 $Ax + By + C = 0$ 形式:

- $A = y_2 - y_1$
- $B = x_1 - x_2$
- $C = x_2 y_1 - x_1 y_2$

就推出来了!

记忆方法:

```cpp
A = y2 - y1;
B = x1 - x2;
C = x2 * y1 - x1 * y2;
// => A = Δy，B = -Δx，C = -叉积

// 叉积
// = (x1, y1) x (x2, y2)
// = x1 * y2 - x2 * y1
```

### 1.2.2 板子

```C++
tuple<int, int, int> getLine(
    vector<int> const& a, 
    vector<int> const& b
) {
    int x1 = a[0], y1 = a[1];
    int x2 = b[0], y2 = b[1];
    int A = y2 - y1;
    int B = x1 - x2;
    int C = x2 * y1 - x1 * y2;

    int g = gcd(gcd(abs(A), abs(B)), abs(C));
    if (g) {
        A /= g, B /= g, C /= g;
    }

    // 保证符号一致性
    if (A < 0 || (A == 0 && B < 0)) {
        A = -A; B = -B; C = -C;
    }

    return {A, B, C};
}
```

> [!TIP]
> 因为计算的`tuple<int, int, int>`是不唯一的, 也就是不是最简的, 因此我们需要使用 $\gcd$ 归一化!