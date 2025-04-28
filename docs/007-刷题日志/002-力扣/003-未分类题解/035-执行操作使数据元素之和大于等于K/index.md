# 3091. 执行操作使数据元素之和大于等于 K
链接: [3091. 执行操作使数据元素之和大于等于 K](https://leetcode.cn/problems/apply-operations-to-make-sum-of-array-greater-than-or-equal-to-k/description/)

---

390场周赛 Q2 中等

给你一个正整数 k 。最初，你有一个数组 nums = [1] 。

你可以对数组执行以下 任意 操作 任意 次数（可能为零）：

选择数组中的任何一个元素，然后将它的值 增加 1 。
复制数组中的任何一个元素，然后将它附加到数组的末尾。
返回使得最终数组元素之 和 大于或等于 k 所需的 最少 操作次数。

 
```
示例 1：

输入：k = 11
输出：5
解释：
可以对数组 nums = [1] 执行以下操作：

将元素的值增加 1 三次。结果数组为 nums = [4] 。
复制元素两次。结果数组为 nums = [4,4,4] 。
最终数组的和为 4 + 4 + 4 = 12 ，大于等于 k = 11 。
执行的总操作次数为 3 + 2 = 5 。

示例 2：

输入：k = 1
输出：0
解释：
原始数组的和已经大于等于 1 ，因此不需要执行操作。
```

提示：

$1 <= k <= 10^5$

# 题解
## 1 贪心
1. 首先我们要明白是先`+1`更好,
    - (m + 1) * 2 > m * 2 + 1

所以, 我们直接枚举所有`加1`的情况 $O(N)$

但此时需要的是 额外`加`和`乘`的次数

~~不过我不明白, 为什么要分是否`除得尽`, 两种情况? (OK现在明白了)~~

给定不等式: $$(a + 1) \times (b + 1) \geq k$$

求解 $$\min(a + b) \ (a, b \geq 0)$$

首先，根据不等式，可以得到 $$b \geq \frac{k}{a + 1} - 1$$

接下来，将上式改写为: $$a + b \geq \frac{k}{a + 1} - 1 + a$$

此时 $a, b$ 均是整数, 所以要对 $\frac{k}{a + 1}$ 进行讨论


1. 当 $\frac{k}{a + 1}$ 是整数时，我们可以取 $b = \frac{k}{a + 1} - 1$，此时 $a + b = a + \frac{k}{a + 1} - 1$。因为 $a$ 和 $b$ 都是非负整数，所以我们可以选择使 $a$ 最小的整数值，从而得到 $\min(a + b)$。

2. 当 $\frac{k}{a + 1}$ 不是整数时，因为 $a + b$ 需要大于 $a + \frac{k}{a + 1} - 1$, ...
    - 解释不明白, 直接举例子: $a + b \geq 1.1$
        - 如果`向下取整`, 则 $a + b \geq \left \lfloor 1.1 \right \rfloor = 1$ 那么 $\min(a + b) = 1$ 显然和原本的 $a + b \geq 1.1$ 冲突
        - 如果`四舍五入`, 同上情况
        - 如果`向上取整`, $a + b \geq 2$, => $\min(a + b) = 2$ 显然是合法的
    - 所以需要`上取整`: $$a + b \geq \left \lfloor \frac{k}{a + 1} \right \rfloor  - 1 + a$$


```C++
class Solution {
public:
    int minOperations(int k) {
// 贪心
// 如果 乘的次数 >= 加的次数, 则使用乘法
/*
(a + 1) * (b + 1) >= k 问 a + b

b >= k / (a + 1) - 1

a + b >= k / (a + 1) - 1 + a (上取整) (a, b 是整数) // 怎么取整, 我不会

定一求一:

for (int a = 0; a < k; ++a) {
    res = min(res, k / (a + 1) - 1 + a);
}

*/
        int res = 1e9;
        for (int a = 0; a < k; ++a) {
            if (k % (a + 1)) { // 不能整除, 向上取整
                res = min(res, k / (a + 1) + a);
            } else { // 整除
                res = min(res, k / (a + 1) - 1 + a);
            }
        }
        return res;
    }
};
```

即

```C++
class Solution {
public:
    int minOperations(int k) {
        int res = 1e9;
        for (int a = 0; a < k; ++a) {
            res = min(res, (int)ceil(k / (a + 1.0)) - 1 + a);
        }
        return res;
    }
};
```


## 2 数学

$
(a + 1) * (b + 1) >= k\\ \ \\
即 \ ab + a + b + 1 >= k\\ \ \\
又 \ ab >= \frac{(a + b)^2}{4}\\ \ \\
所以 \ \frac{(a + b)^2}{4} + a + b + 1 >= k\\ \ \\
整理得 \ a + b \geq 2\sqrt{k} - 2\\ \ \\
因为 \ a + b \ 需要是整数, 所以 上取整才可以满足实际的要求
故 \ a + b = \left \lceil 2\sqrt{k} - 2 \right \rceil 
$

```C++
class Solution {
public:
    int minOperations(int k) {
        double r = 2 * pow(k, 0.5) - 2;
        return r - (int)r > 0 ? (int)r + 1 : r;
    }
};
```

## 3 dp