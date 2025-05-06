# 3200. 三角形的最大高度
链接: [3200. 三角形的最大高度](https://leetcode.cn/problems/maximum-height-of-a-triangle/)

给你两个整数 red 和 blue，分别表示红色球和蓝色球的数量。你需要使用这些球来组成一个三角形，满足第 1 行有 1 个球，第 2 行有 2 个球，第 3 行有 3 个球，依此类推。

每一行的球必须是 相同 颜色，且相邻行的颜色必须 不同。

返回可以实现的三角形的 最大 高度。

# 题解
## 暴力模拟

```C++
class Solution {
    int fk(int a, int b) {
        int res = 0;
        while (1) {
            a -= ++res;
            if (a < 0)
                break;
            b -= ++res;
            if (b < 0)
                break;
        }
        return res - 1;
    }
public:
    int maxHeightOfTriangle(int red, int blue) {
        return max(fk(red, blue), fk(blue, red));
    }
};
```

灵神code:
```C++
class Solution {
public:
    int maxHeightOfTriangle(int red, int blue) {
        int cnt[2]{};
        for (int i = 1; ; i++) {
            cnt[i % 2] += i;
            if ((cnt[0] > red || cnt[1] > blue) && (cnt[0] > blue || cnt[1] > red)) {
                return i - 1;
            }
        }
    }
};
```

## 数学公式

- [两种方法：枚举 / O(1) 数学公式（Python/Java/C++/Go）](https://leetcode.cn/problems/maximum-height-of-a-triangle/solutions/2826643/o1-shu-xue-gong-shi-pythonjavacgo-by-end-t2ht)

```C++
class Solution {
    int f(int n, int m) {
        int odd = sqrt(n);
        int even = (sqrt(m * 4 + 1) - 1) / 2;
        return odd > even ? even * 2 + 1 : odd * 2;
    }

public:
    int maxHeightOfTriangle(int red, int blue) {
        return max(f(red, blue), f(blue, red));
    }
};
```
