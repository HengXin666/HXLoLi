# 曼哈顿距离转切比雪夫距离
## 例题
> [100240. 最小化曼哈顿距离](https://leetcode.cn/problems/minimize-manhattan-distances/description/)

## 证明
> 1. [【图解】曼哈顿距离转切比雪夫距离（Python/Java/C++/Go）](https://leetcode.cn/problems/minimize-manhattan-distances/solutions/2716755/tu-jie-man-ha-dun-ju-chi-heng-deng-shi-b-op84/) (几何的证明法)
>
> 2. [更加数学的](https://leetcode.cn/problems/minimize-manhattan-distances/solutions/2716752/xiang-xi-tui-dao-jing-dian-jie-lun-fu-li-3rkq/)

总而言之, 对于点 $(x_1, y_1), (x_2, y_2)$ **曼哈顿距离** 为: $$\left | x_1 - x_2 \right | + \left | y_1 - y_2 \right |$$

其有如下关系: $$\left | x_1 - x_2 \right | + \left | y_1 - y_2 \right | = max(\left | {x_1}' - {x_2}' \right |, \left | {y_1}' - {y_2}' \right |)$$

其中 $$({x}', {y}') = (y + x, y - x)$$

### 手动证明:

$\left | x_1 - x_2 \right | + \left | y_1 - y_2 \right | \\ = max(x_1 - x_2, x_2 - x_1) + max(y_1-y_2, y_2-y_1)\\ \ \\=max(\\(x_1-x_2) + (y_1-y_2), \\(x_1-x_2)+(y_2-y_1),\\(x_2-x_1)+(y_1-y_2), \\(x_2-x_1)+(y_2-y_1)) \text{[分配律]}\\ \ \\=max(\\(x_1+y_1)-(x_2+y_2),\\(y_2-x_2)-(y_1-x_1),\\(y_1-x_1)-(y_2-x_2),\\(x_2+y_2)-(x_1+y_1))[将上面的x_1和y_1放一起, 整理有]\\ \ \\=max(|(x_1+y_1)-(x_2+y_2)|, |(y_1-x_1) - (y_2-x_2)|)\\ \ \\现在令 \ \ \ x'为y+x, \ \ \ y'为y-x\\ \ \\$ 亦有 $$\left | x_1 - x_2 \right | + \left | y_1 - y_2 \right | = max(\left | {x_1}' - {x_2}' \right |, \left | {y_1}' - {y_2}' \right |)$$ 证毕

## 解例题
因为需要的是去掉一个点后的最值, 则可以 **尝试** 去掉该点, 即可

> 为什么需要两个`multiset<int>`, 而不是一个已经计算好的 multiset<int> = max(x', y') ?
>
> 那么你需要理解一下 **曼哈顿距离转切比雪夫距离** 的几何意义:
>
> > 即 曼哈顿距离 是 在 比雪夫距离 坐标系下 x 或者 y 坐标距离原点的距离
> >
> > 即 点 映射到x/y坐标轴上距离原点更远的那个才是 曼哈顿距离
> >
> > 两个点的曼哈顿距离是 $\left | x_1 - x_2 \right | + \left | y_1 - y_2 \right | = max(\left | {x_1}' - {x_2}' \right |, \left | {y_1}' - {y_2}' \right |)$ 但是它不等价于 $max(|max(x_1', y_1') - max(x_2', y_2')|)$ 因此需要两个`set`

```C++
class Solution {
public:
    int minimumDistance(vector<vector<int>>& points) {
        multiset<int> x;
        multiset<int> y;
        for (auto& it : points) {
            x.insert(it[1] + it[0]);
            y.insert(it[1] - it[0]);
        }

        int res = 1e9;
        for (auto& it : points) {
            // 尝试删除每一个点
            int xd = it[0] + it[1];
            int yd = it[1] - it[0];
            x.erase(x.find(xd));
            y.erase(y.find(yd));
            res = min(res, max(*x.rbegin() - *x.begin(), *y.rbegin() - *y.begin()));
            x.insert(xd);
            y.insert(yd);
        }

        return res;
    }
};
```
