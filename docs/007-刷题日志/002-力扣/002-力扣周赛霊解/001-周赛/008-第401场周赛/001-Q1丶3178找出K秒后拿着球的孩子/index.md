# 3178. 找出 K 秒后拿着球的孩子
链接: [3178. 找出 K 秒后拿着球的孩子](https://leetcode.cn/problems/find-the-child-who-has-the-ball-after-k-seconds/)

给你两个 正整数 n 和 k。有 n 个编号从 0 到 n - 1 的孩子按顺序从左到右站成一队。

最初，编号为 0 的孩子拿着一个球，并且向右传球。每过一秒，拿着球的孩子就会将球传给他旁边的孩子。一旦球到达队列的 任一端 ，即编号为 0 的孩子或编号为 n - 1 的孩子处，传球方向就会 反转 。

返回 k 秒后接到球的孩子的编号。

# 题解
## O(1) 公式

- `k / (n - 1) % 2`来回的次数, 奇偶分方向

- `n - t - 1 : t` 正着还是倒着数

```C++
class Solution {
public:
    int numberOfChild(int n, int k) {
        int t = k % (n - 1);
        return k / (n - 1) % 2 ? n - t - 1 : t;
    }
};
```
