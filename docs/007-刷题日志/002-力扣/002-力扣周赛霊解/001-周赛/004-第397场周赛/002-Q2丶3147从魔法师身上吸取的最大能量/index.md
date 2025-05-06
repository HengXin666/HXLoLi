# 3147. 从魔法师身上吸取的最大能量
链接: [3147. 从魔法师身上吸取的最大能量](https://leetcode.cn/problems/taking-maximum-energy-from-the-mystic-dungeon/)

在神秘的地牢中，n 个魔法师站成一排。每个魔法师都拥有一个属性，这个属性可以给你提供能量。有些魔法师可能会给你负能量，即从你身上吸取能量。

你被施加了一种诅咒，当你从魔法师 i 处吸收能量后，你将被立即传送到魔法师 (i + k) 处。这一过程将重复进行，直到你到达一个不存在 (i + k) 的魔法师为止。

换句话说，你将选择一个起点，然后以 k 为间隔跳跃，直到到达魔法师序列的末端，在过程中吸收所有的能量。

给定一个数组 energy 和一个整数k，返回你能获得的 最大 能量。

- 1 <= energy.length <= 10^5
- -1000 <= energy[i] <= 1000
- 1 <= k <= energy.length - 1

# 题解
## HX
```C++
class Solution {
public:
    int maximumEnergy(vector<int>& arr, int k) {
        vector<int> tmpArr(k);
        int res = -1e9;
        for (int i = arr.size() - 1; i >= 0; i -= k) {
            for (int j = 0; j < k && i - j >= 0; ++j) {
                tmpArr[j] += arr[i - j];
                res = max(res, tmpArr[j]);
            }
        }
        
        return res;
    }
};
```

## 0x3f O(1) 空间

类似于后缀和 的说

```C++
class Solution {
public:
    int maximumEnergy(vector<int>& energy, int k) {
        int n = energy.size(), ans = INT_MIN;
        for (int i = n - k; i < n; i++) {
            int s = 0;
            for (int j = i; j >= 0; j -= k) {
                s += energy[j];
                ans = max(ans, s);
            }
        }
        return ans;
    }
};
```
