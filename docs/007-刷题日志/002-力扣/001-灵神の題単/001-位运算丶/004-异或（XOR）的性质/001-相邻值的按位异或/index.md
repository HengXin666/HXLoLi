# 2683. 相邻值的按位异或
链接: [2683. 相邻值的按位异或](https://leetcode.cn/problems/neighboring-bitwise-xor/)

第 345 场周赛 Q2 `1518`

下标从 0 开始、长度为 `n` 的数组 `derived` 是由同样长度为 n 的原始 二进制数组 original 通过计算相邻值的 按位异或（⊕）派生而来。

特别地，对于范围 `[0, n - 1]` 内的每个下标 `i` ：

- 如果 `i = n - 1` ，那么 `derived[i] = original[i] ⊕ original[0]`
- 否则 `derived[i] = original[i] ⊕ original[i + 1]`

给你一个数组 `derived` ，请判断是否存在一个能够派生得到 `derived` 的 有效原始二进制数组 `original` 。

如果存在满足要求的原始二进制数组，返回 `true` ；否则，返回 `false` 。

二进制数组是仅由 0 和 1 组成的数组。

# 题解
## 模拟

```C++
class Solution {
public:
    bool doesValidArrayExist(vector<int>& derived) {
        // d[i] == 1 则 o[i] 必需和 o[i + 1] 不一样
        // d[i] == 0 则 o[i] 必需和 o[i + 1] 一样
        // 实际上只需要判断最后那个是否符合o[0]

        vector<int> original(derived.size() + 1);
        if (derived[0]) {
            original[0] = 1;
            original[1] = 0;
        } else {
            original[0] = original[1] = 1;
        }

        for (int i = 1; i < derived.size(); ++i) {
            if (derived[i]) { // 1
                original[i + 1] = !original[i];
            } else { // 0
                original[i + 1] = original[i];
            }
        }

        return original[derived.size()] == original[0];
    }
};
```

## 0x3f 推公式

由 $a ⊕ a = 0$ 性质得:

$a ⊕ b = c$ 同时异或上 $a$ 有: $b = a ⊕ c$

则 $$derived[i] = original[i] ⊕ original[i + 1]$$ 同时异或上 $original[i]$ 有 $$original[i + 1] = derived[i] ⊕ original[i]$$

而 $$original[n - 1] = derived[n - 2] ⊕ original[n - 2] (1式)$$ 而 $$original[n - 2] = derived[n - 3] ⊕ original[n - 3] \\ ... \\ original[1] = derived[0] ⊕ original[0]$$ 统统代入 (1式) 有 

$original[n - 1] \\ = derived[n - 2] ⊕ original[n - 2] \\ = derived[n - 2] ⊕ (derived[n - 3] ⊕ original[n - 3]) \\ = derived[n - 2] ⊕ (derived[n - 3] ⊕ (... ⊕ derived[0] ⊕ original[0]))$ (2式)

而题目规定 $$derived[n - 1] = original[n - 1] ⊕ original[0]$$ 同时异或上 $original[n - 1]$ 有 $$original[0] = derived[n - 1] ⊕ original[n - 1]$$ 联立 (2式) 有 $$original[n - 1] = derived[n - 2] ⊕ derived[n - 3] ⊕ ... ⊕ derived[0] ⊕ derived[n - 1] ⊕ original[n - 1]$$ 然后再同时异或上 $original[n - 1]$ 整理得 $$derived[n - 1] ⊕ derived[n - 2] ⊕ derived[n - 3] ⊕ ... ⊕ derived[0] = 0$$ 即


```C++
class Solution {
public:
    bool doesValidArrayExist(vector<int>& derived) {
        int tmp = 0;
        for (int& it : derived)
            tmp ^= it;
        return !tmp;
    }
};
```
