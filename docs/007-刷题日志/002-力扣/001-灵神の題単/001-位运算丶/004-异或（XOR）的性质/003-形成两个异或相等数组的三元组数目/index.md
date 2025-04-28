# 1442. 形成两个异或相等数组的三元组数目

链接: [1442. 形成两个异或相等数组的三元组数目](https://leetcode.cn/problems/count-triplets-that-can-form-two-arrays-of-equal-xor/)

第 188 场周赛 Q2 `1525`

给你一个整数数组 `arr`。

现需要从数组中取三个下标 i、j 和 k ，其中 (`0 <= i < j <= k < arr.length` 。

a 和 b 定义如下：

- `a = arr[i] ^ arr[i + 1] ^ ... ^ arr[j - 1]`
- `b = arr[j] ^ arr[j + 1] ^ ... ^ arr[k]`

注意: `^`表示 **按位异或** 操作。

请返回能够令`a == b`成立的三元组`(i, j , k)`的数目。

# 题解
## 前缀和 $O(n^3)$

```C++
class Solution {
public:
    int countTriplets(vector<int>& arr) {
        // i j k
        const int n = arr.size();
        int res = 0;
        vector<int> SumArr(n + 1); // 前缀和
        for (int i = 0; i < n; ++i) {
            SumArr[i + 1] = SumArr[i] ^ arr[i];
        }

        function<int(int, int)> find = [&](int L, int R) -> int {
            return SumArr[R + 1] ^ SumArr[L];
        };

        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                for (int k = j; k < n; ++k) {
                    if (find(i, j - 1) == find(j, k))
                        ++res;
                }
            }
        }

        return res;
    }
};
```

## $O(n^2)$ 优化
我们求 $a == b$ 即有`a ^ b = a ^ a = 0`, 即`arr[i] ^ arr[i + 1] ^ ... ^ arr[j] ^ ... ^ arr[k] == 0` (说明 $j$ 可以取 $[i + 1, k]$ 的所有整数)

因此我们可以优化到只需要 $o(n^2)$ 的复杂度(枚举 $i$ 与 $k$ 即可,)

```C++
class Solution {
public:
    int countTriplets(vector<int>& arr) {
        const int n = arr.size();
        int res = 0;
        vector<int> SumArr(n + 1);
        for (int i = 0; i < n; ++i) {
            SumArr[i + 1] = SumArr[i] ^ arr[i];
        }

        function<int(int, int)> find = [&](int L, int R) -> int {
            return SumArr[R + 1] ^ SumArr[L];
        };

        for (int i = 0; i < n; ++i) {
            for (int k = i + 1; k < n; ++k) {
                if (!find(i, k))
                    res += k - i;
            }
        }

        return res;
    }
};
```

## $O(n)$ 优化! 前缀和 + 哈希表
记 $S_{[i, k]}$ 是 异或前缀和 $[i, k]$ 区间的结果, 那么对于 上面的优化: 对于所有满足 $S_{[i_1,k]}, S_{[i_2, k]}, ..., S_{[i_3, k]}$ 二元组 $(i_1, k), (i_2, k), ..., (i_m, k)$ 对答案的贡献为:

$$(k - i_1) + (k - i_2) + ... + (k - i_m) = k \times m - (i_1 + i_2 + ... + i_m)= k \times m - \sum_{j=1}^{m}i_j$$

所以我们只需要知道 当前索引 $k$, 以及 当前符合的长度 $m$ 即可

```C++
class Solution {
public:
    int countTriplets(vector<int>& arr) {
        const int n = arr.size();
        int res = 0;
        // 异或前缀和值, 数量, 索引和
        unordered_map<int, pair<int, int>> hash;

        for (int i = 0, sum = 0; i < n; ++i) {
            if (int tmp = sum ^ arr[i]; hash.count(tmp)) {
                auto [sl, s] = hash[tmp];
                res += i * sl - s;
            }

            auto& [sl, s] = hash[sum];
            ++sl;
            s += i;

            sum ^= arr[i];
        }

        return res;
    }
};
```
