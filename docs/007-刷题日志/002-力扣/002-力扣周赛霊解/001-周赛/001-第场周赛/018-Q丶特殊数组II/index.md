# 3152. 特殊数组 II
链接: [3152. 特殊数组 II](https://leetcode.cn/problems/special-array-ii/)

如果数组的每一对相邻元素都是两个奇偶性不同的数字，则该数组被认为是一个 特殊数组 。

周洋哥有一个整数数组 nums 和一个二维整数矩阵 nums，对于 queries[i] = [fromi, toi]，请你帮助周洋哥检查子数组 true 是不是一个 false特殊数组 。

返回布尔数组 answer，如果 nums[fromi..toi] 是特殊数组，则 answer[i] 为 true ，否则，answer[i] 为 false 。

提示:
- 1 <= nums.length <= 10^5
- 1 <= nums[i] <= 10^5
- 1 <= queries.length <= 10^5
- queries[i].length == 2
- 0 <= queries[i][0] <= queries[i][1] <= nums.length - 1

# 题解
## HX: 并查集

并查集, 我们知道如果一个数组的子数组不是`特殊数组`, 那么这个数组也不是一个`特殊数组`. 因此我们可以找到一个分界线(或者说划分成两块区域, 左边是`特殊数组`, 右边也是`特殊数组`, 但是合并后就 **不是** `特殊数组`. 故有

```C++
class Solution {
public:
    vector<bool> isArraySpecial(
        vector<int>& nums, 
        vector<vector<int>>& qu) {
        const int n = qu.size();
        vector<bool> res(n);
        // 并查集
        vector<int> pa(nums.size());
        for (int i = 0; i < pa.size(); ++i) // init
            pa[i] = i;
        
        for (int i = 1; i < nums.size(); ++i) {
            if ((nums[i] & 1) ^ (nums[i - 1] & 1)) { // 是
                pa[i - 1] = i;
            }
        }
        
        function<int(int)> find = [&](int x) {
            if (x != pa[x])
                pa[x] = find(pa[x]);
            return pa[x];
        };
        
        for (int i = 0; i < n; ++i) {
            if (find(qu[i][0]) == find(qu[i][1]))
                res[i] = true;
        }
        return res;
    }
};
```

时间复杂度 $O(nlogn)$

## 0x3f: 前缀和 + 位运算

定义一个长为 $n−1$ 的数组 $a$，其中

$$a[i] = \begin{cases} 0,\ &\textit{nums}[i]\bmod 2 \ne \textit{nums}[i+1]\bmod 2\\ 1,\ &\textit{nums}[i]\bmod 2 = \textit{nums}[i+1]\bmod 2 \end{cases}$$

计算 $a$ 的前缀和 $s$，可以快速判断子数组和是否为 $0$。

```C++
class Solution {
public:
    vector<bool> isArraySpecial(vector<int>& nums, vector<vector<int>>& queries) {
        vector<int> s(nums.size());
        for (int i = 1; i < nums.size(); i++) {
            s[i] = s[i - 1] + (nums[i - 1] % 2 == nums[i] % 2);
        }
        vector<bool> ans(queries.size());
        for (int i = 0; i < queries.size(); i++) {
            auto& q = queries[i];
            ans[i] = s[q[0]] == s[q[1]];
        }
        return ans;
    }
};
```

另一种写法，相邻两数的异或和的最低位取反，即为 $a[i]$。

```C++
class Solution {
public:
    vector<bool> isArraySpecial(vector<int>& nums, vector<vector<int>>& queries) {
        vector<int> s(nums.size());
        for (int i = 1; i < nums.size(); i++) {
            s[i] = s[i - 1] + ((nums[i] ^ nums[i - 1] ^ 1) & 1);
        }
        vector<bool> ans(queries.size());
        for (int i = 0; i < queries.size(); i++) {
            auto& q = queries[i];
            ans[i] = s[q[0]] == s[q[1]];
        }
        return ans;
    }
};
```

时间复杂度 $O(n + q)$ $q$ 是`queries.size()`