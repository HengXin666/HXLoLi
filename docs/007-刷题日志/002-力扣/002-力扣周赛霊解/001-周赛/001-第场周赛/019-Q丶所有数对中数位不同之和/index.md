# 3153. 所有数对中数位不同之和
链接: [3153. 所有数对中数位不同之和](https://leetcode.cn/problems/sum-of-digit-differences-of-all-pairs/)

车尔尼有一个数组 nums ，它只包含 正 整数，所有正整数的数位长度都 相同 。

两个整数的 数位不同 指的是两个整数 相同 位置上不同数字的数目。

请车尔尼返回 nums 中 所有 整数对里，数位不同之和。

提示:
- 2 <= nums.length <= 10^5
- 1 <= nums[i] < 10^9
- nums 中的整数都有相同的数位长度。

# 题解
## HX: 拆位 + 哈希

```C++
class Solution {
public:
    long long sumDigitDifferences(vector<int>& nums) {
        int len = to_string(nums[0]).size();
        vector<unordered_map<int, int>> arr(len);
        for (int i = 1, c = 0; c < len; ++c, i *= 10) {
            for (int it : nums) {
                ++arr[c][(it / i) % 10];
            }
        }
        
        long long res = 0;
        for (int c = 0; c < len; ++c) {
            for (int i = 0; i < 10; ++i) {
                if (arr[c][i])
                for (int j = i + 1; j < 10; ++j) {
                    if (arr[c][j])
                    res += arr[c][i] * arr[c][j];
                }
            }
        }
        return res;
    }
};
```

我是统一计算, 灵神是边加边计算orz..

## 0x3f: 拆位算贡献+一次遍历
横看成岭侧成峰，换一个角度，把每一位拆开，先计算个位数中的不同数对个数，再计算十位数中的不同数对个数，然后是百位数中的不同数对个数，依此类推。


```C++
class Solution {
public:
    long long sumDigitDifferences(vector<int>& nums) {
        long long ans = 0;
        vector<array<int, 10>> cnt(to_string(nums[0]).length());
        for (int k = 0; k < nums.size(); k++) {
            int x = nums[k];
            for (int i = 0; x; x /= 10, i++) {
                int d = x % 10;
                ans += k - cnt[i][d]++;
            }
        }
        return ans;
    }
};
```

逆向思维

```C++
class Solution {
public:
    long long sumDigitDifferences(vector<int>& nums) {
        long long n = nums.size(), m = to_string(nums[0]).length();
        long long ans = m * n * (n - 1) / 2;
        vector<array<int, 10>> cnt(m);
        for (int x : nums) {
            for (int i = 0; x; x /= 10) {
                ans -= cnt[i++][x % 10]++;
            }
        }
        return ans;
    }
};
```
