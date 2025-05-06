# 3143. 正方形中的最多点数
链接: [3143. 正方形中的最多点数](https://leetcode.cn/problems/maximum-points-inside-the-square/)

给你一个二维数组 $points$ 和一个字符串 $s$ ，其中 $points[i]$ 表示第 $i$ 个点的坐标，$s[i]$ 表示第 $i$ 个点的 标签 。

如果一个正方形的中心在 $(0, 0)$ ，所有边都平行于坐标轴，且正方形内 **不** 存在标签相同的两个点，那么我们称这个正方形是 **合法** 的。

请你返回 **合法** 正方形中可以包含的 **最多** 点数。

注意：

- 如果一个点位于正方形的边上或者在边以内，则认为该点位于正方形内。
- 正方形的边长可以为零。

提示:

- $1 <= s.length, points.length <= 10^5$
- points[i].length == 2
- $-10^9 <= points[i][0], points[i][1] <= 10^9$
- s.length == points.length
- points 中的点坐标互不相同。
- s 只包含小写英文字母。

# 题解
## 我的
- [[HX] O(n)做法](https://leetcode.cn/problems/maximum-points-inside-the-square/solutions/2774411/hx-onzuo-fa-by-heng_xin-e13e)

```C++
class Solution {
public:
    int maxPointsInsideSquare(vector<vector<int>>& p, string s) {
        vector<vector<int>> arr(26, vector<int>(4, 1e9 + 1));
        for (int i = 0; i < p.size(); ++i) {
            p[i][0] = abs(p[i][0]);
            p[i][1] = abs(p[i][1]);

            auto& it = arr[s[i] - 'a'];
            if (max(p[i][0], p[i][1]) <= max(it[0], it[1])) {
                it[2] = it[0];
                it[3] = it[1];
                it[0] = p[i][0];
                it[1] = p[i][1];
            } else if (max(p[i][0], p[i][1]) < max(it[2], it[3])) {
                it[2] = p[i][0];
                it[3] = p[i][1];
            }
        }
        
        int minn = 1e9 + 1;
        for (int i = 0; i < 26; ++i)
            minn = min(minn, max(arr[i][2], arr[i][3]));
        
        int res = 0;
        for (int i = 0; i < 26; ++i)
            if (max(arr[i][0], arr[i][1]) < minn)
                ++res;
        return res;
    }
};
```

## 0x3f
### 二分

```C++
class Solution {
public:
    int maxPointsInsideSquare(vector<vector<int>>& points, string s) {
        int ans = 0;
        auto check = [&](int size) -> bool {
            int vis = 0;
            for (int i = 0; i < points.size(); i++) {
                int x = points[i][0];
                int y = points[i][1];
                char c = s[i] - 'a';
                if (abs(x) <= size && abs(y) <= size) {
                    if (vis >> c & 1) {
                        return false;
                    }
                    vis |= 1 << c;
                }
            }
            ans = __builtin_popcount(vis);
            return true;
        };
        int left = -1, right = 1'000'000'001;
        while (left + 1 < right) {
            int mid = (left + right) / 2;
            (check(mid) ? left : right) = mid;
        }
        return ans;
    }
}; // By 0x3f
```

### 维护次小距离的最小值

实际上就和我的思路差不多, 只不过灵神转为切比雪夫距离了qwq...

```C++
class Solution {
public:
    int maxPointsInsideSquare(vector<vector<int>>& points, string s) {
        int min_d[26], min2 = INT_MAX;
        ranges::fill(min_d, INT_MAX);
        for (int i = 0; i < points.size(); i++) {
            int x = points[i][0], y = points[i][1], c = s[i] - 'a';
            int d = max(abs(x), abs(y));
            if (d < min_d[c]) {
                // d 是目前最小的，那么 min_d[c] 是次小的
                min2 = min(min2, min_d[c]);
                min_d[c] = d;
            } else {
                // d 可能是次小的
                min2 = min(min2, d);
            }
        }
        int ans = 0;
        for (int d : min_d) {
            ans += d < min2;
        }
        return ans;
    }
}; // By 0x3f
```
