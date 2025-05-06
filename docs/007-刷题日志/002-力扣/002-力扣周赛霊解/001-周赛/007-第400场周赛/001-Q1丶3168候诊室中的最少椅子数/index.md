# 3168. 候诊室中的最少椅子数
链接: [3168. 候诊室中的最少椅子数](https://leetcode.cn/problems/minimum-number-of-chairs-in-a-waiting-room/)

给你一个字符串 s，模拟每秒钟的事件 i：

- 如果 s[i] == 'E'，表示有一位顾客进入候诊室并占用一把椅子。
- 如果 s[i] == 'L'，表示有一位顾客离开候诊室，从而释放一把椅子。

返回保证每位进入候诊室的顾客都能有椅子坐的 最少 椅子数，假设候诊室最初是 空的 。

# 题解

```C++
class Solution {
public:
    int minimumChairs(string s) {
        int res = 0;
        int tmp = 0;
        for (char c : s) {
            if (c =='E')
                ++res;
            else
                res = res > 0 ? res - 1 : 0; // 这里多此一举
            tmp = max(tmp, res);
        }
        return tmp;
    }
};
```

## 0x3f

```C++
class Solution {
public:
    int minimumChairs(string s) {
        int ans = 0, cnt = 0;
        for (char c : s) {
            if (c == 'E') {
                ans = max(ans, ++cnt);
            } else {
                cnt--;
            }
        }
        return ans;
    }
};
```
