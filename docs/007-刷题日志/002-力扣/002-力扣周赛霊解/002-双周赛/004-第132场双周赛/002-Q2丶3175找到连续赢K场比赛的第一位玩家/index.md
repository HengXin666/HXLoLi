3175\. 找到连续赢 K 场比赛的第一位玩家
------------------------

有 `n` 位玩家在进行比赛，玩家编号依次为 `0` 到 `n - 1` 。

给你一个长度为 `n` 的整数数组 `skills` 和一个 **正** 整数 `k` ，其中 `skills[i]` 是第 `i` 位玩家的技能等级。`skills` 中所有整数 **互不相同** 。

所有玩家从编号 `0` 到 `n - 1` 排成一列。

比赛进行方式如下：

*   队列中最前面两名玩家进行一场比赛，技能等级 **更高** 的玩家胜出。
*   比赛后，获胜者保持在队列的开头，而失败者排到队列的末尾。

这个比赛的赢家是 **第一位连续** 赢下 `k` 场比赛的玩家。

请你返回这个比赛的赢家编号。

**示例 1：**

**输入：** skills = \[4,2,6,3,9\], k = 2

**输出：** 2

**解释：**

一开始，队列里的玩家为 `[0,1,2,3,4]` 。比赛过程如下：

*   玩家 0 和 1 进行一场比赛，玩家 0 的技能等级高于玩家 1 ，玩家 0 胜出，队列变为 `[0,2,3,4,1]` 。
*   玩家 0 和 2 进行一场比赛，玩家 2 的技能等级高于玩家 0 ，玩家 2 胜出，队列变为 `[2,3,4,1,0]` 。
*   玩家 2 和 3 进行一场比赛，玩家 2 的技能等级高于玩家 3 ，玩家 2 胜出，队列变为 `[2,4,1,0,3]` 。

玩家 2 连续赢了 `k = 2` 场比赛，所以赢家是玩家 2 。

**示例 2：**

**输入：** skills = \[2,5,4\], k = 3

**输出：** 1

**解释：**

一开始，队列里的玩家为 `[0,1,2]` 。比赛过程如下：

*   玩家 0 和 1 进行一场比赛，玩家 1 的技能等级高于玩家 0 ，玩家 1 胜出，队列变为 `[1,2,0]` 。
*   玩家 1 和 2 进行一场比赛，玩家 1 的技能等级高于玩家 2 ，玩家 1 胜出，队列变为 `[1,0,2]` 。
*   玩家 1 和 0 进行一场比赛，玩家 1 的技能等级高于玩家 0 ，玩家 1 胜出，队列变为 `[1,2,0]` 。

玩家 1 连续赢了 `k = 3` 场比赛，所以赢家是玩家 1 。

**提示：**

*   `n == skills.length`
*   `2 <= n <= 105`
*   `1 <= k <= 109`
*   `1 <= skills[i] <= 106`
*   `skills` 中的整数互不相同。

[https://leetcode.cn/problems/find-the-first-player-to-win-k-games-in-a-row/description/](https://leetcode.cn/problems/find-the-first-player-to-win-k-games-in-a-row/description/)

# 题解
## HX: 模拟

```C++
class Solution {
public:
    int findWinningPlayer(vector<int>& tmp, int k) {
        vector<tuple<int, int>> arr(tmp.size());
        
        for (int i = 0; i < arr.size(); ++i)
                arr[i] = {tmp[i], i};
        
        if (k >= arr.size()) {
            int res = get<0>(arr[0]) , idx = 0;
            for (int i = 0; i < arr.size(); ++i) {
                if (get<0>(arr[i]) > res) {
                    res = get<0>(arr[i]) ;
                    idx = get<1>(arr[i]) ;
                }
            }
            return idx;
        }
        
        list<tuple<int, int>> nums(arr.begin(), arr.end());
        // 模拟
        int cnt = 0;
        while (1) {
            if (get<0>(*nums.begin()) > get<0>(*(++nums.begin()))) {
                ++cnt;
                if (cnt >= k)
                    return get<1>(*nums.begin());
                auto it = ++nums.begin();
                auto jt = *it;
                nums.erase(it);
                nums.push_back(jt);
            } else {
                auto it = *nums.begin();
                nums.pop_front();
                nums.push_back(it);
                cnt = 1;
                if (cnt >= k)
                    return get<1>(*nums.begin());
            }
        }
        return -1;
    }
};
```

## 0x3f: 实际上就打擂台

- 败者永远也不会再次胜利

- 即在从 0 开始寻找的 k 次为 **当前** 最大值 则 OK!

```C++
class Solution {
public:
    int findWinningPlayer(vector<int>& skills, int k) {
        int mx_i = 0, win = 0;
        for (int i = 1; i < skills.size() && win < k; i++) {
            if (skills[i] > skills[mx_i]) { // 新的最大值
                mx_i = i;
                win = 0;
            }
            win++; // 获胜回合 +1
        }
        return mx_i;
    }
};
```
