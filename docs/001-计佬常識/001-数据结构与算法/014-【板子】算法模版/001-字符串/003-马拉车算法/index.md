# 马拉车算法
3325\. 字符至少出现 K 次的子字符串 I
------------------------

给你一个字符串 s 和一个整数 k，在 s 的所有子字符串中，请你统计并返回 **至少有一个** 字符 **至少出现** k 次的子字符串总数。

**子字符串** 是字符串中的一个连续、 **非空** 的字符序列。

**示例 1：**

**输入：** s = "abacb", k = 2

**输出：** 4

**解释：**

符合条件的子字符串如下：

*   "aba"（字符 'a' 出现 2 次）。
*   "abac"（字符 'a' 出现 2 次）。
*   "abacb"（字符 'a' 出现 2 次）。
*   "bacb"（字符 'b' 出现 2 次）。

**示例 2：**

**输入：** s = "abcde", k = 1

**输出：** 15

**解释：**

所有子字符串都有效，因为每个字符至少出现一次。

**提示：**

*   1 <= s.length <= 3000
*   1 <= k <= s.length
*   s 仅由小写英文字母组成。

[https://leetcode.cn/problems/count-substrings-with-k-frequency-characters-i/solutions/2957691/on-hua-dong-chuang-kou-pythonjavacgo-by-1xgqm/](https://leetcode.cn/problems/count-substrings-with-k-frequency-characters-i/solutions/2957691/on-hua-dong-chuang-kou-pythonjavacgo-by-1xgqm/)

见灵神: [【模板】DFS 时间戳 + Manacher 算法（Python/Java/C++/Go）](https://leetcode.cn/problems/check-if-dfs-strings-are-palindromes/solutions/2957704/mo-ban-dfs-shi-jian-chuo-manacher-suan-f-ttu6)

```C++
class Solution {
public:
    vector<bool> findAnswer(vector<int>& parent, string s) {
        int n = parent.size();
        vector<vector<int>> g(n);
        for (int i = 1; i < n; i++) {
            int p = parent[i];
            // 由于 i 是递增的，所以 g[p] 必然是有序的，下面无需排序
            g[p].push_back(i);
        }

        // dfsStr 是后序遍历整棵树得到的字符串
        string dfsStr(n, 0);
        // nodes[i] 表示子树 i 的后序遍历的开始时间戳和结束时间戳+1（左闭右开区间）
        vector<pair<int, int>> nodes(n);
        int time = 0;
        auto dfs = [&](auto&& dfs, int x) -> void {
            nodes[x].first = time;
            for (int y : g[x]) {
                dfs(dfs, y);
            }
            dfsStr[time++] = s[x]; // 后序遍历
            nodes[x].second = time;
        };
        dfs(dfs, 0);

        // Manacher 模板
        // 将 dfsStr 改造为 t，这样就不需要讨论 n 的奇偶性，因为新串 t 的每个回文子串都是奇回文串（都有回文中心）
        // dfsStr 和 t 的下标转换关系：
        // (dfsStr_i+1)*2 = ti
        // ti/2-1 = dfsStr_i
        // ti 为偶数，对应奇回文串（从 2 开始）
        // ti 为奇数，对应偶回文串（从 3 开始）
        string t = "^";
        for (char c : dfsStr) {
            t += '#';
            t += c;
        }
        t += "#$";

        // 定义一个奇回文串的回文半径=(长度+1)/2，即保留回文中心，去掉一侧后的剩余字符串的长度
        // halfLen[i] 表示在 t 上的以 t[i] 为回文中心的最长回文子串的回文半径
        // 即 [i-halfLen[i]+1,i+halfLen[i]-1] 是 t 上的一个回文子串
        vector<int> halfLen(t.length() - 2);
        halfLen[1] = 1;
        // boxR 表示当前右边界下标最大的回文子串的右边界下标+1
        // boxM 为该回文子串的中心位置，二者的关系为 r=mid+halfLen[mid]
        int boxM = 0, boxR = 0;
        for (int i = 2; i < halfLen.size(); i++) { // 循环的起止位置对应着原串的首尾字符
            int hl = 1;
            if (i < boxR) {
                // 记 i 关于 boxM 的对称位置 i'=boxM*2-i
                // 若以 i' 为中心的最长回文子串范围超出了以 boxM 为中心的回文串的范围（即 i+halfLen[i'] >= boxR）
                // 则 halfLen[i] 应先初始化为已知的回文半径 boxR-i，然后再继续暴力匹配
                // 否则 halfLen[i] 与 halfLen[i'] 相等
                hl = min(halfLen[boxM * 2 - i], boxR - i);
            }
            // 暴力扩展
            // 算法的复杂度取决于这部分执行的次数
            // 由于扩展之后 boxR 必然会更新（右移），且扩展的的次数就是 boxR 右移的次数
            // 因此算法的复杂度 = O(len(t)) = O(n)
            while (t[i - hl] == t[i + hl]) {
                hl++;
                boxM = i;
                boxR = i + hl;
            }
            halfLen[i] = hl;
        }

        // t 中回文子串的长度为 hl*2-1
        // 由于其中 # 的数量总是比字母的数量多 1
        // 因此其在 dfsStr 中对应的回文子串的长度为 hl-1
        // 这一结论可用在 isPalindrome 中

        // 判断左闭右开区间 [l,r) 是否为回文串  0<=l<r<=n
        // 根据下标转换关系得到 dfsStr 的 [l,r) 子串在 t 中对应的回文中心下标为 l+r+1
        // 需要满足 halfLen[l + r + 1] - 1 >= r - l，即 halfLen[l + r + 1] > r - l
        auto isPalindrome = [&](int l, int r) -> bool {
            return halfLen[l + r + 1] > r - l;
        };

        vector<bool> ans(n);
        for (int i = 0; i < n; i++) {
            ans[i] = isPalindrome(nodes[i].first, nodes[i].second);
        }
        return ans;
    }
};
```
