# 894. 所有可能的真二叉树
原题: [894. 所有可能的真二叉树](https://leetcode.cn/problems/all-possible-full-binary-trees/description/)

第 99 场周赛 Q3 中等 `1784` *做不出这道题也不要沮丧。lc难度分1700做到2000，这是最艰难的几道题之一，对递归要求有比较深刻的理解*

给你一个整数`n`，请你找出所有可能含`n`个节点的 真二叉树 ，并以列表形式返回。答案中每棵树的每个节点都必须符合`Node.val == 0`。

答案的每个元素都是一棵真二叉树的根节点。你可以按 **任意顺序** 返回最终的真二叉树列表。

真二叉树 是一类二叉树，树中每个节点恰好有`0`或`2`个子节点。

 

## 示例 1：

输入：n = 7
输出：[[0,0,0,null,null,0,0,null,null,0,0],[0,0,0,null,null,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,null,null,null,null,0,0],[0,0,0,0,0,null,null,0,0]]

## 示例 2：

输入：n = 3
输出：[[0,0,0]]

## 提示：

1 <= n <= 20

# 题解
## 递归

可以使用 map 来记忆化
```C++
/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode() : val(0), left(nullptr), right(nullptr) {}
 *     TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
 *     TreeNode(int x, TreeNode *left, TreeNode *right) : val(x), left(left), right(right) {}
 * };
 */
class Solution {
public:
    vector<TreeNode*> allPossibleFBT(int n) {
        vector<TreeNode*> res;
        if (!(n & 1))
            return res; // 显然的
        
        if (n == 1) {
            res.push_back(new TreeNode); // 特殊处理的
            return res;
        }

        for (int l = 1; l < n; ++l) {
            // root 为 其所有可能的左子树 和右子树 的组合
            vector<TreeNode*> L = allPossibleFBT(l);
            vector<TreeNode*> R = allPossibleFBT(n - l - 1);
            for (TreeNode* & i : L) {
                for (TreeNode* & j : R) {
                    res.push_back(new TreeNode(0, i, j));
                }
            }
        }
        
        return res;
    }
};
```

## dp
> [动态规划（Python/Java/C++/Go/JS/Rust）](https://leetcode.cn/problems/all-possible-full-binary-trees/solutions/2719981/dong-tai-gui-hua-pythonjavacgojsrust-by-u3waz/?envType=daily-question&envId=2024-04-02)

看不懂思密达, 好像又明白了, 就是和上面的差不多(记忆化转dp的)

```C++
vector<TreeNode*> f[11];

auto init = [] {
    f[1] = {new TreeNode()};
    for (int i = 2; i < 11; i++) { // 计算 f[i]
        for (int j = 1; j < i; j++) { // 枚举左子树叶子数
            for (auto left : f[j]) { // 枚举左子树
                for (auto right : f[i - j]) { // 枚举右子树
                    f[i].push_back(new TreeNode(0, left, right));
                }
            }
        }
    }
    return 0;
}();

class Solution {
public:
    vector<TreeNode*> allPossibleFBT(int n) {
        return f[n % 2 ? (n + 1) / 2 : 0];
    }
};

// 作者：灵茶山艾府
// 链接：https://leetcode.cn/problems/all-possible-full-binary-trees/solutions/2719981/dong-tai-gui-hua-pythonjavacgojsrust-by-u3waz/
// 来源：力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```