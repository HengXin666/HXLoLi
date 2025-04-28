# 字典树
> 学习: [字典树的巧妙运用【力扣周赛 385】](https://www.bilibili.com/video/BV1jZ42127Yf/)

## 题目

给你一个下标从 $0$ 开始的字符串数组 $words$ 。

定义一个 **布尔** 函数 `isPrefixAndSuffix`，它接受两个字符串参数`str1`和`str2`:

当 `str1` 同时是 `str2` 的前缀 (<a title="字符串的前缀是从字符串的开头开始并延伸到其中任意点的子串。">prefix</a>) 和后缀 (<a title="字符串的后缀是从字符串的任意点开始并延伸到其末尾的子串">suffix</a>) 时，`isPrefixAndSuffix(str1, str2)`返回`true`，否则返回`false`。

例如: `isPrefixAndSuffix("aba", "ababa")`返回 `true`，因为`"aba"`既是`"ababa"`的前缀，也是`"ababa"`的后缀，但是`isPrefixAndSuffix("abc", "abcd")`返回`false`。

以整数形式，返回满足 $i < j$ 且`isPrefixAndSuffix(words[i], words[j])`为`true`的下标对 $(i, j)$ 的 **数量**。 --By: [3045. 统计前后缀下标对 II](https://leetcode.cn/problems/count-prefix-and-suffix-pairs-ii/description/)

**提示**: 

$
1 <= words.length <= 10^5 \\
1 <= words[i].length <= 10^5 \\
words[i] 仅由小写英文字母组成 \\
所有 words[i] 的长度之和不超过 5 * 10^5
$

显然使用 [3042. 统计前后缀下标对 I](https://leetcode.cn/problems/count-prefix-and-suffix-pairs-i/) 的暴力做法(枚举+哈希)是行不通的

我们需要一种至少时间复杂度为 $O(N)$ ( $N$ 为所有字符串长度之和) 的算法.

## 引入
我们暂时将上面问题 **简化** 为: 求`words[j]`的`前缀`是否是`words[i]`( $i < j$ )

显然我们可以对 前缀按字符进行分组: 对于 `["a","aba","ababa","aa"]`

可以分为
```
a
aa
aba
ababa
...
```

显然不太友好, 并且有很多重叠部分, 不妨想办法压缩一下:
```
    a
   / \
  a   b
     /
    a
   /
  b
 /
a
```
显然这是一颗树, 而且不是二叉树, 而是一般树(至少有26个字母(分支))

然后加上 一个 $num$ 字段, 表示数量 (比如`["a", "aba"]`, 那么 对于`"aba"`应该在`aba`的最后一个结点(a)处才标上数字(具体怎么来实际上看题目安排的));

为了只使用一颗树, 不妨把根结点记为`""`(空字符串(一个也没有匹配))

综上即有 `["a","aba","ababa","aa"]` 的字典树为:

```
      ""
     /
    a (1)
   / \
  a   b
 (1) /
    a (1)
   /
  b
 /
a (1)
```

## 定义
那么现在, 你就可以对字典树下一个定义:

> 1. 把每个字母对应到一棵树的某个节点
>
> 2. <b style="color: yellow; text-shadow: 6px 4px 2.4px rgb(200, 60, 125);">保证对于每个字符串 s, s[i] 一定是 s[i + 1] 的父节点</b>

## 解决问题
现在, 回到原来的问题, 我们是不是可以把求 前缀和后缀 和在一起?

- 对于前缀我们是`s[i] for i in (i = 0, i < s.size(); ++i)`

- 对于后缀我们也是`s[i] for i in (i = s.size() - 1, i >= 0, --i)`

我们能不能将他们合起来呢?

- 答: 可: `s[i] 与 s[s.size() - 1 - i] for i in (i = 0, i < s.size(); ++i)`

- 对于 "aba" 与 "ababa" 是合适的:

  ```C++
  ababa
  aba (前缀)
  
  ababa
    aba (后缀)
  ```

那么现在求得 前缀和后缀 不就是和求 前缀没有区别吗, 只是把 字典数 的节点从单个前缀字符, 变成了 (前缀, 后缀) 的 pair

我们看看代码怎么写: (最需要注意的是, Node的子节点我使用的是哈希映射, 但是子节点记得要使用`Node *`类型, 不然你的内存会因为循环创建而自杀)
```C++
struct Node {
    map<pair<char, char>, Node*> node;
    int num; // 记录有多少个字符串, 比如 ["a", "a", "a"] 这样就需要 num = 3 啦

    Node() : node() {
        this->num = 0;
    }
};

class Solution {
    long long isPrefixAndSuffix(const string& s , Node* root) { // 添加和查询写在一起啦
        Node* tmp = root;
        Node* mae = nullptr;
        int n = s.size();
        long long res = 0;
        for (int i = 0; i < n; ++i) {
            mae = tmp;
            tmp = tmp->node[make_pair(s[i], s[n - 1 - i])];
            if (!tmp)
                tmp = mae->node[make_pair(s[i], s[n - 1 - i])] = new Node();
            if (tmp->num)
                res += tmp->num;
        }
        ++(tmp->num);
        return res;
    }

public:
    long long countPrefixSuffixPairs(vector<string>& words) {
        Node* root = new Node();
        long long res = 0;
        for (int i = 0; i < words.size(); ++i) {
            res += isPrefixAndSuffix(words[i], root);
        }
        return res;
    }
};
```

## 灵活运用
至于 字典树 的其他字段什么的, 什么时候应该怎么定义, 应该根据题目要求而定, 比如: [3093. 最长公共后缀查询](https://leetcode.cn/problems/longest-common-suffix-queries/description/)

AC代码
```C++
struct Node { // 字典树
    map<char, Node*> node;
    int index;
    int len;

    Node(int index) : node() {
        this->index = index;
        this->len = 1e7;
    }
};

void bui(int index, const string& s, Node* root) {
    Node* node = root;
    Node* mae = nullptr;
    for (int i = s.size() - 1; i >= 0; --i) {
        if (node->len > s.size()) {
            node->len = s.size();
            node->index = index;
        }

        mae = node;
        node = node->node[s[i]];
        if (!node)
            node = mae->node[s[i]] = new Node(index);
    }

    if (node->len > s.size()) { // 对于为什么要写两次, 而不是 写在上面的 if (!node) 后面
        node->len = s.size();   // 是因为, 题目的恶趣味: 不满足的时候当做有""个匹配(0个), 此时使用的是最短的 wordsContainer 中的字符串的索引最靠前的字符串 (当然你另开变量也不是不行)
        node->index = index;
    }
}

int find(const string& s, Node* root) {
    Node* node = root;
    Node* mae = nullptr;
    for (int i = s.size() - 1; i >= 0; --i) {
        mae = node;
        node = node->node[s[i]];
        if (!node)
            return mae->index;
    }
    return node->index;
}

class Solution {
public:
    vector<int> stringIndices(
        vector<string>& wordsContainer, 
        vector<string>& wordsQuery) {
        vector<int> res(wordsQuery.size());
        Node* root = new Node(-1);

        for (int i = 0; i < wordsContainer.size(); ++i) { // 构建字典树
            bui(i, wordsContainer[i], root);
        }
        
        for (int i = 0; i < wordsQuery.size(); ++i) {
            res[i] = find(wordsQuery[i], root);
        }

        return res;
    }
};
```
