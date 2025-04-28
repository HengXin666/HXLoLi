# 字典树

- 学习: [字典树](https://blog.HXLoLi.com/blog/#/articles?articleId=20558 "##20558##")

## 一、模版: 两种写法

- 例题: [2261. 含最多 K 个可整除元素的子数组](https://leetcode.cn/problems/k-divisible-elements-subarrays/)

### 1.1 指针式

```C++
struct Node {
    map<int, Node*> next{};  
};

class Solution {
public:
    int countDistinct(vector<int>& nums, int k, int p) {
        int res = 0, n = nums.size();
        Node* root = new Node;
        for (int i = 0; i < n; ++i) { // 从 i 开始, 一直构建子数组
            Node* node = root;
            Node* mae = nullptr;
            for (int j = i, cnt = 0; j < n; ++j) {
                int x = nums[j];
                if (x % p == 0 && ++cnt > k)
                    break;
                mae = node;
                node = node->next[x];
                if (!node) {
                    node = mae->next[x] = new Node;
                    ++res;
                }
            }
        }
        return res;
    }
};
```

比较好写, 但是因为使用`new`而没有`del`(因为del也需要时间), 因此可能没有那么工程...

### 1.2 数组哈希式

```C++
class Solution {
public:
    int countDistinct(vector<int>& nums, int k, int p) {
        /*
    题目要求的子数组: 子数组中最多 k 个可被 p 整除的元素
    
    什么是不同的子数组:
        长度不同 || (长度相同 && 里面的元素不同)
        */

        // 构建一个字典树
        // ne[i][j] 为 第 ne 索引为 i 的结点, 
        // 它的子结点是 j 对应的结点的字典树 ne索引为 ne[i][j]
        vector<map<int, int>> ne;
        // ps: 如果使用struct Node字典树的做法, 只需要一个全局的cnt
        // 以记录new的结点数量即可 (见上)

        auto get = [&]() -> int {
            ne.push_back(map<int, int>()); 
            return (int)ne.size() - 1; 
        };

        get(); // 生成字典树的根
        
        for (int i = 0; i < nums.size(); ++i) {
            for (int j = i, node = 0, cnt = 0; j < nums.size(); ++j) {
                // 题目的约束
                if (nums[j] % p == 0 && ++cnt > k) 
                    break;
                
                // 如果结点不存在, 则生成新结点
                if (!ne[node].count(nums[j])) {
                    ne[node][nums[j]] = get();
                }
                node = ne[node][nums[j]];
            }
        }
        
        return (int)ne.size() - 1;
    }
};
```

好处, RAII可以自动释放字典树; 但是写起来可能不是很顺? 尤其是在需要自定义的时候.

```C++
struct Node {
    map<int, int> next;
    int isEnd = false;
}

vector<Node> ne;
auto get = [&]() -> int {
    ne.push_back(Node{});
    return ne.size() - 1;
};

for (int j = i, node = 0, cnt = 0; j < nums.size(); ++j) {
    // 题目的约束
    if (nums[j] % p == 0 && ++cnt > k) 
        break;
    
    // 如果结点不存在, 则生成新结点
    if (!ne[node].next.count(nums[j])) {
        ne[node].next[nums[j]] = get();
    }
    node = ne[node].next[nums[j]];
}
```

当然, 你可以重载`[]`运算符, 这样又顺手了! (C++魅力时刻!)