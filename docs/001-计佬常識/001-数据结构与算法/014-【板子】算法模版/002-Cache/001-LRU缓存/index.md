# LRU 缓存
题目: [146. LRU 缓存](https://leetcode.cn/problems/lru-cache/)

超级热门的面试题!

> 灵神: [【图解】一张图秒懂 LRU！（Python/Java/C++/Go/JS/Rust）](https://leetcode.cn/problems/lru-cache/solutions/2456294/tu-jie-yi-zhang-tu-miao-dong-lrupythonja-czgt/) (有图直接秒了!)

```C++
class LRUCache {
    struct Node {
        Node* prev = nullptr;
        Node* next = nullptr;
        int key = 0;
        int val = 0;
    };

    Node* head; // 头节点(哨兵)
    unordered_map<int, Node *> hash;
    int maxLen;

    // 删除这个节点 (A->B->C) => (A->C)
    void remove(Node* node) {
        node->prev->next = node->next;
        node->next->prev = node->prev;
    }

    // 头插
    void push_front(Node* node) {
        node->next = head->next;
        head->next->prev = node;
        node->prev = head;
        head->next = node;
    }

    // 更新该节点到头节点头插
    void upNodeToTop(Node* node) {
        // 取出
        remove(node);
        // 头插
        push_front(node);
    }

    // 头插一个新的
    void topInsert(int key, int value) {
        Node* node = new Node;
        node->key = key;
        node->val = value;
        hash[key] = node;
        // 头插
        push_front(node);
    }

    // 删除一个尾的
    void pop_back() {
        Node* node = head->prev;
        hash.erase(node->key);
        // 更新头结点
        head->prev = node->prev;
        // 取出
        remove(node);
        delete node;
    }
public:
    LRUCache(int capacity)
        : head(new Node)
        , hash()
        , maxLen(capacity) {
        head->next = head->prev = head;
    }
    
    int get(int key) {
        if (auto it = hash.find(key); 
            it != hash.end()) {
            upNodeToTop(it->second);
            return it->second->val;
        }
        return -1;
    }
    
    void put(int key, int value) {
        if (auto it = hash.find(key); 
            it != hash.end()) {
            upNodeToTop(it->second);
            it->second->val = value;
        } else {
            // 插入一个新的在头
            topInsert(key, value);
            if (hash.size() > maxLen) {
                // 删除尾结点
                pop_back();
            }
        }
    }
};

/**
 * Your LRUCache object will be instantiated and called as such:
 * LRUCache* obj = new LRUCache(capacity);
 * int param_1 = obj->get(key);
 * obj->put(key,value);
 */
```
