# LFU 缓存
题目: [460. LFU 缓存](https://leetcode.cn/problems/lfu-cache/)

> 直接看灵神: [【图解】一张图秒懂 LFU！（Python/Java/C++/Go/JS/Rust）](https://leetcode.cn/problems/lfu-cache/solutions/2457716/tu-jie-yi-zhang-tu-miao-dong-lfupythonja-f56h/)

```C++
class LFUCache {
    struct Node {
        Node* prev = nullptr;
        Node* next = nullptr;
        int key = 0;
        int val = 0;
        int cnt = 1;

        Node() {}
        Node(int k, int v) : key(k) , val(v) {}
    };

    unordered_map<int, Node *> hash;
    unordered_map<int, Node *> headHash;
    int maxLen;
    int minNodeCnt = 1;

    // 删除这个节点 (A->B->C) => (A->C)
    void remove(Node* node) {
        node->prev->next = node->next;
        node->next->prev = node->prev;
    }

    // 头插
    void push_front(Node* head, Node* node) {
        node->next = head->next;
        head->next->prev = node;
        node->prev = head;
        head->next = node;
    }

    // 如果这个头结点为空, 则删除
    void tryDelHeadNode(int cnt) {
        auto it = headHash.find(cnt);
        Node* head = it->second;
        if (head->prev == head) {
            // 为空
            headHash.erase(it);
            // [难点]: 如果最左边的为空, 则最小频率加 1
            if (head->cnt == minNodeCnt)
                ++minNodeCnt;
            delete head;
        }
    }

    // 如果不存在则创建新的头结点
    Node* tryNewHeadNode(int cnt) {
        auto it = headHash.find(cnt);
        if (it == headHash.end()) {
            Node* head = new Node;
            head->cnt = cnt;
            head->prev = head->next = head;
            headHash.insert({cnt, head});
            return head;
        }
        return it->second;
    }

    // 更新该节点到新的头节点头插
    void upNodeToTop(Node* node) {
        // 取出
        remove(node);
        // 如果只有头节点就删了
        tryDelHeadNode(node->cnt);
        // 头插
        push_front(tryNewHeadNode(++node->cnt), node);
    }

    // 头插一个新的
    void topInsert(int key, int value) {
        Node* node = hash[key] = new Node(key, value);
        // 头插
        push_front(tryNewHeadNode(1), node);
    }

    // 删除一个尾的
    void pop_back() {
        Node* head = headHash[minNodeCnt];
        Node* node = head->prev;
        hash.erase(node->key);
        // 更新头结点
        head->prev = node->prev;
        // 取出
        remove(node);
        delete node;
        // 看看是不是只有头结点了
        tryDelHeadNode(head->cnt);
    }
public:
    LFUCache(int capacity)
        : hash()
        , headHash()
        , maxLen(capacity) {
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
            if (hash.size() == maxLen) {
                // 删除尾结点
                pop_back();
            }
            topInsert(key, value);
            minNodeCnt = 1;
        }
    }
};

/**
 * Your LFUCache object will be instantiated and called as such:
 * LFUCache* obj = new LFUCache(capacity);
 * int param_1 = obj->get(key);
 * obj->put(key,value);
 */
```