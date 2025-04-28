# B-树代码实现
## By 01
### 头文件
```C
#ifndef DISK_BTREE_H
#define DISK_BTREE_H
/* B树的阶是MaxOrder（3），说明最大有3棵子树，那么节点最大键值为MaxOrder-1（2）
 * 在插入节点时，当键值数量为3时，进行分离，申请MaxOrder+1，从1索引开始填入键值
 * 多申请的k3，考虑到节点要存放3时，触发分离事件
 * | k0 | k1 | k2 | k3 |    k0无效    k3防越界
 * | p0 | p1 | p2 | p3 |
 * p0 < k1   k1 < p1 < k2  p2 > k2，p3是不会访问的
 * */
#define MaxOrder 3          // B树的阶
typedef int KeyType;        // B树节点键值类型

// B树的节点结构
typedef struct BTNode {
    KeyType key[MaxOrder + 1];          // [1 ... MaxOrder-1]
    struct BTNode *ptr[MaxOrder + 1];   // 0开始存入关系索引
    struct BTNode *parent;              // 指向父节点
    int keyNum;                         // 该节点的键值数，最多MaxOrder-1
}BTNode;

// B树的结构封装
typedef struct {
    BTNode *root;           // B树的根节点
    int count;              // B树的节点数量
} DiskBTree;
/* B树查找的结果集，包含查找成功和失败
 * ptr : 查找成功，标记为键值所在的节点地址
 *       查找失败，标记待插入节点的父节点地址
 * pos : 查找成功，标记为键值所在的节点的位序索引号
 *       查找失败，标记待插入节点的父节点的位序索引号
 * tag : 1表示查找成功，0表示查找失败
 * */
typedef struct {
    BTNode *ptr;
    int pos;
    int tag;
} Result;

DiskBTree *initDiskBTree();             // 初始化B树
void releaseDiskBTree(DiskBTree *tree); // 释放B树
// 查找B树中key的位置，分查找成功和失败，分别更新res
void searchBTree(DiskBTree *tree, KeyType key, Result *res);
// 将key值插入到B树
void insertKey(DiskBTree *tree, KeyType key);
// 将key值从B树删除
void deleteKey(DiskBTree *tree, KeyType key);
// 按层次打印B树
void PrintBTree(const BTNode *t, int tab);
// 按关键字大小升序输出B-树关键字
void orderTravelBTree(DiskBTree *tree);

#endif
```

### .c

```C
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "diskBTree.h"

static void restoreBTree(DiskBTree *tree, BTNode *node);

/* 从节点中，搜索符合条件的key的位序 */
static int searchNodePos(BTNode *node, KeyType key) {
    int i = 1;      // 从1号索引开始搜索
    // keyNum描述树阶-1的大小，当该节点找不到满足要求的，i就是最后的占位索引
    while (i <= node->keyNum && key > node->key[i]) {
        i++;
    }
    return i;
}

// ******************** B-Tree的插入操作 *************************** //
/* 将node节点分裂成两个节点，前一半保留在原节点，后一半移动到other节点
 * 1. 申请新节点，将后半部分信息拷贝到新节点
 *      1.1 中点后一个元素开始拷贝到新节点，中点是为了上移使用
 *      1.2 中点的p是新元素的0位置，从中点后一个元素，逐个更新px
 *      1.3 更新新节点的元素个数，其实是固定数字，阶数 - 中点索引值
 *      1.4 更新新节点的父信息，是原node节点的父亲
 * 2. 新申请节点的子节点的父信息也要更新
 *      2.1 从p0开始，更新到元素个数
 * 此时节点的keyNum正好是maxOrder，而传入的中间索引值也是自然序
 * 那么 n - s 表示的就是从s+1到最后的元素个数
 * */
static void split(BTNode **node, int s, BTNode **other) {
    int i = s + 1;      // 老节点搬移位置
    int j = 1;          // 新节点待放入位置
    int n = (*node)->keyNum;
    BTNode *splitNode = (BTNode *) malloc(sizeof(BTNode));
    memset(splitNode, 0, sizeof(BTNode));
    *other = splitNode;
    splitNode->ptr[0] = (*node)->ptr[s];
    for (; i <= n; ++i, ++j) {
        splitNode->key[j] = (*node)->key[i];
        splitNode->ptr[j] = (*node)->ptr[i];
        (*node)->key[i] = 0;
        (*node)->ptr[i] = 0;
    }
    // splitNode->keyNum = n - s;          // n肯定是最大阶数，不然不会split
    splitNode->keyNum = MaxOrder - s;
    splitNode->parent = (*node)->parent;
    for (i = 0; i <= n - s; ++i) {         // n-s的索引是最后一个有效元素的索引
        if (splitNode->ptr[i]) {
            splitNode->ptr[i]->parent = splitNode;
        }
    }
    (*node)->keyNum = s - 1;
}

/* 创建一个根节点，key表示k1，更新2个子节点信息，同时更新p0,p1的父节点 */
static BTNode *createRootNode(KeyType key, BTNode *p0, BTNode *p1) {
    BTNode *node = (BTNode *) malloc(sizeof(BTNode));
    memset(node, 0, sizeof(BTNode));
    node->keyNum = 1;
    node->key[1] = key;
    node->ptr[0] = p0;
    node->ptr[1] = p1;
    if (p0) {
        p0->parent = node;
    }
    if (p1) {
        p1->parent = node;
    }
    node->parent = NULL;
    return node;
}

/* 从节点node的pos位序处，插入关键字key，
 * 若是上移键，更新比key大的child子节点索引，即ptr[pos]处更新
 * 在叶子节点插入，child直接为null
 * */
static void insertNode(BTNode *node, int pos, KeyType key, BTNode *child) {
    // 1. 从key表中移动pos及pos后的元素，更新pos位置的值
    int i;
    int n = node->keyNum;
    for (i = n; i >= pos; --i) {
        node->key[i + 1] = node->key[i];
        node->ptr[i + 1] = node->ptr[i];
    }
    node->key[pos] = key;
    node->ptr[pos] = child;
    if (child) {
        child->parent = node;
    }
    node->keyNum++;
}

/* 将key插入到B树中的node节点的pos位序处，根据B树规则决定是否分离 */
static void insertBTree(DiskBTree *tree, KeyType key, BTNode *node, int pos) {
    // 在插入键值后，可能满足要求可能不满足要求，也可能需要更新根节点
    int finish = 0;
    int needNewRoot = 0;
    BTNode *child = NULL;
    int mid;
    KeyType x = key;

    if (node) { // 有父节点，要么直接插入在父节点，要么进行分离
        while (!finish && !needNewRoot) {
            insertNode(node, pos, x, child);
            if (node->keyNum < MaxOrder) {
                finish = 1;
            } else {        // 已经超过最大有效位置
                mid = (MaxOrder + 1) / 2;
                split(&node, mid, &child);
                x = node->key[mid];
                if (node->parent) {
                    node = node->parent;
                    pos = searchNodePos(node, x);
                } else {
                    needNewRoot = 1;
                }
            }
        }
    } else {    // 待插入节点的父节点是空，说明产生一个根节点
        tree->root = createRootNode(key, NULL, NULL);
        tree->count = 1;
    }
    if (needNewRoot) {
        tree->root = createRootNode(x, node, child);
    }
}

void insertKey(DiskBTree *tree, KeyType key) {
    Result res;
    // 1. 查找key在B树的待插入位置
    searchBTree(tree, key, &res);
    if (res.tag == 1) {
        printf("键值 %d 已经存在了!\n", key);
    } else {
        insertBTree(tree, key, res.ptr, res.pos);
        tree->count++;
    }
}

// ******************** B-Tree的删除操作 *************************** //
/* 向兄弟借关键字 */
static void borrowFromBrother(BTNode *node, BTNode *leftBrother, BTNode *rightBrother, BTNode *parent, int pos) {
    if (leftBrother && leftBrother->keyNum >= ((MaxOrder + 1) / 2)) {
        // 左兄弟有富余关键字，向左兄弟借
        for (int j = node->keyNum + 1; j > 0; --j) {
            // 关键字与指针后移，腾出第一个位置
            if (j > 1) {
                node->key[j] = node->key[j - 1];
            }
            node->ptr[j] = node->ptr[j - 1];
        }
        node->ptr[0] = leftBrother->ptr[leftBrother->keyNum];
        if (node->ptr[0]) {
            node->ptr[0]->parent = node;
        }
        leftBrother->ptr[leftBrother->keyNum] = NULL;
        node->key[1] = parent->key[pos];     // 被删节点存父节点关键字
        parent->key[pos] = leftBrother->key[leftBrother->keyNum];   // 父节点的key变为被删除节点左兄弟的最大值
        leftBrother->keyNum--;
        node->keyNum++;
    } else if (rightBrother && rightBrother->keyNum >= ((MaxOrder + 1) / 2)) {
        // 右兄弟有富余关键字
        node->key[node->keyNum + 1] = parent->key[pos + 1];
        node->ptr[node->keyNum + 1] = rightBrother->ptr[0];
        if (node->ptr[node->keyNum + 1]) {
            node->ptr[node->keyNum + 1]->parent = node;
        }
        node->keyNum++;
        parent->key[pos + 1] = rightBrother->key[1];
        for (int j = 0; j < rightBrother->keyNum; ++j) {
            if (j > 0) {
                rightBrother->key[j] = rightBrother->key[j + 1];
            }
            rightBrother->ptr[j] = rightBrother->ptr[j + 1];

        }
        rightBrother->ptr[rightBrother->keyNum] = NULL;
        rightBrother->keyNum--;
    }
}

/* 与左兄弟合并 */
static void mergerWithLeftBrother(BTNode *leftBrother, BTNode *parent, BTNode *node, DiskBTree *tree, int pos) {
    // 与左兄弟合并
    leftBrother->key[leftBrother->keyNum + 1] = parent->key[pos];   // 从父节点拿下分割本节点与左兄弟的关键字
    leftBrother->ptr[leftBrother->keyNum + 1] = node->ptr[0];
    if (leftBrother->ptr[leftBrother->keyNum + 1]) {
        // 给左兄弟的节点，当此结点存在时需要把其父亲指向左节点
        leftBrother->ptr[leftBrother->keyNum + 1]->parent = leftBrother;
    }
    leftBrother->keyNum++;      // 左兄弟关键字加1
    for (int i = 1; i <= node->keyNum; ++i) {
        // 把本节点的关键字和子树指针赋给左兄弟
        leftBrother->key[leftBrother->keyNum + i] = node->key[i];
        leftBrother->ptr[leftBrother->keyNum + i] = node->ptr[i];
        if (leftBrother->ptr[leftBrother->keyNum + i]) {
            leftBrother->ptr[leftBrother->keyNum + i]->parent = leftBrother;
        }
    }
    leftBrother->keyNum += node->keyNum;
    parent->ptr[pos] = NULL;
    free(node);
    for (int j = pos; j < parent->keyNum; ++j) {    // 左移
        parent->key[j] = parent->key[j + 1];
        parent->ptr[j] = parent->ptr[j + 1];
    }
    parent->ptr[parent->keyNum] = NULL;
    parent->keyNum--;       // 父节点关键字个数减1
    if (tree->root == parent) {
        // 如果此时父节点为根，则当父节点没有关键字时才调整
        if (parent->keyNum == 0) {
            for (int i = 0; i <= parent->keyNum + 1; ++i) {
                if (parent->ptr[i]) {
                    tree->root = parent->ptr[i];
                    break;
                }
                tree->root->parent = NULL;
            }
        }
    } else {
        // 如果父节点不为根，则需要判断是否需要重新调整
        if (parent->keyNum < ((MaxOrder - 1) / 2)) {
            restoreBTree(tree, parent);
        }
    }
}

/* 与右兄弟合并 */
static void mergerWithRightBrother(BTNode *rightBrother, BTNode *parent, BTNode *node, DiskBTree *tree, int pos) {
    for (int i = rightBrother->keyNum; i > 0; --i) {
        if (i > 0) {
            rightBrother->key[i + 1 + node->keyNum] = rightBrother->key[i];
        }
        rightBrother->ptr[i + 1 + node->keyNum] = rightBrother->ptr[i];
    }
    rightBrother->key[node->keyNum + 1] = parent->key[pos + 1];     // 把父节点分割两个本兄弟和右兄弟的关键字拿下来使用
    for (int i = 0; i <= node->keyNum; ++i) {
        // 把本结点的关键字及子树指针移动到右兄弟中去
        if (i > 0) {
            rightBrother->key[i] = node->key[i];
        }
        rightBrother->ptr[i] = node->ptr[i];
        if (rightBrother->ptr[i]) {
            rightBrother->ptr[i]->parent = rightBrother;        // 给右兄弟的节点需要把其父节点指向右兄弟
        }
    }
    rightBrother->keyNum += (node->keyNum + 1);
    parent->ptr[pos] = NULL;
    free(node);
    for (int i = pos; i < parent->keyNum; ++i) {
        if (i > pos) {
            parent->key[i] = parent->key[i + 1];
        }
        parent->ptr[i] = parent->ptr[i + 1];
    }
    if (parent->keyNum == 1) {
        // 如果父结点在关键字减少之前只有一个结点，那么需要把父结点的右孩子赋值给左孩子
        parent->ptr[0] = parent->ptr[1];
    }
    parent->ptr[parent->keyNum] = NULL;
    parent->keyNum--;           // 父节点关键字数减1
    if (tree->root == parent) {
        //如果此时父结点为根，则当父结点没有关键字时才调整
        if (parent->keyNum == 0) {
            for (int i = 0; i <= parent->keyNum; ++i) {
                if (parent->ptr[i]) {
                    tree->root = parent->ptr[i];
                    break;
                }
            }
            tree->root->parent = NULL;
        }
    } else {
        // 如果父结点不为根，则需要判断是否需要重新调整
        if (parent->keyNum < ((MaxOrder - 1) / 2)) {
            restoreBTree(tree, parent);
        }
    }
}

/* 调整B树，node为需调整的节点 */
static void restoreBTree(DiskBTree *tree, BTNode *node) {
    // 待调整节点的父节点、左右兄弟节点
    BTNode *parent, *leftBrother, *rightBrother;
    parent = node->parent;
    if (parent) {
        // 寻找左右兄弟
        int i;
        for (i = 0; i <= parent->keyNum; ++i) {
            if (parent->ptr[i] == node) {
                break;
            }
        }
        if (i > 0) {
            leftBrother = parent->ptr[i - 1];
        } else {
            leftBrother = NULL;
        }
        if (i < parent->keyNum) {
            rightBrother = parent->ptr[i + 1];
        } else {
            rightBrother = NULL;
        }
        // 左兄弟或右兄弟有富余的关键字
        if ((leftBrother && leftBrother->keyNum >= (MaxOrder + 1) / 2) ||
            (rightBrother && rightBrother->keyNum >= (MaxOrder + 1) / 2)) {
            borrowFromBrother(node, leftBrother, rightBrother, parent, i);
        } else {
            // 左右兄弟都没有富余的关键字，需要合并
            if (leftBrother) {
                // 与左兄弟合并
                mergerWithLeftBrother(leftBrother, parent, node, tree, i);
            } else if (rightBrother) {
                mergerWithRightBrother(rightBrother, parent, node, tree, i);
            } else {
                // 当左右子树不存在时，改变根节点
                for (int j = 0; j < node->keyNum; ++j) {
                    if (node->ptr[j]) {
                        tree->root = node->ptr[j];
                        break;
                    }
                }
                tree->root->parent = NULL;
            }
        }
    } else {
        // 根节点，去掉根节点，使树减一层
        BTNode *a;
        for (int j = 0; j <= node->keyNum; ++j) {
            if (node->ptr[j]) {
                a = node;
                node = node->ptr[j];
                a->ptr[j] = NULL;
                free(a);
                break;
            }
        }
        tree->root = node;
        tree->root->parent = NULL;
    }
}

/* 找到node后继节点最小关键字，交换值并更新待删除的key信息 */
static void successor(BTNode **node, int pos) {
    BTNode *leaf = *node;
    if (leaf == NULL) {
        return;
    }
    leaf = leaf->ptr[pos];
    while (leaf->ptr[0]) {
        leaf = leaf->ptr[0];
    }
    (*node)->key[pos] = leaf->key[1];
    (*node) = leaf;
}

/* 从节点node的pos位序中移除关键字key */
static void removeNode(BTNode *node, int pos) {
    for (int i = pos; i < node->keyNum; ++i) {
        node->key[i] = node->key[i + 1];
        node->ptr[i] = node->ptr[i + 1];
    }
    node->keyNum--;
}

/* 从tree中删除目标关键字所在node节点的pos位序上 */
static void deleteBTree(DiskBTree *tree, BTNode *node, int pos) {
    if (node->ptr[pos]) {   // 内部节点，非终端节点
        successor(&node, pos);          // 找到后继节点中最小的关键字替代
        deleteBTree(tree, node, 1);     // 删除最下层非终端节点中的最小关键字
    } else {                // 从node节点中删除pos
        removeNode(node, pos);
        if (node->keyNum < ((MaxOrder - 1) / 2)) {
            restoreBTree(tree, node);       // 调整B树
        }
    }
}

/* 从B-tree树上删除key值 */
void deleteKey(DiskBTree *tree, KeyType key) {
    Result res;
    searchBTree(tree, key, &res);
    if (res.tag) {
        deleteBTree(tree, res.ptr, res.pos);
        tree->count--;
    } else {
        printf("关键字key不在B树上!\n");
    }
}

// ******************** B-Tree的初始化操作 *************************** //
DiskBTree *initDiskBTree() {
    DiskBTree *bTree = (DiskBTree *) malloc(sizeof(DiskBTree));
    bTree->root = NULL;
    bTree->count = 0;
    return bTree;
}

static void destroyBTNode(DiskBTree *tree, BTNode *node) {
    if (node) {
        for (int i = 0; i <= node->keyNum; ++i) {
            if (node->ptr[i]) {
                destroyBTNode(tree, node->ptr[i]);
            }
        }
        free(node);
    }
}

void releaseDiskBTree(DiskBTree *tree) {
    if (tree) {
        destroyBTNode(tree, tree->root);
        free(tree);
    }
}

// ******************** B-Tree的查找操作 *************************** //
void searchBTree(DiskBTree *tree, KeyType key, Result *res) {
    BTNode *cur = tree->root;       // 定义当前节点索引
    BTNode *pre = NULL;             // 定义指向父节点索引
    int tag = 0;                    // 标记成功或失败状态，默认未找到
    int pos;                        // 节点中的位序
    while (cur && !tag) {
        pos = searchNodePos(cur, key);
        if (pos <= cur->keyNum && cur->key[pos] == key) {
            tag = 1;
        } else {
            pre = cur;
            cur = pre->ptr[pos - 1];  // 找不到pos就越界，调整到最后一个索引位置
        }
    }
    if (tag) {      // 查找成功，就更新找到的节点信息
        res->tag = 1;
        res->ptr = cur;
        res->pos = pos;
    } else {        // 查找失败，就把待插入节点的父节点信息更新
        res->tag = 0;
        res->ptr = pre;
        res->pos = pos;
    }
}

void PrintBTree(const BTNode *node, int tab) {
    if (!node) {
        return;
    }
    int i;
    for (i = 0; i <= tab; ++i) {
        printf("    ");
    }
    for (i = 1; i <= node->keyNum; i++) {
        printf("%d", node->key[i]);
        if (i != node->keyNum) {
            printf(", ");
        }
    }
    printf("\n");
    for (i = 0; i <= node->keyNum; i++) {
        PrintBTree(node->ptr[i], tab + 1);
    }
}

static void inOrderBTNode(BTNode *node) {
    if (node) {
        for (int i = 0; i <= node->keyNum; ++i) {
            if (i > 0) {
                printf("%d ", node->key[i]);
            }
            if (node->ptr[i]) {
                inOrderBTNode(node->ptr[i]);
            }
        }
    }
}

void orderTravelBTree(DiskBTree *tree) {
    if (tree) {
        printf("BTree order: ");
        inOrderBTNode(tree->root);
        printf("\n");
    }
}
```

### main.c

```C
#include <stdio.h>
#include "diskBTree.h"

int main() {
    DiskBTree *bTree = initDiskBTree();

    insertKey(bTree, 8);
    insertKey(bTree, 9);
    insertKey(bTree, 10);
    insertKey(bTree, 11);
    insertKey(bTree, 15);
    insertKey(bTree, 16);
    insertKey(bTree, 17);
    insertKey(bTree, 18);
    insertKey(bTree, 20);
    insertKey(bTree, 23);
    PrintBTree(bTree->root,1);

    printf("====================\n");
    deleteKey(bTree, 15);
    PrintBTree(bTree->root,1);
    orderTravelBTree(bTree);

    Result res;
    searchBTree(bTree, 15, &res);
    if (res.tag) {
        printf("find: %d\n", res.ptr->key[res.pos]);
    } else {
        printf("Not find!\n");
    }
    releaseDiskBTree(bTree);
    return 0;
}
```