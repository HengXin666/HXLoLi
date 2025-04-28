# 手撕红黑树
## 数据结构
首先我们需要设计一些数据结构

因为是二叉树的一种, 那肯定是要有左右子树的.

又因为红黑树涉及旋转, 染色操作, 经常需要切换到父节点/叔节点/祖先结点等, 如果是使用递归回溯的话, 可能会使得平衡操作变得更加复杂和低效<sup>[By GPT-3.5]</sup>, 因此还需要设计父节点.

那么`struct`的雏形已经出现了, 然后就是显然:

```C
typedef int keyType;
typedef int RBcolor;

enum RBcolor {
    RED,
    BLACK
};

typedef struct _rbNode {
    keyType key;            // 关键字
    struct _rbNode *left;   // 左子树
    struct _rbNode *right;  // 右子树
    struct _rbNode *parent; // 父结点
    RBcolor color;          // 颜色
} RBNode;                   // 红黑树一个节点

typedef struct {
    RBNode *root;           // 根
    int count;              // 节点数
} RBTree;                   // 红黑树表头
```

## 涉及的方法

```C
RBTree *initRBTree(void);                               // 初始化红黑树
void insertRBNode(RBTree *tree, keyType key);           // 红黑树插入节点
void deleteRBTree(RBTree *tree, keyType key);           // 删除红黑树节点
void freeRBTree(RBTree *T);                             // 释放红黑树

// 拓展
void putRBTree(const RBTree *Tree);                     // 打印红黑树

// 女生自用九九新函数 (static)
void RBTreeLeftRotate(RBTree* tree, RBNode* y);         // 红黑树左旋
void RBTreeRightRotate(RBTree* tree, RBNode* y);        // 红黑树右旋
```

## 完整代码

```C
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

typedef int keyType;

typedef int RBcolor;

enum RBcolor {RED, BLACK};

typedef struct _rbNode
{
    keyType key;            // 关键字
    struct _rbNode *left;   // 左子树
    struct _rbNode *right;  // 右子树
    struct _rbNode *parent; // 父结点
    RBcolor color;          // 颜色
} RBNode;                   // 红黑树一个节点

typedef struct
{
    RBNode *root;           // 根
    int count;              // 节点数
} RBTree;                   // 红黑树表头

RBTree *initRBTree(void);                               // 初始化红黑树
void insertRBNode(RBTree *tree, keyType key);           // 红黑树插入节点
void deleteRBTree(RBTree *tree, keyType key);           // 删除红黑树节点
void RBTreeLeftRotate(RBTree* tree, RBNode* y);         // 红黑树左旋
void RBTreeRightRotate(RBTree* tree, RBNode* y);        // 红黑树右旋
void putRBTree(const RBTree *Tree);                     // 打印红黑树
void freeRBTree(RBTree *T);                             // 释放红黑树

static void *errorPrint(const char *str);               // 输出错误的女生自用函数

static void *errorPrint(const char *str)
{
    printf(str);
    return NULL;
}

RBTree *initRBTree(void)
{
    RBTree *T = (RBTree *)malloc(sizeof(RBTree));
    if (!T)
        return errorPrint("Malloc RBTree Error!\n");

    T->count = 0;
    T->root = NULL;

    return T;
}

// 初始化结点 并且着色为红
static RBNode *_initRBNode(keyType key)
{
    RBNode *node = (RBNode *)malloc(sizeof(RBNode));
    if (!node)
        return errorPrint("Malloc RBNode Error!\n");
    
    node->key = key;
    node->color = RED;  // 插入都是红色的
    node->left = NULL;
    node->right = NULL;
    node->parent = NULL;

    return node;
}

// 调整红黑树
/**
 * 1. 判断是否出现[红红]
 *      1.1 否 --> 结束
 *      1.2 是 --> 2.
 * 
 * 2. 判断插入节点的叔叔节点的着色
 *      2.1 着色为红
 *          修改着色即可: <父, 叔>节点着色为黑, <祖先>节点着色为红
 *      2.2 着色为黑
 *          >> 判断失衡类型 << 
 *          LL: 对祖先节点右旋
 *          LR: 对父亲结点左旋, 然后是xp交换, 就是LL
 *          RR: 对祖先结点左旋
 *          RL: 对父亲结点右旋, 然后是xp交换, 就是RR
 */
static void _adjustRBTree(RBTree* tree, RBNode *node)
{
    RBNode *parent = node->parent;
    RBNode *uncle;
    RBNode *grandfather;
    RBNode *tmp = NULL;

    // 红红
    while (parent && parent->color == RED)
    {
        grandfather = parent->parent;
        uncle = (grandfather->left == parent) ? grandfather->right : grandfather->left;
        
        // 叔叔节点为红 (注意要考虑可能叔叔是NUL)
        if (uncle && uncle->color == RED)
        {
            // 更改着色
            parent->color = BLACK;
            uncle->color = BLACK;
            grandfather->color = RED;

            // 更新节点
            node = grandfather;
            parent = grandfather->parent;
            continue;
        }

        // 叔叔节点为黑
        // 判断失衡类型
        if (grandfather->left == parent)    // L
        {
            if (parent->right == node)      // R
            {
                RBTreeLeftRotate(tree, parent);
                // xp交换
                tmp = node;
                node = parent;
                parent = tmp;
            }
            RBTreeRightRotate(tree, grandfather);
            // 着色
            grandfather->color = RED;
            parent->color = BLACK;
        }
        else                                // R
        {
            if (parent->left == node)       // L
            {
                RBTreeRightRotate(tree, parent);
                // xp交换
                tmp = node;
                node = parent;
                parent = tmp;
            }
            RBTreeLeftRotate(tree, grandfather);
            // 着色
            grandfather->color = RED;
            parent->color = BLACK;
        }
        // node = grandfather;
        break;  // 旋转不会while了! (不用递归判断)
    }

    if (node == tree->root)
    {
        node->color = BLACK;
        node->parent = NULL;
    }
}

void insertRBNode(RBTree *tree, keyType key)
{
    // 1. 生成节点
    RBNode *node = _initRBNode(key);

    // 2. 插入节点 (同二叉搜索树)
    RBNode *cur = tree->root;
    RBNode *tmp = NULL;
    while (cur)
    {
        tmp = cur;

        // 左小右大
        if (key < cur->key)
        {
            cur = cur->left;
        }
        else if (key > cur->key)
        {
            cur = cur->right;
        }
        else
        {
            printf("ERROR: Var is onaji!\n");
            return;
        }
    }
    // cur为空, 说明当前就是需要插入的位置!

    if (tmp)
    {
        if (key < tmp->key)
        {
            tmp->left = node;
        }
        else
        {
            tmp->right = node;
        }
        node->parent = tmp;
    }
    else
    {
        tree->root = node;
        node->color = BLACK;
        node->parent = NULL;
    }

    // 3. 调整节点
    _adjustRBTree(tree, node);

    ++tree->count;
}

static RBNode *_searchRBNode(RBTree *tree, keyType key)
{
    RBNode *node = tree->root;
    while (node)
    {
        if (key < node->key)
        {
            node = node->left;
        }
        else if (key > node->key)
        {
            node = node->right;
        }
        else
        {
            // 找到了
            return node;
        }
    }

    printf("没有找到: %d\n", key);
    return NULL;
}

static void _fixupRBTree(RBTree *tree, RBNode *y, RBNode *x)
{
    while (1)
    {
        RBNode *parent = y->parent;
        RBNode *w = NULL;
        // 判断L 还是 R
        if (parent->right == y)  // L (删除了R侧)
        {
            w = parent->left;
            if ((w->left && w->left->color == BLACK) && (w->right && w->right->color == RED)) // LR
            {
                RBTreeLeftRotate(tree, w);
                // 同xp交换 (指针指向改变, 因为旋转了, 所以实际上辈分已经变了)
                w = w->parent;
                // 染色
                w->color = BLACK;
                w->left->color = RED;
            }

            if (w->left && w->left->color == RED)
            {
                // LL
                w->left->color = w->color;
                w->color = parent->color;
                RBTreeRightRotate(tree, parent);
                parent->color = BLACK;
            }
            else if (w->color == BLACK)
            {
                // x的兄弟为黑节点
                w->color = RED;
                // 往上判断, 直到发现一个父结点为红, 或者根节点
                while (parent)
                {
                    if (parent->color == RED)
                    {
                        // 终止循环
                        parent->color = BLACK;
                        parent->left->color = RED;
                        break;
                    }
                    else
                    {
                        parent->left->color = RED;
                    }
                    parent = parent->parent;
                }
            }
            else
            {
                // x的兄弟为红节点 (在左边)
                // 染色
                w->color = BLACK;
                parent->color = RED;
                // 右旋父节点
                RBTreeRightRotate(tree, parent);
                // 变为之前的情况
                continue;
            }
        }
        else                    // R (删除了L侧)
        {
            w = parent->right;
            if ((w->right && w->right->color == BLACK) && (w->left && w->left->color == RED)) // RL
            {
                RBTreeRightRotate(tree, w);
                // 同xp交换 (指针指向改变, 因为旋转了, 所以实际上辈分已经变了)
                w = w->parent;
                // 染色
                w->color = BLACK;
                w->right->color = RED;
            }

            if (w->right && w->right->color == RED) // RR
            {
                // RR
                w->right->color = w->color;
                w->color = parent->color;
                RBTreeLeftRotate(tree, parent);
                parent->color = BLACK;
            }
            else if (w->color == BLACK)
            {
                // x的兄弟为黑节点
                w->color = RED;
                while (parent)
                {
                    if (parent->color == RED)
                    {
                        // 终止循环
                        parent->color = BLACK;
                        parent->right->color = RED;
                        break;
                    }
                    else
                    {
                        parent->right->color = RED;
                    }
                    parent = parent->parent;
                }
            }
            else
            {
                // x的兄弟为红节点 (在右边)
                // 染色
                w->color = BLACK;
                parent->color = RED;
                // 左旋父节点
                RBTreeLeftRotate(tree, parent);
                // 变为之前的情况
                continue;
            }
        }
        break;
    }
}

static void _deleteRBNode(RBTree *tree, RBNode *node)
{
    RBNode *y = node;   // 待删除
    RBNode *x = NULL;   // 用于替换

    // 如果这个节点度为二, 那么与该节点后继节点进行值交换, 然后 node 是指向后继节点的指针
    if (node->left && node->right)
    {
        // 度为二
        y = node->right;
        while (y->left)
        {
            y = y->left;
        }
        node->key = y->key;
    }
    
    // 因为 y度为一
    x = y->left ? y->left : y->right;

    // 度为一
    // 判断 x, y节点是否有一个是红, 注意: x需要存在哦~, 另外还有一种情况是: y是根, 并且度为0
    if (y->color == RED || (x && x->color == RED) || (tree->root == y && !x))
    {
        // [简单情况]
        // 替换与着色为黑即可
        if (x)
            x->parent = y->parent;

        if (y->parent)
        {
            if (y->parent->left == y)
            {
                y->parent->left = x;
            }
            else
            {
                y->parent->right = x;
            }
        }
        else
        {
            // y为根节点
            if (x)
            {
                tree->root = x;
                x->color = BLACK;
            }
            else
                tree->root = NULL;
        }
    }
    else    // 为双黑节点
    {
        // [复杂情况]
        // 判断 y 是不是根节点 <tm的这种情况下, 根本不可能出现 y 为根节点>
        _fixupRBTree(tree, y, x);
        if (y->parent->left == y)
        {
            y->parent->left = x;
        }
        else
        {
            y->parent->right = x;
        }
    }
    
    free(y);
}

void deleteRBTree(RBTree *tree, keyType key)
{
    // 1. 寻找需要删除的节点
    RBNode *node = _searchRBNode(tree, key);

    // 2. 删除该节点
    if (node)
    {
        _deleteRBNode(tree, node);
    }

    --tree->count;
}

/** 左旋 <叔叔为黑> // 将y进行左旋
 *          py                      py
 *          |                       |
 *          y                       x
 *         / \          -->        / \
 *        ly  x                   y   rx
 *           / \                 / \
 *          lx  rx              ly  lx
 */
void RBTreeLeftRotate(RBTree* tree, RBNode* y)
{
    RBNode* x = y->right;
    x->parent = y->parent;
    y->right = x->left;
    if (x->left)
    {
        x->left->parent = y;
    }
    x->left = y;
    
    if (y->parent)
    {
        if (y->parent->left == y)
        {
            y->parent->left = x;
        }
        else
        {
            y->parent->right = x;
        }
    }
    else
    {
        tree->root = x;
    }
    y->parent = x;
}

/** 右旋 <叔叔为黑> // 将y进行右旋
 *          py                      py
 *          |                       |
 *          y                       x
 *         / \          -->        / \
 *        x   ry                  lx  y
 *       / \                         / \
 *      lx  rx                      rx  ry
 */
void RBTreeRightRotate(RBTree* tree, RBNode* y)
{
    RBNode *x = y->left;
    x->parent = y->parent;
    y->left = x->right;

    // 记得 rx 的父也变了, 所以 parent 要改, 但是前提是 rx != NULL
    if (y->left)
    {
        y->left->parent = y;
    }

    // 如果 y原本是根, 那么x的地位要变
    if (y->parent)
    {
        // 非空, 那么要更新 到底是谁指向的x, 左还是右
        /**       
         *       /         \
         *      pl          pr
         *     /      还是    \
         *    y                y
         *      上头的 p->left 或者 p->right 是需要更新的!, 不然就还是指向原来的y了
         */
        if (y->parent->left == y)   // 左
        {
            y->parent->left = x;
        }
        else                        // 右
        {
            y->parent->right = x;
        }
    }
    else
    {
        // 为空, 说明是根
        tree->root = x;
    }
    x->right = y;
    y->parent = x;
}

static void _putRBNode(RBNode *node, keyType parentKey, int tag)
{
    if (!node)
        return;
    printf("%d<%s> 是 %d 的 %s\n", node->key, (node->color == RED) ? "红色" : "黑色", parentKey, (tag == 1) ? "右儿子" : (tag == -1) ? "左儿子" : "根结点");
    _putRBNode(node->left, node->key, -1);
    _putRBNode(node->right, node->key, 1);
}

void putRBTree(const RBTree *Tree)
{
    RBNode *p = Tree->root;
    printf("The RB-Tree Node Num: %d\n", Tree->count);
    _putRBNode(Tree->root, -1, 0);
}

int main(void)
{
    // 高阶搜索树 - 红黑树 - 已完善删除的(b, c)情况
    RBTree *RB_Tree = initRBTree();

    for (int i = 10; i <= 180; i += 10)
        insertRBNode(RB_Tree, i);

    putRBTree(RB_Tree);

    deleteRBTree(RB_Tree, 10);  // 情况(b)
    deleteRBTree(RB_Tree, 150); // 情况(a)
    deleteRBTree(RB_Tree, 130); // 情况(c)
    printf("------\n");
    putRBTree(RB_Tree);

    getchar();
    return 0;
}
```
