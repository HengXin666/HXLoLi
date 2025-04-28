# 代码实现
## 前序遍历
### 递归
```C
void 前序遍历(BTree *root, BTNode *node) {
    if (!node) {
        return;
    }

    // 需要做的事情
    printf("%d ", node->data);
    前序遍历(root, node->left);
    前序遍历(root, node->right);
}
```
### 非递归

```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

#define MAXSIZE 16      // 栈的大小

typedef int State;      // 状态 OK or ERROR
typedef char ElemType;

typedef struct BiTNobe
{
    ElemType data;
    struct BiTNobe *lchild, *rchild;// 左孩子, 右孩子
} BiTNobe, *BiTree;

typedef struct SqStack
{
    BiTNobe *data[MAXSIZE];
    int top;// 用于栈顶的指针
}SqStack;

SqStack *initStack(void);               // 初始化操作, 建立一个空栈的S
State destroyStack(SqStack **S);        // 销毁栈S

State pushStack(SqStack **S, BiTNobe * e);   // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
State popStack(SqStack **S, BiTNobe **e);    // 删除栈顶元素, 并用e返回其值 


void PreOrderTraveres(BiTree T);            // 前序遍历
State initTree(BiTree *T);                  // 初始化树
State createBiTree(BiTree *T);              // 创建一颗二叉树
State printStack(SqStack *S);               // 先出先打印

// ----------------------- 前序遍历 - 需要掌握 -----------------------
/* 非递归的先序遍历，引入栈
 * 1. 先压入根节点到栈
 * 2. 出栈出任务
 * 3. 打印任务，分配任务的机制，有右先压右，有左再压左<先右后左>
 * 4. 取任务，回到2
 * 5. 退出条件：栈空
 * */
void PreOrderTraveres(BiTree T)
{
    // 前序遍历 - 非递归
    SqStack *stack = initStack();
    BiTNobe *node = T;
    pushStack(&stack, T);
    while (popStack(&stack, &node))
    {
        if (node->rchild)
        {
            pushStack(&stack, node->rchild);
        }

        if (node->lchild)
        {
            pushStack(&stack, node->lchild);
        }
    }
    destroyStack(&stack);
}

State initTree(BiTree *T)
{
    *T = NULL;
    return OK;
}

State createBiTree(BiTree *T)
{
    char c;// 问题的没有, 是输入格式和0的不一样
    // 结点为叶结点则2空格
    scanf("%c", &c);
    if (c == ' ')
    {
        *T = NULL;// 不用释放是因为压根就没有分配
    }
    else
    {
        *T = (BiTree)malloc(sizeof(BiTNobe));
        (*T)->data = c;
        createBiTree(&(*T)->lchild);
        createBiTree(&(*T)->rchild);
    }
    return OK;
}



SqStack *initStack(void)
{
    // 初始化操作, 建立一个空栈的S
    SqStack *S = (SqStack *)malloc(sizeof(SqStack));
    S->top = -1;
    return S;
}

State pushStack(SqStack **S, BiTNobe *e)
{
    // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
    if ((*S)->top == MAXSIZE - 1)
    {
        return ERROR;
    }

    (*S)->data[++((*S)->top)] = e;
    return OK;
}

State popStack(SqStack **S, BiTNobe **e)
{
    // 删除栈顶元素, 并用e返回其值 
    if ((*S)->top == -1)
    {
        return ERROR;
    }
    *e = (*S)->data[((*S)->top)--];
    
    /* 执行的操作 */
    printf("%c ", (*e)->data);
    return OK;
}

State printStack(SqStack *S)
{
    // 先出先打印

    for (int i = S->top; i > -1; i--)
    {
        printf("%d ", S->data[i]);
    }

    return OK;
}

State destroyStack(SqStack **S)
{
    free(*S);
    *S = NULL;
    return OK;
}

int main(void)
{
    // 栈 的 数组(顺序存储结构)表示
    // 二叉树的非递归前序遍历
    BiTree bitree;
    initTree(&bitree);
    printf("请输入:");
    createBiTree(&bitree);
    printf("已接受!\n");
    PreOrderTraveres(bitree);

    getchar();
    getchar();
    return 0;
}
```


## 中序遍历
### 递归
```C
void 中序遍历(BTree *root, BTNode *node) {
    if (!node) {
        return;
    }

    中序遍历(root, node->left);
    // 需要做的事情
    printf("%d ", node->data);
    中序遍历(root, node->right);
}
```
### 非递归

```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

#define MAXSIZE 16      // 栈的大小

typedef int State;      // 状态 OK or ERROR
typedef char ElemType;

typedef struct BiTNobe
{
    ElemType data;
    struct BiTNobe *lchild, *rchild;// 左孩子, 右孩子
} BiTNobe, *BiTree;

typedef struct SqStack
{
    BiTNobe *data[MAXSIZE];
    int top;// 用于栈顶的指针
}SqStack;

SqStack *initStack(void);            // 初始化操作, 建立一个空栈的S
State destroyStack(SqStack **S);     // 销毁栈S

State pushStack(SqStack **S, BiTNobe * e);   // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
State popStack(SqStack **S, BiTNobe **e);    // 删除栈顶元素, 并用e返回其值
State ifStackSky(SqStack *S);                // 判断栈是否有内容


void PreOrderTraveres(BiTree T);            // 中序遍历
State initTree(BiTree *T);                  // 初始化树
State createBiTree(BiTree *T);              // 创建一颗二叉树
State printStack(SqStack *S);               // 先出先打印

State ifStackSky(SqStack *S)
{
    if (S->top == -1)
        return ERROR;
    return OK;
}

// ----------------------- 中序遍历 - 需要理解 -----------------------
/* 非递归的中序遍历
 * 1. 以根节点开始，把整条左边进栈
 * 2. 出栈
 * 3. 处理任务，打印，判断是否在这个任务上，还有新任务
 * 4. 以这个任务的右孩子为新节点，再次执行1
 * 5. 退出栈为空
 * */
void PreOrderTraveres(BiTree T)
{
    // 中序遍历 - 非递归
    SqStack *stack = initStack();
    BiTNobe *node = T;
    while (node || ifStackSky(stack))
    {
        if (node)
        {
            pushStack(&stack, node);
            node = node->lchild;
        }
        else
        {
            popStack(&stack, &node);
            node = node->rchild;
        }
    }
    destroyStack(&stack);
}

State destroyStack(SqStack **S)
{
    free(*S);
    *S = NULL;
    return OK;
}


State initTree(BiTree *T)
{
    *T = NULL;
    return OK;
}

State createBiTree(BiTree *T)
{
    char c;// 问题的没有, 是输入格式和0的不一样
    // 结点为叶结点则2空格
    scanf("%c", &c);
    if (c == ' ')
    {
        *T = NULL;// 不用释放是因为压根就没有分配
    }
    else
    {
        *T = (BiTree)malloc(sizeof(BiTNobe));
        (*T)->data = c;
        createBiTree(&(*T)->lchild);
        createBiTree(&(*T)->rchild);
    }
    return OK;
}



SqStack *initStack(void)
{
    // 初始化操作, 建立一个空栈的S
    SqStack *S = (SqStack *)malloc(sizeof(SqStack));
    S->top = -1;
    return S;
}

State pushStack(SqStack **S, BiTNobe *e)
{
    // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
    if ((*S)->top == MAXSIZE - 1)
    {
        return ERROR;
    }

    (*S)->data[++((*S)->top)] = e;
    return OK;
}

State popStack(SqStack **S, BiTNobe **e)
{
    // 删除栈顶元素, 并用e返回其值 
    if ((*S)->top == -1)
    {
        return ERROR;
    }
    *e = (*S)->data[((*S)->top)--];
    
    /* 执行的操作 */
    printf("%c ", (*e)->data);
    return OK;
}

State printStack(SqStack *S)
{
    // 先出先打印

    for (int i = S->top; i > -1; i--)
    {
        printf("%d ", S->data[i]);
    }

    return OK;
}

int main(void)
{
    // 栈 的 数组(顺序存储结构)表示
    // 二叉树的非递归中序遍历
    BiTree bitree;
    initTree(&bitree);
    printf("请输入:");
    createBiTree(&bitree);
    printf("已接受!\n");
    PreOrderTraveres(bitree);

    getchar();
    getchar();
    return 0;
}
```


## 后序遍历
### 递归
```C
void 后序遍历(BTree *root, BTNode *node) {
    if (!node) {
        return;
    }

    后序遍历(root, node->left);
    后序遍历(root, node->right);
  
    // 需要做的事情
    printf("%d ", node->data);
}
```

### 非递归

```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

#define MAXSIZE 16      // 栈的大小

typedef int State;      // 状态 OK or ERROR
typedef char ElemType;

typedef struct BiTNobe
{
    ElemType data;
    struct BiTNobe *lchild, *rchild;// 左孩子, 右孩子
} BiTNobe, *BiTree;

typedef struct SqStack
{
    BiTNobe *data[MAXSIZE];
    int top;// 用于栈顶的指针
}SqStack;

SqStack *initStack(void);            // 初始化操作, 建立一个空栈的S
State destroyStack(SqStack **S);     // 销毁栈S

State pushStack(SqStack **S, BiTNobe * e);   // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
State popStack(SqStack **S, BiTNobe **e);    // 删除栈顶元素, 并用e返回其值
State ifStackSky(SqStack *S);                // 判断栈是否有内容


void PreOrderTraveres(BiTree T);            // 后序遍历
State initTree(BiTree *T);                  // 初始化树
State createBiTree(BiTree *T);              // 创建一颗二叉树
State printStack(SqStack *S);               // 先出先打印

State ifStackSky(SqStack *S)
{
    if (S->top == -1)
        return ERROR;
    return OK;
}

// ----------------------- 后序遍历 - 知道即可 -----------------------
/* 非递归后序遍历，需要两个栈，第一个栈为辅助栈，第二个栈才是输出栈
 * 第一个栈初始化头结点，弹出到第二个栈，根节点就最后出了
 * 辅助栈要先左后右压，弹出辅助栈的一个元素，放入第二个栈
 * 放入第二个栈的节点 先左后右放入辅助栈
 * 直到辅助栈为空
 * */
void PreOrderTraveres(BiTree T)
{
    // 后序遍历 - 非递归
    SqStack *stack_fuzu = initStack();  // 辅助栈
    SqStack *stack_put = initStack();   // 输出栈
    BiTNobe *node = T;
    pushStack(&stack_fuzu, node);
    while (ifStackSky(stack_fuzu))
    {
        popStack(&stack_fuzu, &node);
        pushStack(&stack_put, node);
        if (node->lchild)
            pushStack(&stack_fuzu, node->lchild);
        if (node->rchild)
            pushStack(&stack_fuzu, node->rchild);
    }

    while (ifStackSky(stack_put))
    {
        popStack(&stack_put, &node);
        printf("%c ", node->data);
    }
    destroyStack(&stack_fuzu);
    destroyStack(&stack_put);
}

State destroyStack(SqStack **S)
{
    free(*S);
    *S = NULL;
    return OK;
}

State initTree(BiTree *T)
{
    *T = NULL;
    return OK;
}

State createBiTree(BiTree *T)
{
    char c;// 问题的没有, 是输入格式和0的不一样
    // 结点为叶结点则2空格
    scanf("%c", &c);
    if (c == ' ')
    {
        *T = NULL;// 不用释放是因为压根就没有分配
    }
    else
    {
        *T = (BiTree)malloc(sizeof(BiTNobe));
        (*T)->data = c;
        createBiTree(&(*T)->lchild);
        createBiTree(&(*T)->rchild);
    }
    return OK;
}



SqStack *initStack(void)
{
    // 初始化操作, 建立一个空栈的S
    SqStack *S = (SqStack *)malloc(sizeof(SqStack));
    S->top = -1;
    return S;
}

State pushStack(SqStack **S, BiTNobe *e)
{
    // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
    if ((*S)->top == MAXSIZE - 1)
    {
        return ERROR;
    }

    (*S)->data[++((*S)->top)] = e;
    return OK;
}

State popStack(SqStack **S, BiTNobe **e)
{
    // 删除栈顶元素, 并用e返回其值 
    if ((*S)->top == -1)
    {
        return ERROR;
    }
    *e = (*S)->data[((*S)->top)--];
    return OK;
}

State printStack(SqStack *S)
{
    // 先出先打印

    for (int i = S->top; i > -1; i--)
    {
        printf("%d ", S->data[i]);
    }

    return OK;
}

int main(void)
{
    // 栈 的 数组(顺序存储结构)表示
    // 二叉树的非递归后序遍历
    BiTree bitree;
    initTree(&bitree);
    printf("请输入:");
    createBiTree(&bitree);
    printf("已接受!\n");
    PreOrderTraveres(bitree);

    getchar();
    getchar();
    return 0;
}
```


## 广度优先 / 层序遍历

```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错

typedef int State;      // 状态 OK or ERROR
typedef char ElemType;

typedef struct BiTNobe
{
    ElemType data;
    struct BiTNobe *lchild, *rchild;        // 左孩子, 右孩子
} BiTNobe, *BiTree;

typedef struct QueueBiTNode
{
    BiTNobe *treeNode;
    struct QueueBiTNode *next;
} QueueBiTNode;

typedef struct QueueBiT
{
    QueueBiTNode *top;  // 队头
    QueueBiTNode *base; // 队尾
    int node_num;
} QueueBiT;

void DFS(BiTree T);                         // 广度优先遍历
State initTree(BiTree *T);                  // 初始化树
State createBiTree(BiTree *T);              // 创建一颗二叉树

QueueBiT *initQueueBiT(void);                   // 申请队列
State addQueueBiTNode(QueueBiT **Q, BiTree T);  // 入队
State delQueueBiTNode(QueueBiT **Q);            // 出队<定制>
_Bool ifQueueBiTNum(QueueBiT *Q);               // 判断队列是否为空
void freeQueueBiT(QueueBiT **queue);            // 释放队列

_Bool ifQueueBiTNum(QueueBiT *Q)
{
    // 返回 1 为空
    // 返回 0 非空
    if (Q->node_num)
        return 0;
    return 1;
}

State delQueueBiTNode(QueueBiT **Q)
{
    // 出队列, 并且将出队列的树的子树按先左后右的形式入队, 并且保证入队的不能又出队
    QueueBiTNode *p = (*Q)->top;
    for (int delNum = (*Q)->node_num; delNum > 0; --delNum)
    {
        if (p->treeNode->lchild)
        {
            addQueueBiTNode(Q, p->treeNode->lchild);
        }
        if (p->treeNode->rchild)
        {
            addQueueBiTNode(Q, p->treeNode->rchild);
        }
        p = p->next;
        --(*Q)->node_num;
    }
    (*Q)->top = p;
    return OK;
}

State addQueueBiTNode(QueueBiT **Q, BiTree T)
{
    QueueBiTNode *p = (QueueBiTNode *)malloc(sizeof(QueueBiTNode));
    if (!p)
        return ERROR;
    p->treeNode = T;

    /* 在这里进行操作 */
    printf("%c ", T->data);

    if (!(*Q)->top)
    {
        p->next = NULL;
        (*Q)->top = p;
        (*Q)->base = p;
        ++(*Q)->node_num;
        return OK;
    }
    
    p->next = (*Q)->base->next;
    (*Q)->base->next = p;
    (*Q)->base = p;
    ++(*Q)->node_num;
    return OK;
}

void freeQueueBiT(QueueBiT **queue)
{
    // 疑问滴有: free(queue) 呢?
    free(*queue);
    *queue = NULL;
}

QueueBiT *initQueueBiT(void)
{
    QueueBiT *queue = (QueueBiT *)malloc(sizeof(QueueBiT));
    queue->base = NULL;
    queue->top = NULL;
    queue->node_num = 0;
    return queue;
}

void DFS(BiTree T)
{
    // 广度优先遍历 / 层序遍历
    QueueBiT *queue = initQueueBiT(); // 申请一个辅助队列
    addQueueBiTNode(&queue ,T);
    while(!ifQueueBiTNum(queue))
    {
        delQueueBiTNode(&queue);
    }
    freeQueueBiT(&queue);
}

State initTree(BiTree *T)
{
    *T = NULL;
    return OK;
}

State createBiTree(BiTree *T)
{
    char c;// 问题的没有, 是输入格式和0的不一样
    // 结点为叶结点则2空格
    scanf("%c", &c);
    if (c == ' ')
    {
        *T = NULL;// 不用释放是因为压根就没有分配
    }
    else
    {
        *T = (BiTree)malloc(sizeof(BiTNobe));
        (*T)->data = c;
        createBiTree(&(*T)->lchild);
        createBiTree(&(*T)->rchild);
    }
    return OK;
}

int main(void)
{
    // 二叉树 - 二叉链表 - BFS遍历 - 自个的 ()

    // 空格代表为空节点
    BiTree bitree;
    initTree(&bitree);
    printf("请输入:");
    createBiTree(&bitree);
    BFS(bitree);

    getchar();
    getchar();
    return 0;
}
```

