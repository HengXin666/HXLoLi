# Kruskal算法
**克鲁斯卡尔算法(Kruskal)** 是一种使用贪婪方法的最小生成树算法。该算法初始将图视为森林，图中的每一个顶点视为一棵单独的树。一棵树只与它的邻接顶点中权值最小且不违反最小生成树属性（不构成环）的树之间建立连边。

从边的角度求网的最小生成树，更适合于求边稀疏的网的最小生成树。

$基本思想$: **按照权值从小到大**的顺序选择`n-1`条**边**，并保证这`n-1`条边**不构成回路**。

具体做法：首先构造一个只含n个顶点的森林，然后依权值从小到大从连通网中选择边加入到森林中，并使森林中不产生回路，直至森林变成一棵树为止。

代码编写的问题：
- 问题一: 对图的所有边按照权值大小进行排序。
- 问题二: 将边添加到最小生成树中时，怎么样判断是否形成了回路。

克鲁斯卡尔算法的具体思路是：

> [!TIP]
> 将所有边按照权值的大小进行升序排序，然后从小到大一一判断，条件为：
>
> - 如果这个边不会与之前选择的所有边组成回路，就可以作为最小生成树的一部分；反之，舍去。
>
> - 直到具有 n 个顶点的连通网筛选出来 n-1 条边为止。筛选出来的边和所有的顶点构成此连通网的最小生成树。

> 判断是否会产生回路的方法为：在初始状态下给每个顶点赋予不同的标记，对于遍历过程的每条边，其都有两个顶点，判断这两个顶点的标记是否一致，如果一致，说明它们本身就处在一棵树中，如果继续连接就会产生回路；如果不一致，说明它们之间还没有任何关系，可以连接。

## 代码
```C++
#include <stdio.h>
#include <stdlib.h>

#define ARR_LEN_MAX 12
typedef int Element;

// 本质是树的线性存储结构
typedef struct
{
    Element *data;      // 数据
    int len;            // 最大长度
    int n;              // 当前元素个数 (可以用作索引)
    int *father_index;  // 父索引
    int *node_size;     // 以当前结点为根的总的结点数, 默认是 1 (独立个体)
} QU;

QU *initQU(void);                           // 初始化并查集
void addQUNode(QU *Q, Element e);           // 在QU中创建孤立的结点
_Bool QUUnion(QU *Q, Element a, Element b); // 将两个元素所在集合合并到一个集合
_Bool QUFind(QU *Q, Element a, Element b);  // 查询 a与b 元素是不是在同一个集合

QU *initQU(void)
{
    QU *Q = (QU *)malloc(sizeof(QU));
    if (Q == NULL)
    {
        printf("ERROR - malloc - QU\n");
        return NULL;
    }

    Q->len = ARR_LEN_MAX;
    Q->n = 0;

    Q->data = (Element *)malloc(sizeof(Element) * ARR_LEN_MAX);
    if (Q->data == NULL)
    {
        printf("ERROR - malloc - data\n");
        return NULL;
    }

    Q->father_index = (int *)malloc(sizeof(int) * ARR_LEN_MAX);
    if (Q->father_index == NULL)
    {
        printf("ERROR - malloc - f_i\n");
        return NULL;
    }

    Q->node_size = (int *)malloc(sizeof(int) * ARR_LEN_MAX);
    if (Q->node_size == NULL)
    {
        printf("ERROR - malloc - n_s\n");
        return NULL;
    }

    return Q;
}

static _Bool addLenQU(QU *Q)
{
    // 加长数组
    Element *data_a = Q->data;
    int *index_a = Q->father_index;
    int *size_a = Q->node_size;

    Q->len += ARR_LEN_MAX;  // 这个可以自己改 (*2什么的)

    Q->data = (Element *)malloc(sizeof(Element) * Q->len);
    if (Q->data == NULL)
    {
        printf("ERROR - malloc - data\n");
        return 1;
    }

    Q->father_index = (int *)malloc(sizeof(int) * Q->len);
    if (Q->father_index == NULL)
    {
        printf("ERROR - malloc - f_i\n");
        return 1;
    }

    Q->node_size = (int *)malloc(sizeof(int) * Q->len);
    if (Q->node_size == NULL)
    {
        printf("ERROR - malloc - n_s\n");
        return 1;
    }

    for (int i = 0; i < Q->n; ++i)
    {
        Q->data[i] = data_a[i];
        Q->father_index[i] = index_a[i];
        Q->node_size[i] = size_a[i];
    }

    free(data_a);
    free(index_a);
    free(size_a);

    return 0;
}

void addQUNode(QU *Q, Element e)
{
    if (Q->n == Q->len)
    {
        // 长度过长, 需要修改
        if (addLenQU(Q))
        {
            printf("ERROR: Add Node\n");
            return;
        }
    }

    Q->data[Q->n] = e;
    Q->father_index[Q->n] = Q->n;
    Q->node_size[Q->n] = 1;
    ++Q->n;
}

#define _LujinYaSuo_    // 删除或者注释这里就可以使用下面 普通算法
#ifndef _LujinYaSuo_
// 普通算法 (size)
static int getQUNodeIndex(QU *Q, Element e)
{
    // 寻找结点e的祖先结点的索引, 如果找不到则返回-1
    for (int i = 0; i < Q->n; ++i)
    {
        if (Q->data[i] == e)
        {
            while (Q->father_index[i] != i)
            {
                i = Q->father_index[i];
            }

            return i;
        }
    }
    return -1;
}
#else
// 路径压缩: 使得树的高度变小, 如果在合并集合的时候, 单独来进行压缩, 显然也是会浪费一些时间的
// 但如果在查找祖先结点的路上, 顺便把结点给存储起来, 等找到了祖先结点,, 再依次将他们的父结点改为祖先结点不就完美了吗
// 那么下次就可以生效了!
// 当然, 路上遇到的结点也是需要存储起来的, 毕竟找祖先的路「一方通行」
// 所以, 可以使用一个东西存储, 因为对顺序没有什么要求, 所以理论上任何东西都可以拿来存储, 栈/队列...
// 这里使用栈来进行演示

// 因为这个栈是专门为了这个函数定制的, 所以可以不用那么标准(?), 这个就是临时栈
// (由于路上会遇到多少个结点我们不知道, 所以使用链式结构存储)

typedef struct _QU_stack_node
{
    int data_index;         // 记录结点的索引就可以了 (我的意思是, 当前的i, 然后是父亲结点i...)
    struct _QU_stack_node *next;
} _QU_stack_node;

typedef struct
{
    _QU_stack_node *top;    // NULL 就是ok了嘛
} _QU_stack;

static _Bool _pushQUS(_QU_stack *S, int index)
{
    _QU_stack_node *node = (_QU_stack_node *)malloc(sizeof(_QU_stack_node));
    if (!node)
    {
        printf("ERROR - malloc - Snode\n");
        return 0;
    }
    node->data_index = index;
    node->next = S->top;
    S->top = node;
    return 1;
}

static _Bool _popQUS(_QU_stack *S, int *index)
{
    if (S->top == NULL)
        return 0;
    _QU_stack_node *tmp = S->top;
    *index = tmp->data_index;
    S->top = tmp->next;
    free(tmp);
    return 1;
}

static int getQUNodeIndex(QU *Q, Element e)
{
    // 创建一个临时栈用于路径压缩
    _QU_stack *S = (_QU_stack *)malloc(sizeof(_QU_stack));
    if (S == NULL)
    {
        printf("ERROR - malloc - S\n");
        return -1;
    }
    S->top = NULL;

    // 寻找结点e的祖先结点的索引, 如果找不到则返回-1
    for (int i = 0; i < Q->n; ++i)
    {
        if (Q->data[i] == e)
        {
            while (Q->father_index[i] != i)
            {
                _pushQUS(S, i);
                i = Q->father_index[i];
            }

            int index;
            while (_popQUS(S, &index))
            {
                Q->father_index[index] = i;
            }

            return i;
        }
    }
    return -1;
}

#endif

_Bool QUFind(QU *Q, Element a, Element b)
{
    int aRoot_index = getQUNodeIndex(Q, a);
    if (aRoot_index == -1)
    {
        printf("没有找到元素 %c 于并查集中!\n", a);
        return 0;
    }

    int bRoot_index = getQUNodeIndex(Q, b);
    if (bRoot_index == -1)
    {
        printf("没有找到元素 %c 于并查集中!\n", b);
        return 0;
    }

    return aRoot_index == bRoot_index;
}

_Bool QUUnion(QU *Q, Element a, Element b)
{
    // 返回值为 0 是执行失败, 反之成功 (1)
    // 合并元素: 选择将 a, b 中 结点数(node_size)最小的合并到结点数大的去, 如果结点数相同则 a-->b (a合并到b去)
    // 1. 找到a, b的索引
    int aRoot_index = getQUNodeIndex(Q, a);
    if (aRoot_index == -1)
    {
        printf("没有找到元素 %c 于并查集中!\n", a);
        return 0;
    }

    int bRoot_index = getQUNodeIndex(Q, b);
    if (bRoot_index == -1)
    {
        printf("没有找到元素 %c 于并查集中!\n", b);
        return 0;
    }

    if (aRoot_index == bRoot_index)
    {
        printf("不能合并一个相同的元素!\n");
        return 0;
    }

    if (Q->node_size[aRoot_index] > Q->father_index[bRoot_index])
    {
        // B --> A
        Q->father_index[bRoot_index] = aRoot_index;
        Q->node_size[aRoot_index] += Q->node_size[bRoot_index];
    }
    else
    {
        // A --> B
        Q->father_index[aRoot_index] = bRoot_index;
        Q->node_size[bRoot_index] += Q->node_size[aRoot_index];
    }
    return 1;
}

typedef struct
{
    char **show;
    int **weight;
    int *tagArray;  // 用于遍历时候的标记
    int side_num;   // 边的个数
    int add_index;
    int number;
} AdjacencyMatrix;

AdjacencyMatrix *initAdjacencyMatrix(int n);                // 初始化AdjacencyMatrix
void addAdjacencyMatrix(AdjacencyMatrix *A, char *show);    // 添加元素
void connectAdjacencyMatrix(AdjacencyMatrix *A, char *show_1, char *show_2, int weight);    // 连接元素
void initTagArray(AdjacencyMatrix *A);                      // 重置遍历的标记数组 (-1)
void DFS(AdjacencyMatrix *A, int index);                    // 深度优先遍历
void BFS(AdjacencyMatrix *A);                               // 广度优先遍历
void freeAdjacencyMatrix(AdjacencyMatrix *A);               // 免费

AdjacencyMatrix *initAdjacencyMatrix(int n)
{
    AdjacencyMatrix *A = (AdjacencyMatrix *)malloc(sizeof(AdjacencyMatrix));
    if (!A)
    {
        MALLOC_ERROR:
        printf("Malloc ERROR!\n");
        return NULL;
    }

    A->show = (char **)malloc(sizeof(char *) * n);
    if (!A->show)
        goto MALLOC_ERROR;
    
    A->weight = (int **)malloc(sizeof(int *) * n);
    if (!A->weight)
        goto MALLOC_ERROR;

    for (int i = 0; i < n; ++i)
    {
        A->weight[i] = (int *)malloc(sizeof(int) * n);
        if (!A->weight[i])
            goto MALLOC_ERROR;
        for (int j = 0; j < n; ++j)
            A->weight[i][j] = 0;        // 这个是标记数_可改, 记0为未连接
    }
    
    A->side_num = 0;
    A->add_index = 0;
    A->number = n;
    A->tagArray = (int *)malloc(sizeof(int) * n);
    if (!A->tagArray)
        goto MALLOC_ERROR;
    initTagArray(A);
    return A;
}

void addAdjacencyMatrix(AdjacencyMatrix *A, char *show)
{
    if (A->add_index == A->number)
        return; // ERROR
    A->show[A->add_index++] = show;
}

void initTagArray(AdjacencyMatrix *A)
{
    for (int i = 0; i < A->number; ++i)
        A->tagArray[i] = -1;
}

void connectAdjacencyMatrix(AdjacencyMatrix *A, char *show_1, char *show_2, int weight)
{
    int s_1 = -1;
    for (int i = 0; i < A->number; ++i)
    {
        if (A->show[i] == show_1)
            s_1 = i;
    }

    if (s_1 == -1)
        return; // 找不到
    
    int s_2 = -1;
    for (int i = 0; i < A->number; ++i)
    {
        if (A->show[i] == show_2)
            s_2 = i;
    }

    if (s_2 == -1)
        return; // 找不到
    
    A->weight[s_1][s_2] = weight;
    A->weight[s_2][s_1] = weight;
    ++A->side_num;
}

// 注意遍历的是连通图
void DFS(AdjacencyMatrix *A, int index)
{
    printf("%s ", A->show[index]);
    A->tagArray[index] = 1;
    for (int i = 0; i < A->add_index; ++i)
    {
        if (A->weight[index][i] != 0 && A->tagArray[i] == -1)
        {
            DFS(A, i);
        }
    }
}

// 依旧是连通图
// 复杂过头了吧...
void BFS(AdjacencyMatrix *A)
{
    // 临时队列
    int queue[A->add_index];
    int q_h = 0;
    int q_t = 0;
    printf("%s ", A->show[0]);
    A->tagArray[0] = 1;
    for (int i = 0; i < A->add_index; ++i)
    {
        for (int j = 0; j < A->add_index; ++j)
        {
            if (A->weight[i][j] != 0 && A->tagArray[j] == -1)
            {
                A->tagArray[j] = 1;
                queue[q_t++] = j;
                q_t = q_t % A->add_index;
            }
        }

        if (q_h != q_t)
            break;
    }
    
    while (q_h != q_t)
    {
        printf("%s ", A->show[queue[q_h]]);
        for (int i = 0; i < A->add_index; ++i)
        {
            if (A->weight[queue[q_h]][i] != 0 && A->tagArray[i] == -1)
            {
                A->tagArray[q_t == 0 ? A->add_index - 1 : q_t - 1] = 1;
                queue[q_t++] = i;
                q_t = q_t % A->add_index;
            }
        }
        ++q_h;
        q_h = q_h % A->add_index;
    }
}

void freeAdjacencyMatrix(AdjacencyMatrix *A)
{
    free(A->show);
    for (int i = 0; i < A->number; ++i)
    {
        free(A->weight[i]);
    }
    free(A->weight);
    free(A->tagArray);
    free(A);
}

typedef struct
{
    int begin;  // 开始端点
    int end;    // 结束端点
    int weight; // 权
} Edge;

typedef struct
{
    int *vertex;    // 顶点集 (G的索引)
    Edge *side;     // 边集
    int side_len;   // 边集长度
    int vertex_len; // 顶点集长度
} EdgeSet;      // 边集数组

// 转化为边集数组
EdgeSet *adjacencyMatrix_to_edgeSet(AdjacencyMatrix *G);

// Kruskal
EdgeSet *kruskal(AdjacencyMatrix *G);   // 给入一个 邻接矩阵, 返回一个最小生成树

// 获取权和
int getWeightSum(const AdjacencyMatrix *G,const EdgeSet *E)
{
    int res = 0;
    printf("最小生成树边为 %d\n", E->side_len);
    for (int i = 0; i < E->side_len; ++i)
    {
        // printf("%d--%d--%d\n", E->side[i].begin, E->side[i].weight, E->side[i].end);
        printf("%s--%d--%s\n", G->show[E->side[i].begin], E->side[i].weight, G->show[E->side[i].end]);
        res += E->side[i].weight;
    }
    return res;
}

// 交换
static void _exchange(int *a, int *b)
{
    *a = *a ^ *b;
    *b = *a ^ *b;
    *a = *a ^ *b;
}

// 冒泡排序
static void _bubble_sort(EdgeSet *eSet)
{
    _Bool tag = 1;
    for (int i = 0; i < eSet->side_len; ++i)
    {
        tag = 1;
        for (int j = i + 1; j < eSet->side_len; ++j)
        {
            if (eSet->side[i].weight > eSet->side[j].weight)
            {
                _exchange(&eSet->side[i].begin, &eSet->side[j].begin);
                _exchange(&eSet->side[i].end, &eSet->side[j].end);
                _exchange(&eSet->side[i].weight, &eSet->side[j].weight);
                tag = 0;
            }
        }

        if (tag)
            return;
    }
}

EdgeSet *kruskal(AdjacencyMatrix *G)
{
    // 转化为边集数组
    EdgeSet *eSet = adjacencyMatrix_to_edgeSet(G);

    // 对边集数组 按权进行排序
    _bubble_sort(eSet);

    // getWeightSum(G, eSet);

    // ---开始kruskal算法---

    // 有 BUG ! 不能只创建一个, 因为一开始他们边不是相通的可能出现 A B 在 unionFind[] 里面, 但是 A-C B-D 连接造成的, 实际上 B-C没有连接
    // 创建 临时并查集, 只需要创建一个, 其他的只要判断是否在这个里面即可
        // 在则已连通(不可添加), 不在则添加
    // int *unionFind = (int *)malloc(sizeof(int) * eSet->vertex_len);
    // int index_UF = 0;
    // 因为存的是结点, 那么最大数量是结点数

    // 并查集
    QU* uf = initQU();
    for (int i = 0; i < eSet->vertex_len; ++i)
        addQUNode(uf, eSet->vertex[i]);

    // 创建 返回的边集数组
    EdgeSet *resES = (EdgeSet *)malloc(sizeof(EdgeSet));
    if (!resES)
    {
        ERROR:
        printf("malloc error!\n");
        return NULL;
    }

    resES->side = (Edge *)malloc(sizeof(Edge) * (eSet->vertex_len - 1));
    if (!resES->side)
        goto ERROR;

    // resES->vertex = (int *)malloc(sizeof(int) * eSet->side_len);
    // if (!resES->vertex)
    //     goto ERROR;

    resES->vertex = eSet->vertex;
    resES->side_len = eSet->vertex_len - 1; // dddd
    resES->vertex_len = eSet->vertex_len;

    // 正式开始
    int side_index = 0;
    for (int i = 0; i < eSet->side_len; ++i)
    {
        // int j = 0;
        // int k = 0;
        // int noAdd = -1;
        // for (; j < index_UF && k < 2; ++j)
        // {
        //     if (unionFind[j] == eSet->side[i].begin)
        //     {
        //         ++k;
        //         noAdd = eSet->side[i].begin;
        //     }
        //     else if (unionFind[j] == eSet->side[i].end)
        //     {
        //         ++k;
        //         noAdd = eSet->side[i].end;
        //     }
        // }

        if (!QUFind(uf, eSet->side[i].begin, eSet->side[i].end))
        {
            resES->side[side_index].begin = eSet->side[i].begin;
            resES->side[side_index].end = eSet->side[i].end;
            resES->side[side_index].weight = eSet->side[i].weight;
            // printf("\n%d == %d\n",  resES->side[side_index].weight, eSet->side[i].weight);
            ++side_index;

            // if (noAdd == -1)
            // {
                // unionFind[index_UF++] = eSet->side[i].begin;
                // unionFind[index_UF++] = eSet->side[i].end;
            QUUnion(uf, eSet->side[i].begin, eSet->side[i].end);
            // }
            // else
            // {
            //     // unionFind[index_UF++] = noAdd != eSet->side[i].begin ? eSet->side[i].begin : eSet->side[i].end;

            // }
        }
        // getWeightSum(G, resES);
    }
    
    // free(unionFind);
    free(eSet->side);
    free(eSet);
    
    return resES;
}

EdgeSet *adjacencyMatrix_to_edgeSet(AdjacencyMatrix *G)
{
    EdgeSet *resES = (EdgeSet *)malloc(sizeof(EdgeSet));
    if (!resES)
        return NULL;

    resES->side = (Edge *)malloc(sizeof(Edge) * G->side_num);
    if (!resES->vertex)
        return NULL;

    resES->vertex = (int *)malloc(sizeof(int) * G->add_index);
    if (!resES->vertex)
        return NULL;

    resES->side_len = G->side_num;
    resES->vertex_len = G->add_index;

    // 构建
    for (int i = 0, k = 0; i < G->add_index; ++i)
    {
        resES->vertex[i] = i;
        for (int j = i + 1; j < G->add_index; ++j)
        {
            if (G->weight[i][j] != 0)   // 定义了 0是无效值
            {
                resES->side[k].begin = i;
                resES->side[k].end = j;
                resES->side[k].weight = G->weight[i][j];
                // printf("%s--%d--%s\n", G->show[i], G->weight[i][j], G->show[j]);
                ++k;
            }
        }
    }
    // printf("--END--\n");
    return resES;
}



void text(void)
{
    AdjacencyMatrix *A = initAdjacencyMatrix(6);
    addAdjacencyMatrix(A, "A");
    addAdjacencyMatrix(A, "B");
    addAdjacencyMatrix(A, "C");
    addAdjacencyMatrix(A, "D");
    addAdjacencyMatrix(A, "E");
    addAdjacencyMatrix(A, "F");
    connectAdjacencyMatrix(A, "E", "F", 5);
    connectAdjacencyMatrix(A, "E", "A", 2);
    connectAdjacencyMatrix(A, "E", "C", 4);
    connectAdjacencyMatrix(A, "E", "B", 10);
    connectAdjacencyMatrix(A, "A", "F", 3);
    connectAdjacencyMatrix(A, "D", "F", 4);
    connectAdjacencyMatrix(A, "D", "C", 1);
    connectAdjacencyMatrix(A, "B", "C", 3);
    connectAdjacencyMatrix(A, "B", "A", 7);

    DFS(A, 0);
    putchar('\n');
    EdgeSet *eSet = kruskal(A);
    
    printf("\n最小生成树的权和是 %d\n", getWeightSum(A, eSet));

    // 释放pass
}

int main(void)
{
    //  - 最小生成树 - Kruskal算法 - 无向有权图
    // 寻找连通图的 权和最小 连通子图, 无环

    /*
    *   Kruskal算法
    *   使用贪婪的思想: 权和最小 -->  将变按权由小到大排序, 依次选择最小.次小.即可
    *                               但是这样盲目选择, 会导致成环, 故有:
    *   
    *   Kruskal算法:    将变按权由小到大排序, 在保证不会成环的情况下, 依次选择最小.次小.
    * 
    *   可行性分析:
    *       1. 如何判断是否成环?
    *           并查集
    *               例:
    *                   选择到边L, 判断该边的端点 A,B,
    *                   如果 A的祖先 == B的祖先, 那么 A与B已经相通, 再添加边L就会成环, 所以Pass
    *                   反之没问题
    *                   // (全部结点最初时是以自己为祖先结点(均不相连))
    * 
    *       2. 既然算法是通过边来判断, 那么如何快速通过边来找到顶点?
    *           使用 边集数组
    *               注: 在图里面, 广泛使用的数据结构是 邻接矩阵 与 邻接表
    *                   因此, 最好可以在算法内部实现转化, 以兼容
    * */
    text();
    getchar();
    return 0;
}
```

# 贪婪算法
## 贪心算法思想
**贪心算法总是作出在当前看来最好的选择**。也就是说贪心算法并**不从整体最优考虑**，它所作出的选择只是在某种意义上的**局部最优选择**。当然，希望贪心算法得到的最终结果也是整体最优的。虽然贪心算法不能对所有问题都得到整体最优解，但对许多问题它能产生整体最优解。如单源最短路经问题，最小生成树问题等。在一些情况下，即使贪心算法不能得到整体最优解，其最终结果却是最优解的很好近似。

贪心算法的基本要素：
1. 贪心选择性质。所谓贪心选择性质是指所求问题的整体最优解可以通过一系列局部最优的选择，即贪心选择来达到。这是贪心算法可行的第一个基本要素，也是贪心算法与动态规划算法的主要区别。

    动态规划算法通常以自底向上的方式解各子问题（后面会讲），而贪心算法则通常以自顶向下的方式进行，以迭代的方式作出相继的贪心选择，每作一次贪心选择就将所求问题简化为规模更小的子问题。

    对于一个具体问题，要确定它是否具有贪心选择性质，必须证明每一步所作的贪心选择最终导致问题的整体最优解。

2. 当一个问题的最优解包含其子问题的最优解时，称此问题具有最优子结构性质。问题的最优子结构性质是该问题可用动态规划算法或贪心算法求解的关键特征。

## 贪心算法的基本思路
从问题的某一个初始解出发逐步逼近给定的目标，以尽可能快的地求得更好的解。当达到算法中的某一步不能再继续前进时，算法停止。

该算法存在问题：
1. 不能保证求得的最后解是最佳的；
2. 不能用来求最大或最小解问题;
3. 只能求满足某些约束条件的可行解的范围。

实现该算法的过程：

```C 伪代码
从问题的某一初始解出发；

while 能朝给定总目标前进一步 do
    求出可行解的一个解元素；

由所有解元素组合成问题的一个可行解。
```
