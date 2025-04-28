# 邻接矩阵

```C
#include <stdio.h>
#include <stdlib.h>

typedef struct
{
    char **show;
    int **weight;
    int *tagArray;  // 用于遍历时候的标记
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

int main(void)
{
    // 邻接矩阵 - By01 - BFS + DFS
    AdjacencyMatrix *A = initAdjacencyMatrix(6);
    addAdjacencyMatrix(A, "v1");
    addAdjacencyMatrix(A, "v2");
    addAdjacencyMatrix(A, "v3");
    addAdjacencyMatrix(A, "v4");
    addAdjacencyMatrix(A, "v5");
    addAdjacencyMatrix(A, "v6");
    connectAdjacencyMatrix(A, "v1", "v2", 1);
    connectAdjacencyMatrix(A, "v1", "v6", 1);
    connectAdjacencyMatrix(A, "v1", "v4", 1);
    connectAdjacencyMatrix(A, "v2", "v3", 1);
    connectAdjacencyMatrix(A, "v6", "v3", 1);
    connectAdjacencyMatrix(A, "v6", "v2", 1);
    connectAdjacencyMatrix(A, "v6", "v4", 1);
    connectAdjacencyMatrix(A, "v3", "v5", 1);
    printf("DFS: ");
    DFS(A, 0);
    initTagArray(A);
    printf("\nBFS: ");
    BFS(A);
    freeAdjacencyMatrix(A);
    getchar();
    return 0;
}
```

# 邻接表

```C
#include <stdio.h>
#include <stdlib.h>

typedef struct _AdjacencyListNode
{
    int index;
    struct _AdjacencyListNode *next;    // 有权也可再加
} AdjacencyListNode;

typedef struct
{
    char **show;                // data 可以搞个唯一识别码
    AdjacencyListNode **list;

    /* 用于快速插入 */
    int n;
    int add_index;

    /* 用于遍历 */
    _Bool *tag_arr;
} AdjacencyList;

AdjacencyList *initAdjacencyList(int n);                    // 初始化邻接表
void addAdjacencyListNode(AdjacencyList *A, char *show);    // 添加元素
_Bool connectAdjacencyListNode(AdjacencyList *A, char *show_1, char *show_2/*, int 权*/);   // 连接结点
void DFS(AdjacencyList *A, int index);                      // 深度优先遍历
void BFS(AdjacencyList *A, int index);                      // 广度优先遍历
void initTagArr(AdjacencyList *A);
void freeAdjacencyList(AdjacencyList *A);

void initTagArr(AdjacencyList *A)
{
    for (int i = 0; i < A->n; ++i)
    {
        A->tag_arr[i] = 0;
    }
}

AdjacencyList *initAdjacencyList(int n)
{
    AdjacencyList* A = (AdjacencyList *)malloc(sizeof(AdjacencyList));
    if (!A)
    {
        MALLOC_ERROR:
        printf("MALLOC ERROR!\n");
        return NULL;
    }
    A->show = (char **)malloc(sizeof(char *) * n);
    if (!A->show)
        goto MALLOC_ERROR;
    
    A->list = (AdjacencyListNode **)malloc(sizeof(AdjacencyListNode *) * n);
    if (!A->list)
        goto MALLOC_ERROR;
    
    A->tag_arr = (_Bool *)malloc(sizeof(_Bool) * n);
    if (!A->tag_arr)
        goto MALLOC_ERROR;
    A->n = n;
    initTagArr(A);
    for (int i = 0; i < n; ++i)
    {
        A->list[i] = NULL;
    }
    A->add_index = 0;
    return A;
}

void addAdjacencyListNode(AdjacencyList *A, char *show)
{
    if (A->add_index == A->n)
        return; // 满
    
    A->show[A->add_index++] = show;
}

// 定义为 show_1 --> show_2 (有向图)
_Bool connectAdjacencyListNode(AdjacencyList *A, char *show_1, char *show_2/*, int 权*/)
{
    int i_1 = -1;
    for (int i = 0; i < A->add_index; ++i)
    {
        if (A->show[i] == show_1)
        {
            i_1 = i;
            break;
        }
    }

    if (i_1 == -1)
        return 0;   // 没找到
    
    int i_2 = -1;
    for (int i = 0; i < A->add_index; ++i)
    {
        if (A->show[i] == show_2)
        {
            i_2 = i;
            break;
        }
    }

    if (i_2 == -1)
        return 0;   // 没找到
    
    AdjacencyListNode *p = (AdjacencyListNode *)malloc(sizeof(AdjacencyListNode));
    if (!p)
        return 0; // malloc error!
    
    p->index = i_2;
    p->next = A->list[i_1];
    A->list[i_1] = p;
    return 1;
}

void DFS(AdjacencyList *A, int index)
{
    printf("%s ", A->show[index]);
    A->tag_arr[index] = 1;
    AdjacencyListNode *p = A->list[index];
    while (p)
    {
        if (!A->tag_arr[p->index])
            DFS(A, p->index);
        p = p->next;
    }
}

// 只能遍历连通图, 不连通需要再加个for!
void BFS(AdjacencyList *A, int index)
{
    // 临时队列
    int queue[A->add_index + 1];
    int q_h = 0, q_t = 0;
    queue[q_t++] = index;
    A->tag_arr[index] = 1;
    while (q_t != q_h)
    {
        printf("%s ", A->show[queue[q_h]]);
        AdjacencyListNode *p = A->list[queue[q_h]];
        while (p)
        {
            if (!A->tag_arr[p->index])
            {
                queue[q_t] = p->index;
                A->tag_arr[p->index] = 1;
                q_t = (q_t + 1) % (A->add_index + 1);
            }
            p = p->next;
        }
        q_h = (q_h + 1) % (A->add_index + 1);
    }
}

void freeAdjacencyList(AdjacencyList *A)
{
    for (int i = 0; i < A->add_index; ++i)
    {
        AdjacencyListNode *p = A->list[i];
        AdjacencyListNode *tmp = NULL;
        while (p)
        {
            tmp = p;
            p = p->next;
            free(tmp);
        }
    }

    free(A->tag_arr);
    free(A->show);
    free(A->list);
    free(A);
}

int main(void)
{
    // 邻接表 - By01 - BFS + DFS
    AdjacencyList *A = initAdjacencyList(6);
    addAdjacencyListNode(A, "v1");
    addAdjacencyListNode(A, "v2");
    addAdjacencyListNode(A, "v3");
    addAdjacencyListNode(A, "v4");
    addAdjacencyListNode(A, "v5");
    addAdjacencyListNode(A, "v6");
    connectAdjacencyListNode(A, "v1", "v2");
    connectAdjacencyListNode(A, "v2", "v6");
    connectAdjacencyListNode(A, "v2", "v3");
    connectAdjacencyListNode(A, "v3", "v5");
    connectAdjacencyListNode(A, "v3", "v1");
    connectAdjacencyListNode(A, "v3", "v4");

    printf("\nBFS: ");
    BFS(A, 0);
    initTagArr(A);

    printf("\nDFS: ");
    DFS(A, 0);
    initTagArr(A);
    
    freeAdjacencyList(A);
    getchar();
    return 0;
}
```

# 十字链表

```C
#include <stdio.h>
#include <stdlib.h>

/*
 * 链表
 * 出度指向的目标索引 出度指向的目标*next 入度目标索引 入度*next
 * */
typedef struct _G_node
{
    /*出度*/
    int out_index;
    struct _G_node *out_next;
    /*入度*/
    int in_index;
    struct _G_node *in_next;
} OrthogonalListNode;

typedef struct
{
    const char *show;
    int no;             // 编号(理解为索引也可以, 用于判断是否是目标元素)
} OrthogonalListData;

typedef struct
{
    OrthogonalListData *data;
    OrthogonalListNode **out_list;  // 出度链表
    OrthogonalListNode **in_list;   // 入度链表
    int len_max;                    // 最大长度
    int add_index;                  // 当前长度 / 添加元素时候的索引
} OrthogonalList;

OrthogonalList *initOrthogonalList(int n);                      // 初始化
void addOrthogonalListData(OrthogonalList *G, const char *e);   // 添加元素
void connectGraphNode(OrthogonalList *G, int no_1, int no_2);   // 连接结点 no_1 --> no_2
// 遍历省略
void freeOrthogonalList(OrthogonalList  *G);                    // 免费它
int getOutListLen(OrthogonalList *G, int no);                   // 返回元素no出度的链表长度
int getInListLen(OrthogonalList *G, int no);                    // 返回元素no入度的链表长度

OrthogonalList *initOrthogonalList(int n)
{
    OrthogonalList *G = (OrthogonalList *)malloc(sizeof(OrthogonalList));
    if (!G)
    {
        ERROR:
        printf("MALLOC ERROR!\n");
        return NULL;
    }
    G->data = (OrthogonalListData *)malloc(sizeof(OrthogonalListData) * n);
    if (!G->data)
        goto ERROR;
    G->out_list = (OrthogonalListNode **)malloc(sizeof(OrthogonalListNode *) * n);
    if (!G->out_list)
        goto ERROR;
    G->in_list = (OrthogonalListNode **)malloc(sizeof(OrthogonalListNode *) * n);
    if (!G->in_list)
        goto ERROR;
    for (int i = 0; i < n; ++i)
    {
        G->out_list[i] = NULL;
        G->in_list[i] = NULL;
    }
    G->len_max = n;
    G->add_index = 0;
    return G;
}

void addOrthogonalListData(OrthogonalList *G, const char *e)
{
    if (G->add_index == G->len_max)
        return; // error 满了
    G->data[G->add_index].show = e;
    G->data[G->add_index].no = G->add_index;
    ++G->add_index;
}

// 找毛no不是有吗
// static int getIndexNode(OrthogonalList *G, int no)  // 找结点
// {
//     for (int i = 0; i < G->add_index; ++i)
//     {
//         if (G->data[i].no == no)
//             return i;
//     }
//     return -1;
// }

void connectGraphNode(OrthogonalList *G, int no_1, int no_2)
{
    // 连接结点 no_1 --> no_2
    OrthogonalListNode *node = (OrthogonalListNode *)malloc(sizeof(OrthogonalListNode));
    if (!node)
        return; // error malloc

    // 头插法 出度
    node->out_index = no_1;
    node->out_next = G->out_list[no_1];
    G->out_list[no_1] = node;

    // 头插法 入度
    node->in_index = no_2;
    node->in_next = G->in_list[no_2];
    G->in_list[no_2] = node;
}

int getOutListLen(OrthogonalList *G, int no)
{
    int res = 0;
    OrthogonalListNode *p = G->out_list[no];
    while (p)
    {
        ++res;
        p = p->out_next;
    }
    return res;
}

int getInListLen(OrthogonalList *G, int no)
{
    int res = 0;
    OrthogonalListNode *p = G->in_list[no];
    while (p)
    {
        ++res;
        p = p->in_next;
    }
    return res;
}

// 只要释放出度或者入度即可, 因为node是共用的!
void freeOrthogonalList(OrthogonalList  *G)
{
    for (int i = 0; i < G->add_index; ++i)
    {
        OrthogonalListNode *p = G->in_list[i];
        OrthogonalListNode *tmp = NULL;
        while(p)
        {
            tmp = p;
            p = p->in_next;
            free(tmp);
        }
    }

    free(G->data);
    free(G);
}

int main(void)
{
    //  - 十字链表 - 有向图
    /*
    * 索引 元素data 入度 出度
    * */
    OrthogonalList *G = initOrthogonalList(4);
    addOrthogonalListData(G, "v0");
    addOrthogonalListData(G, "v1");
    addOrthogonalListData(G, "v2");
    addOrthogonalListData(G, "v3");
    connectGraphNode(G, 0, 3);
    connectGraphNode(G, 0, 1);
    connectGraphNode(G, 0, 2);
    connectGraphNode(G, 3, 0);
    connectGraphNode(G, 3, 1);
    connectGraphNode(G, 3, 2);
    connectGraphNode(G, 1, 2);
    for (int i = 0; i < 4; ++i)
        printf("v%c 的出度: %d, 入度: %d\n", '0' + i,getOutListLen(G, i), getInListLen(G, i));
    freeOrthogonalList(G);
    getchar();
    return 0;
}
```

# 邻接多重表

```C
#include <stdio.h>
#include <stdlib.h>

/*
 *  链表, 存储的边的关系, 顺序无所谓
 * */
typedef struct _G_node
{
    int i_index;
    int j_index;
    struct _G_node *i_next;
    struct _G_node *j_next;
} AMTNode;

typedef struct
{
    const char *show;
    int no;             // 编号(理解为索引也可以, 用于判断是否是目标元素)
} AMTData;

typedef struct
{
    AMTData *data;
    AMTNode **list;
    int max_len;
    int add_index;
} AMT;

AMT *initAMT(int n);                                // init
void addAMTData(AMT *G, const char *e);             // add e
void connectAMTNode(AMT *G, int no_1, int no_2);    // 连接
int getNodeNum(AMT *G, int no);                     // 获取度
void delAMTSide(AMT *G, int no, int no_2);          // 删除边
void freeAMT(AMT *G);                               // 免费它


AMT *initAMT(int n)
{
    AMT *G = (AMT *)malloc(sizeof(AMT));
    if (!G)
    {
        ERROR:
        printf("error malloc!\n");
        return NULL;
    }

    G->data = (AMTData *)malloc(sizeof(AMTData) * n);
    if (!G->data)
        goto ERROR;
    
    G->list = (AMTNode **)malloc(sizeof(AMTNode *) * n);
    if (!G->list)
        goto ERROR;
    
    for (int i = 0; i < n; ++i)
        G->list[i] = NULL;
    G->add_index = 0;
    G->max_len = n;

    return G;
}

void addAMTData(AMT *G, const char *e)
{
    G->data[G->add_index].show = e;
    G->data[G->add_index].no = G->add_index;
    ++G->add_index;
}

void connectAMTNode(AMT *G, int no_1, int no_2)
{
    AMTNode *node = (AMTNode *)malloc(sizeof(AMTNode));
    node->i_index = no_1;
    node->j_index = no_2;
    node->i_next = G->list[no_1];
    G->list[no_1] = node;
    node->j_next = G->list[no_2];
    G->list[no_2] = node;
}

int getNodeNum(AMT *G, int no)
{
    AMTNode *p = G->list[no];
    int res = 0;
    while (p)
    {
        ++res;
        p = p->i_index == no ? p->i_next : p->j_next;
    }
    return res;
}

void delAMTSide(AMT *G, int no_1, int no_2)
{
    // 先决条件是 no_1 and no_2 存在且正确
    AMTNode *node = G->list[no_1];
    AMTNode *tmp = NULL;
    while (node)
    {
        if (node->i_index == no_2 || node->j_index == no_2)
            break;
        tmp = node;
        node = node->i_index == no_1 ? node->i_next : node->j_next;
    }

    if (!node)
        return; // 没找到

    if (tmp)
        if (tmp->i_index == no_1)
            tmp->i_next = node->i_index == no_1 ? node->i_next : node->j_next;
        else
            tmp->j_next = node->i_index == no_1 ? node->i_next : node->j_next;
    else
        G->list[no_1] = node->i_index == no_1 ? node->i_next : node->j_next;

    node = G->list[no_2];
    tmp = NULL;
    while (node)
    {
        if (node->i_index == no_1 || node->j_index == no_1)
            break;
        tmp = node;
        node = node->i_index == no_2 ? node->i_next : node->j_next;
    }

    if (!node)
        return; // 没找到

    if (tmp)
        if (tmp->i_index == no_2)
            tmp->i_next = node->i_index == no_2 ? node->i_next : node->j_next;
        else
            tmp->j_next = node->i_index == no_2 ? node->i_next : node->j_next;
    else
        G->list[no_2] = node->i_index == no_2 ? node->i_next : node->j_next;
    free(node);
}

void freeAMT(AMT *G)
{
    for (int i = 0; i < G->add_index; ++i)
    {
        for (int j = i + 1; j < G->add_index; ++j)
            delAMTSide(G, i, j);
    }
    free(G->data);
    free(G->list);
    free(G);
}

int main(void)
{
    //  - 邻接多重表 - 无向图!
    AMT *G = initAMT(5);
    addAMTData(G, "A");
    addAMTData(G, "B");
    addAMTData(G, "C");
    addAMTData(G, "D");
    addAMTData(G, "E");
    connectAMTNode(G, 0, 3);
    connectAMTNode(G, 0, 1);
    connectAMTNode(G, 0, 2);
    connectAMTNode(G, 1, 3);
    connectAMTNode(G, 1, 4);
    connectAMTNode(G, 1, 2);
    connectAMTNode(G, 3, 4);

    for (int i = 0; i < G->add_index; ++i)
        printf("%c 的度为 %d\n", 'A' + i, getNodeNum(G, i));
    delAMTSide(G, 2, 1);
    putchar('\n');
    for (int i = 0; i < G->add_index; ++i)
        printf("%c 的度为 %d\n", 'A' + i, getNodeNum(G, i));
    freeAMT(G);
    getchar();
    return 0;
}
```
