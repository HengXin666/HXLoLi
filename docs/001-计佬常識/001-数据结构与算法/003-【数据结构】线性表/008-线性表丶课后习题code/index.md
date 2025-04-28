# 线性表-课后习题
## 约瑟夫问题

```C
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define MAXPOOL 1024
#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

// --- 线性表 封装 --- <链表>
typedef int ElemType;

typedef struct Nobe{
    ElemType data;// 密码
    int number;   // 编号
    struct Nobe* next;
} Nobe;
typedef Nobe* LinkList;

int getElem(LinkList L, int i, int *e);         // 将线性表L第i个元素返回给e
int listEmpty(LinkList L);                      // 判断是否为空表 1是0否
void clearList(LinkList *L);                    // 将线性表清空
int locateElem(LinkList L, int e);              // 查找线性表L中是否有值为e的元素, 有则返回其在表中的序号, 否0
int listInsert(LinkList *L, int i, int e);      // 在线性表L中第i个位置插入新元素e
int listDelete(LinkList *L, int i, int *e);     // 删除线性表L中第i个位置的元素, 并返回其值给e

int listHAdd(LinkList *L, int e);               // 头插元素
int listAdd(LinkList *L, int e);                // 在线性表尾部添加元素
int listPrint(LinkList L);                      // 打印单链表
int listLength(LinkList L);                     // 返回线性表的长度

void listJosephusProblem(LinkList *L, int interval, int len, int remaining);    // 约瑟夫问题
void listJosephusProblemAssignment(LinkList *L, int len, int remaining);        // 约瑟夫问题的课后作业
int listJPAHAdd(LinkList *L, int e, int number);// 约瑟夫问题课后作业专用函数(头插法)

int listJPAHAdd(LinkList *L, int e, int number)
{
    // 头插法
    LinkList p;
    p = (LinkList)malloc(sizeof(Nobe));
    if (p == NULL)
    {
        printf("内存申请失败!\n");
        return ERROR;
    }

    p->data = e;
    p->number = number;
    if (*L == NULL)
    {
        *L = p;
        p->next = p;
    }
    else
    {
        LinkList cache = (*L)->next;
        while (cache->next != *L)
        {
            cache = cache->next;
        }
        
        p->next = *L;
        *L = p;
        cache->next = p; 
    }
    return OK;
}

int listHAdd(LinkList *L, int e)
{
    // 头插法
    LinkList p;
    p = (LinkList)malloc(sizeof(Nobe));
    if (p == NULL)
    {
        printf("内存申请失败!\n");
        return ERROR;
    }

    p->data = e;
    if (*L == NULL)
    {
        *L = p;
        p->next = p;
    }
    else
    {
        LinkList cache = (*L)->next;
        while (cache->next != *L)
        {
            cache = cache->next;
        }
        
        p->next = *L;
        *L = p;
        cache->next = p; 
    }
    return OK;
}

int listPrint(LinkList L)
{
    // 打印单链表
    LinkList cache = L;
    if (L != NULL)
    {
        printf("%d ", cache->data);
        while (cache->next != L)
        {
            cache = cache->next;
            printf("%d ", cache->data);
        }
    }
    putchar('\n');
    return OK;
}

int listJPAPrint(LinkList L)
{
    // 打印单链表 (课后作业定制)
    LinkList cache = L;
    if (L != NULL)
    {
        printf("[%d] {%d}\n", cache->number, cache->data);
        while (cache->next != L)
        {
            cache = cache->next;
            printf("[%d] {%d}\n", cache->number, cache->data);
        }
    }
    putchar('\n');
    return OK;
}

int listInsert(LinkList *L, int i, int e)
{
    // 在线性表L中第i个位置插入新元素e
    if (i < 0)
    {
        return ERROR;
    }

    LinkList p;
    p = (LinkList)malloc(sizeof(Nobe));
    if (p == NULL)
    {
        printf("内存申请失败!\n");
        return ERROR;
    }

    p->data = e;
    if (i == 1)
    {
        LinkList cache = (*L)->next;
        while (cache->next != *L)
        {
            cache = cache->next;
        }
        
        p->next = *L;
        *L = p;
        cache->next = p; 
    }
    else
    {
        LinkList cache = *L;
        for (int j = 2; j < i && cache->next != *L; j++)
        {
            cache = cache->next;
        }// 前一个

        p->next = cache->next;
        cache->next = p;
    }

    return OK;
}

int listDelete(LinkList *L, int i, int *e)
{
    // 删除线性表L中第i个位置的元素, 并返回其值给e
    if (i < 1)
    {
        return ERROR;
    }

    LinkList cache = NULL;
    LinkList free_nobe = NULL;
    if (i == 1)
    {
        free_nobe = *L;
        cache = free_nobe->next;
        *e = free_nobe->data;
        if (cache == *L)
        {
            *L = NULL;
        }
        else
        {
            while (cache->next != free_nobe)
            {
                cache = cache->next;
            }
            cache->next = free_nobe->next;
            *L = free_nobe->next;
        }
        free(free_nobe);
    }
    else
    {
        cache = *L;
        int j = 2;
        for (; j < i && cache->next != *L; j++)
        {
            cache = cache->next;
        }// 前一个

        if (cache->next != *L)
        {
            *e = cache->next->data;
            free_nobe = cache->next;
            cache->next = cache->next->next;
            free(free_nobe);
        }
       else
       {
            return ERROR;
       }
    }
    return OK;
}

void listJosephusProblem(LinkList *L, int interval, int len, int remaining)
{
    /*
    功能: 约瑟夫问题
    参数:   循环链表的二级指针 - L
            跨度 - interval (间隔多少杀一个)
            长度 - len
            余下 - remaining
    */
    int i = len, j;
    LinkList p_mae = NULL;
    LinkList p_free = NULL;
    do
    {
        listHAdd(L, i);
    }while (i --> 1);// 生成链表
    
    LinkList p = *L;
    i = len;
    interval--;
    do// 淘汰赛
    {
        for (j = interval; j > 0; --j)
        {
            p_mae = p;
            p = p->next;
        }

        p_free = p;
        p = p->next;
        p_mae->next = p;
        if (p_free == *L)
        {
            *L = p;
        }
        printf("删除了 %d\n", p_free->data);
        free(p_free);
    }while (--i > remaining);
}

void listJosephusProblemAssignment(LinkList *L, int len, int remaining)
{
    /*
    详情: 提高挑战难度:编号为1～N的N个人按顺时针方向围坐一圈，
    每人持有一个密码（正整数，可以自由输入），
    开始人选一个正整数作为报数上限值M，从第一个人按顺时针方向自1开始顺序报数，报道M时停止报数。
    报M的人出列，将他的密码作为新的M值，从他顺时针方向上的下一个人开始从1报数,如此下去，直至所有人全部出列为止。

    功能: 约瑟夫问题的课后作业
    参数:   循环链表的二级指针 - L
            长度 - len
            余下人数 - remaining
    */
    int i = len, j;
    LinkList p_mae = NULL;
    LinkList p_free = NULL;
    time_t t;
    srand((unsigned)time(&t));// 使用当前时间值初始化伪随机数种子序列
    do
    {
        listJPAHAdd(L, rand() % len * 2 + 1, i);// 每人持有一个密码（正整数，可以自由输入）
    }while (i --> 1);// 生成链表

     LinkList p = *L;
    i = len;
    int interval = p->data;
    printf("第一个人的密码是{%d} | 当前len = %d\n", interval, i);
    do// 淘汰赛
    {
        interval = interval % i;
        for (j = interval; j > 0; --j)
        {
            p_mae = p;
            p = p->next;
        }

        p_free = p;
        p = p->next;
        p_mae->next = p;
        if (p_free == *L)
        {
            *L = p;
        }
        printf("位置 += %d の %d, 手上的密码是 %d | 当前len = %d\n", interval, p_free->number ,p_free->data, --i);
        interval = p_free->data;
        free(p_free);
    }while (i > remaining);
}

int main(void)
{
    // 循环(单)链表 (无头结点版) + 约瑟夫问题 + 约瑟夫问题的课后作业
    LinkList head = NULL;

    // listJosephusProblem(&head, 3, 41, 2);
    listJosephusProblemAssignment(&head, 41, 2);
    listJPAPrint(head);

    return 0;
}
```

## 魔术师发牌

```C
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define MAXPOOL 1024
#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

// --- 线性表 封装 --- <链表>
typedef int ElemType;

typedef struct Nobe{
    ElemType data;
    struct Nobe* next;
} Nobe;
typedef Nobe* LinkList;

int getElem(LinkList L, int i, int *e);         // 将线性表L第i个元素返回给e
int listEmpty(LinkList L);                      // 判断是否为空表 1是0否
void clearList(LinkList *L);                    // 将线性表清空
int locateElem(LinkList L, int e);              // 查找线性表L中是否有值为e的元素, 有则返回其在表中的序号, 否0
int listInsert(LinkList *L, int i, int e);      // 在线性表L中第i个位置插入新元素e
int listDelete(LinkList *L, int i, int *e);     // 删除线性表L中第i个位置的元素, 并返回其值给e

int listHAdd(LinkList *L, int e);               // 头插元素
int listAdd(LinkList *L, int e);                // 在线性表尾部添加元素
int listPrint(LinkList L);                      // 打印单链表
int listLength(LinkList L);                     // 返回线性表的长度

void magicDealingProblem(LinkList *L);          // 魔术师发牌问题

int listHAdd(LinkList *L, int e)
{
    // 头插法
    LinkList p;
    p = (LinkList)malloc(sizeof(Nobe));
    if (p == NULL)
    {
        printf("内存申请失败!\n");
        return ERROR;
    }

    p->data = e;
    if (*L == NULL)
    {
        *L = p;
        p->next = p;
    }
    else
    {
        LinkList cache = (*L)->next;
        while (cache->next != *L)
        {
            cache = cache->next;
        }
        
        p->next = *L;
        *L = p;
        cache->next = p; 
    }
    return OK;
}

int listPrint(LinkList L)
{
    // 打印单链表
    LinkList cache = L;
    if (L != NULL)
    {
        printf("%d ", cache->data);
        while (cache->next != L)
        {
            cache = cache->next;
            printf("%d ", cache->data);
        }
    }
    putchar('\n');
    return OK;
}


int listInsert(LinkList *L, int i, int e)
{
    // 在线性表L中第i个位置插入新元素e
    if (i < 0)
    {
        return ERROR;
    }

    LinkList p;
    p = (LinkList)malloc(sizeof(Nobe));
    if (p == NULL)
    {
        printf("内存申请失败!\n");
        return ERROR;
    }

    p->data = e;
    if (i == 1)
    {
        LinkList cache = (*L)->next;
        while (cache->next != *L)
        {
            cache = cache->next;
        }
        
        p->next = *L;
        *L = p;
        cache->next = p; 
    }
    else
    {
        LinkList cache = *L;
        for (int j = 2; j < i && cache->next != *L; j++)
        {
            cache = cache->next;
        }// 前一个

        p->next = cache->next;
        cache->next = p;
    }

    return OK;
}

int listDelete(LinkList *L, int i, int *e)
{
    // 删除线性表L中第i个位置的元素, 并返回其值给e
    if (i < 1)
    {
        return ERROR;
    }

    LinkList cache = NULL;
    LinkList free_nobe = NULL;
    if (i == 1)
    {
        free_nobe = *L;
        cache = free_nobe->next;
        *e = free_nobe->data;
        if (cache == *L)
        {
            *L = NULL;
        }
        else
        {
            while (cache->next != free_nobe)
            {
                cache = cache->next;
            }
            cache->next = free_nobe->next;
            *L = free_nobe->next;
        }
        free(free_nobe);
    }
    else
    {
        cache = *L;
        int j = 2;
        for (; j < i && cache->next != *L; j++)
        {
            cache = cache->next;
        }// 前一个

        if (cache->next != *L)
        {
            *e = cache->next->data;
            free_nobe = cache->next;
            cache->next = cache->next->next;
            free(free_nobe);
        }
       else
       {
            return ERROR;
       }
    }
    return OK;
}

void magicDealingProblem(LinkList *L)
{
    // 魔术师发牌问题
    int i = 13, j;
    do
    {
        listHAdd(L, 0);
    }while (i --> 1);// 生成链表
    LinkList p = *L;

    i = 1;
    A:
    j = i;
    while (j > 0)
    {
        if (j == 1 && p->data == 0)
        {
            p->data = i++;
            --j;
        }
        else if (p->data != 0)
        {

        }
        else
        {
            --j;
        }
        p = p->next;
    }

    if (i < 14)
    {
        goto A;
    }
}

int main(void)
{
    // 循环(单)链表 (无头结点版) + 魔术师发牌问题
    LinkList head = NULL;

    magicDealingProblem(&head);
    listPrint(head);

    getchar();
    return 0;
}
```

## 拉丁方阵

```C
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define MAXPOOL 1024
#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

// --- 线性表 封装 --- <链表>
typedef int ElemType;

typedef struct Nobe{
    ElemType data;
    struct Nobe* next;
} Nobe;
typedef Nobe* LinkList;

int getElem(LinkList L, int i, int *e);         // 将线性表L第i个元素返回给e
int listEmpty(LinkList L);                      // 判断是否为空表 1是0否
void clearList(LinkList *L);                    // 将线性表清空
int locateElem(LinkList L, int e);              // 查找线性表L中是否有值为e的元素, 有则返回其在表中的序号, 否0
int listInsert(LinkList *L, int i, int e);      // 在线性表L中第i个位置插入新元素e
int listDelete(LinkList *L, int i, int *e);     // 删除线性表L中第i个位置的元素, 并返回其值给e

int listHAdd(LinkList *L, int e);               // 头插元素
int listAdd(LinkList *L, int e);                // 在线性表尾部添加元素
int listPrint(LinkList L);                      // 打印单链表
int listLength(LinkList L);                     // 返回线性表的长度

int listFree(LinkList *L);                      // 释放线性表
void latinSquare(LinkList *L, int n);           // 拉丁方阵问题

int listHAdd(LinkList *L, int e)
{
    // 头插法
    LinkList p;
    p = (LinkList)malloc(sizeof(Nobe));
    if (p == NULL)
    {
        printf("内存申请失败!\n");
        return ERROR;
    }

    p->data = e;
    if (*L == NULL)
    {
        *L = p;
        p->next = p;
    }
    else
    {
        LinkList cache = (*L)->next;
        while (cache->next != *L)
        {
            cache = cache->next;
        }
        
        p->next = *L;
        *L = p;
        cache->next = p; 
    }
    return OK;
}

int listPrint(LinkList L)
{
    // 打印单链表
    LinkList cache = L;
    if (L != NULL)
    {
        printf("%d ", cache->data);
        while (cache->next != L)
        {
            cache = cache->next;
            printf("%d ", cache->data);
        }
    }
    putchar('\n');
    return OK;
}


int listInsert(LinkList *L, int i, int e)
{
    // 在线性表L中第i个位置插入新元素e
    if (i < 0)
    {
        return ERROR;
    }

    LinkList p;
    p = (LinkList)malloc(sizeof(Nobe));
    if (p == NULL)
    {
        printf("内存申请失败!\n");
        return ERROR;
    }

    p->data = e;
    if (i == 1)
    {
        LinkList cache = (*L)->next;
        while (cache->next != *L)
        {
            cache = cache->next;
        }
        
        p->next = *L;
        *L = p;
        cache->next = p; 
    }
    else
    {
        LinkList cache = *L;
        for (int j = 2; j < i && cache->next != *L; j++)
        {
            cache = cache->next;
        }// 前一个

        p->next = cache->next;
        cache->next = p;
    }

    return OK;
}

int listDelete(LinkList *L, int i, int *e)
{
    // 删除线性表L中第i个位置的元素, 并返回其值给e
    if (i < 1)
    {
        return ERROR;
    }

    LinkList cache = NULL;
    LinkList free_nobe = NULL;
    if (i == 1)
    {
        free_nobe = *L;
        cache = free_nobe->next;
        *e = free_nobe->data;
        if (cache == *L)
        {
            *L = NULL;
        }
        else
        {
            while (cache->next != free_nobe)
            {
                cache = cache->next;
            }
            cache->next = free_nobe->next;
            *L = free_nobe->next;
        }
        free(free_nobe);
    }
    else
    {
        cache = *L;
        int j = 2;
        for (; j < i && cache->next != *L; j++)
        {
            cache = cache->next;
        }// 前一个

        if (cache->next != *L)
        {
            *e = cache->next->data;
            free_nobe = cache->next;
            cache->next = cache->next->next;
            free(free_nobe);
        }
       else
       {
            return ERROR;
       }
    }
    return OK;
}

void latinSquare(LinkList *L, int n)
{
    int i, j, k;
    for (i = 1; i <= n; i++)
    {
        listHAdd(L, i);
    }
    LinkList p = *L;
    for (i = 1; i <= n; i++)
    {
        for (k = 0; k < i; k++)
        {
            p = p->next;
        }

        for (j = 0; j < n; j++)
        {
            printf("%d ", p->data);
            p = p->next;
        }
        p = *L;
        putchar('\n');
    }
}

int listFree(LinkList *L)
{
    // 释放线性表
    if (*L == NULL)
    {
        return ERROR;
    }

    LinkList p = *L;
    LinkList p_free = NULL;

    do
    {
        p_free = p;
        p = p->next;
        free(p_free);
    }while (p != *L);

    *L = NULL;
    return OK;
}

int main(void)
{
    // 循环(单)链表 (无头结点版) + 拉丁方阵问题 + 释放线性表
    LinkList head = NULL;

    int n = 0;
    printf("请输入拉丁方阵的边长:");
    scanf("%d", &n);
    getchar();
    latinSquare(&head, n);
    listFree(&head);

    return 0;
}
```

## 维吉尼亚加密
### 课堂练习
```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

typedef char ElemType;
typedef struct DualNobe
{
    ElemType data;
    struct DualNobe *prior; // 前驱结点
    struct DualNobe *next;  // 后继结点
} DualNobe, *DuLinkList;

int initList(DuLinkList *L);                      // 初始化操作, 建立一个空的线性表 和 头节点
int listInsert(DuLinkList *L, int i, int e);      // 在线性表L中第i个位置插入新元素e
int listDelete(DuLinkList *L, int i, int *e);     // 删除线性表L中第i个位置的元素, 并返回其值给e

int listFree(DuLinkList *L);                      // 释放线性表
int listAdd(DuLinkList *L, int e);                // 在线性表尾部添加元素
int listPrintE(DuLinkList L, int e);              // 按照偏移量e打印单链表

int initList(DuLinkList *L)
{
    // 初始化空结构体指针为只有头节点的双向循环链表
    if (!*L)
    {
        DuLinkList p;
        p = (DuLinkList)malloc(sizeof(DualNobe));
        if (!p)
        {
            return ERROR;
        }
        *L = p;
        p->next = p;
        p->prior = p;
        return OK;
    }
    else
    {
        return ERROR;
    }
}

int listAdd(DuLinkList *L, int e)
{
    // 尾插法 插入
    if (!*L)
    {
        return ERROR;
    }

    DuLinkList p, cecha = (*L)->prior;
    p = (DuLinkList)malloc(sizeof(DualNobe));
    if (!p)
    {
        return ERROR;
    }
    p->data = e;

    cecha->next = p;
    p->next = *L;
    (*L)->prior = p;
    p->prior = cecha;
}

int listPrintE(DuLinkList L, int e)
{
    // 打印单链表
    e = e % 26;

    DuLinkList p = L;

    if (e >= 0)
    {
        while (e--)
        {
            p = p->prior;
        }   
    }
    else
    {
        --e;
        while (e++)
        {
            p = p->next;
        } 
    }

    L = p;

    do
    {
        if (p->data == '\r')
        {
            p = p->next;
            continue;
        }
        printf("%c ", p->data);
        p = p->next;
    }while (p != L);
    putchar('\n');
    return OK;
}

int listInsert(DuLinkList *L, int i, int e)
{
    // 在线性表L中第i个位置插入新元素e, 如果i大于链表长度,则只在最后的位置插入
    if (i < 1)
    {
        return ERROR;
    }

    DuLinkList p, cecha = *L;
    p = (DuLinkList)malloc(sizeof(DualNobe));
    if (!p)
    {
        return ERROR;
    }

    p->data = e;
    int j = 1;
    while (cecha->next != *L && j < i)
    {
        cecha = cecha->next;
        ++j;
    }// 寻找插入位置的前驱

    cecha->next->prior = p;
    p->next = cecha->next;
    cecha->next = p;
    p->prior = cecha;
    
    return OK;
}

int listDelete(DuLinkList *L, int i, int *e)
{
    // 删除线性表L中第i个位置的元素, 并返回其值给e
    if (i < 1)
    {
        return ERROR;
    }

    DuLinkList cecha = *L, p_free = NULL;

    int j = 1;
    while (cecha->next != *L && j < i)
    {
        cecha = cecha->next;
        ++j;
    }// 寻找删除位置的前驱

    if (cecha->next == *L)
    {
        return ERROR;
    }

    *e = cecha->next->data;
    p_free = cecha->next;
    cecha->next = cecha->next->next;
    cecha->next->next->prior = cecha;
    free(p_free);

    return OK;
}

int listFree(DuLinkList *L)
{
    // 释放线性表
    DuLinkList cecha = (*L)->next, p_free = NULL;

    while (cecha != *L)
    {
        p_free = cecha;
        cecha = cecha->next;
        free(p_free);
    }
    
    p_free = cecha;
    *L = NULL;
    free(p_free);

    return OK;
}

int main(void)
{
    // 双向循环链表 + 带头节点 + 课堂练习
    DuLinkList head = NULL;
    int e;
    initList(&head);
    for (int i = 'A'; i <= 'Z'; i++)
    {
        listAdd(&head, i);
    }
    printf("请输入一个偏移量:");
    scanf("%d", &e);
    getchar();

    listPrintE(head, e);
    listFree(&head);
    getchar();
    return 0;
}
```
### 课后练习

```C
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

clock_t t;

typedef char ElemType;
typedef struct DualNobe
{
    ElemType data;
    struct DualNobe *prior; // 前驱结点
    struct DualNobe *next;  // 后继结点
} DualNobe, *DuLinkList;

int initList(DuLinkList *L);                      // 初始化操作, 建立一个空的线性表 和 头节点
int listInsert(DuLinkList *L, int i, int e);      // 在线性表L中第i个位置插入新元素e
int listDelete(DuLinkList *L, int i, int *e);     // 删除线性表L中第i个位置的元素, 并返回其值给e

int listFree(DuLinkList *L);                      // 释放线性表
int listAdd(DuLinkList *L, int e);                // 在线性表尾部添加元素
int listPrintV(DuLinkList L, char d);              // 按照偏移量e打印单链表

int initList(DuLinkList *L)
{
    // 初始化空结构体指针为只有头节点的双向循环链表
    if (!*L)
    {
        DuLinkList p;
        p = (DuLinkList)malloc(sizeof(DualNobe));
        if (!p)
        {
            return ERROR;
        }
        *L = p;
        p->next = p;
        p->prior = p;
        return OK;
    }
    else
    {
        return ERROR;
    }
}

int listAdd(DuLinkList *L, int e)
{
    // 尾插法 插入
    if (!*L)
    {
        return ERROR;
    }

    DuLinkList p, cecha = (*L)->prior;
    p = (DuLinkList)malloc(sizeof(DualNobe));
    if (!p)
    {
        return ERROR;
    }
    p->data = e;

    cecha->next = p;
    p->next = *L;
    (*L)->prior = p;
    p->prior = cecha;
}

int listPrintV(DuLinkList L, char d)
{
    // 打印单链表
    DuLinkList p = L;
    for (int x = d - 'A'; x; --x)
    {
        p = p->next;
    }

    t = clock();
    while (1)
    {
        if (clock() - t > 100)
        {
            t = clock();
            break;
        }
        continue;
    }
    srand((unsigned)t);
    int e = rand() % 100;
    printf("%c\t%d\t", d, e);
    e = e % 26;
    
    if (e >= 0)
    {
        while (e--)
        {
            p = p->prior;
        }   
    }
    else
    {
        --e;
        while (e++)
        {
            p = p->next;
        } 
    }

    L = p;

    do
    {
        if (p->data == '\r')
        {
            p = p->next;
            continue;
        }
        putchar(p->data);
        p = p->next;
        break;
    }while (p != L);
    putchar('\n');
    return OK;
}

int listInsert(DuLinkList *L, int i, int e)
{
    // 在线性表L中第i个位置插入新元素e, 如果i大于链表长度,则只在最后的位置插入
    if (i < 1)
    {
        return ERROR;
    }

    DuLinkList p, cecha = *L;
    p = (DuLinkList)malloc(sizeof(DualNobe));
    if (!p)
    {
        return ERROR;
    }

    p->data = e;
    int j = 1;
    while (cecha->next != *L && j < i)
    {
        cecha = cecha->next;
        ++j;
    }// 寻找插入位置的前驱

    cecha->next->prior = p;
    p->next = cecha->next;
    cecha->next = p;
    p->prior = cecha;
    
    return OK;
}

int listDelete(DuLinkList *L, int i, int *e)
{
    // 删除线性表L中第i个位置的元素, 并返回其值给e
    if (i < 1)
    {
        return ERROR;
    }

    DuLinkList cecha = *L, p_free = NULL;

    int j = 1;
    while (cecha->next != *L && j < i)
    {
        cecha = cecha->next;
        ++j;
    }// 寻找删除位置的前驱

    if (cecha->next == *L)
    {
        return ERROR;
    }

    *e = cecha->next->data;
    p_free = cecha->next;
    cecha->next = cecha->next->next;
    cecha->next->next->prior = cecha;
    free(p_free);

    return OK;
}

int listFree(DuLinkList *L)
{
    // 释放线性表
    DuLinkList cecha = (*L)->next, p_free = NULL;

    while (cecha != *L)
    {
        p_free = cecha;
        cecha = cecha->next;
        free(p_free);
    }
    
    p_free = cecha;
    *L = NULL;
    free(p_free);

    return OK;
}

int main(void)
{
    // 双向循环链表 + 带头节点 + 课后练习 维吉尼亚加密
    DuLinkList head = NULL;
    char str[32];
    int e = 0;
    initList(&head);
    for (int i = 'A'; i <= 'Z'; i++)// 生成
    {
        listAdd(&head, i);
    }
    printf("请输入需要加密的内容(A-Z):");
    scanf("%s", str);
    getchar();
    
    printf("原\t位移\t加密结果\n");
    while (str[e] != '\0')
    {
        listPrintV(head, str[e++]);
    }
    listFree(&head);

    return 0;
}
```
