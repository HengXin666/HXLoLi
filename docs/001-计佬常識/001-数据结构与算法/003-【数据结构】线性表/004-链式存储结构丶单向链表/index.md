# 链式存储结构-单向链表
## 代码

// wdf, 之前我这么闲的吗? 还写个内存池...(虽然也不难)

```C
#include <stdio.h>
#include <stdlib.h>

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

// --- 内存池 ---
LinkList pool = NULL;
int pool_num = 0;

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

int getElem(LinkList L, int i, int *e)
{
    // 将线性表L第i个元素返回给e
    if (i < 1)
    {
        return ERROR;
    }

    for (int j = 0; j < i; j++)
    {
        if (L != NULL)
        {
            L = L->next;
        }
        else
        {
            return ERROR;
        }
    }

    *e = L->data;
    return OK;
}

int listHAdd(LinkList *L, int e)
{
    // 头插法添加元素
    Nobe *p;
    if (pool_num == 0)
    {
        p = (Nobe *)malloc(sizeof(Nobe));
        if (p == NULL)
        {
            return ERROR;
        }
    }
    else
    {   
        p = pool;
        pool = pool->next;
        pool_num--;
    }

    p->data = e;

    p->next = *L;
    *L = p;
    return OK;
}

int listAdd(LinkList *L, int e)
{   
    // 添加一个元素(尾插法) <包含初始化?>
    Nobe *p;
    Nobe *cache = *L;

    if (pool_num == 0)
    {
        p = (Nobe *)malloc(sizeof(Nobe));
        if (p == NULL)
        {
            return ERROR;
        }
    }
    else
    {   
        p = pool;
        pool = pool->next;
        pool_num--;
    }
    p->data = e;

    if (*L == NULL)
    {
        p->next = *L;
        *L = p;
    }
    else
    {
        while (cache->next != NULL)
        {
            cache = cache->next;
        }
        cache->next = p;
        p->next = NULL;
    }
    return OK;
}

int listDelete(LinkList *L, int i, int *e)
{
    LinkList p = *L;
    LinkList cache = NULL;
    if (i < 1)
    {
        return ERROR;
    }

    for (int j = 1; j < i; j++)
    {
        cache = p;
        p = p->next;
    }

    if (p)
    {
        *e = p->data;
        if (i == 1)
        {
            *L = p->next;
            if (pool_num < MAXPOOL)
            {
                if (pool_num == 0)
                {
                    p->next = NULL;
                }
                else
                {
                    p->next = pool->next;
                }
                pool = p;
                pool_num++;
            }
            else
            {
                free(p);
            }
        }
        else
        {
            cache->next = p->next;
            if (pool_num < MAXPOOL)
            {   
                if (pool_num == 0)
                {
                    p->next = NULL;
                }
                else
                {
                    p->next = pool->next;
                }
                pool = p;
                pool_num++;
            }
            else
            {
                free(p);
            }
        }
        return OK;
    }
    return ERROR;
}

int listPrint(LinkList L)
{   
    int i = 1;
    while (L != NULL)
    {
        printf("%d: %d\n", i++, L->data);
        L = L->next;
    }
    return OK;
}

int main(void)
{
    // 线性表的单链表版本 --- 下面是测试代码 ---
    // 头指针 与 头节点(本项目无设计这个) 与 第一个节点 (笔记见)
    LinkList hp = NULL;
    int e;

    listAdd(&hp, 1);
    listAdd(&hp, 2);
    listAdd(&hp, 3);
    listDelete(&hp, 1, &e);
    listHAdd(&hp, 999);
    printf("删除的p.data == %d\n", e);
    listPrint(hp);

    return 0;
}
```
