# 链式存储结构-循环链表
## 代码实现

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
    if (i < 0)
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

int main(void)
{
    // 循环(单)链表 (无头结点版)
    LinkList head = NULL;
    int e;
    listHAdd(&head, 1);
    listHAdd(&head, 4);
    listHAdd(&head, 3);
    listHAdd(&head, 3);
    listHAdd(&head, 2);
    listHAdd(&head, 2);
    listHAdd(&head, 3);
    listInsert(&head, 2, 666);
    listInsert(&head, 1, 314);
    listDelete(&head, 9, &e);
    printf("删除了%d\n", e);
    listPrint(head);

    return 0;
}
```
