# 链式存储结构-双向循环链表
## 代码实现

```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

typedef int ElemType;
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
int listPrint(DuLinkList L);                      // 打印单链表

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

int listPrint(DuLinkList L)
{
    // 打印单链表
    DuLinkList p = L->next;
    while (p != L)
    {
        printf("%d ", p->data);
        p = p->next;
    }
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
    // 双向循环链表 + 带头节点
    DuLinkList head = NULL;
    int e;
    initList(&head);
    listAdd(&head, 1);
    listAdd(&head, 2);
    listAdd(&head, 3);
    listAdd(&head, 4);
    listInsert(&head, 1, 666);
    listDelete(&head, 5, &e);
    printf("删除 %d\n", e);
    listPrint(head);
    listFree(&head);

    return 0;
}
```
