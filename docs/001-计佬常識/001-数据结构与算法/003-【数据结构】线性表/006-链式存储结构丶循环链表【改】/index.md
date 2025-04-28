# 链式存储结构-循环链表(改)
## 代码实现

太懒了, 释放就不说, 删除都不写...

```C
#include <stdio.h>
#include <stdlib.h>

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

int initList(LinkList *L);                      // 初始化操作, 建立一个空的线性表 和 头节点
int getElem(LinkList L, int i, int *e);         // 将线性表L第i个元素返回给e
int listEmpty(LinkList L);                      // 判断是否为空表 1是0否
void clearList(LinkList *L);                    // 将线性表清空
int locateElem(LinkList L, int e);              // 查找线性表L中是否有值为e的元素, 有则返回其在表中的序号, 否0
int listInsert(LinkList *L, int i, int e);      // 在线性表L中第i个位置插入新元素e
int listDelete(LinkList *L, int i, int *e);     // 删除线性表L中第i个位置的元素, 并返回其值给e

int listMontage(LinkList *L_main, LinkList *L_free);    // 拼接链表
int listHAdd(LinkList *L, int e);               // 头插元素
int listAdd(LinkList *L, int e);                // 在线性表尾部添加元素
int listPrint(LinkList L);                      // 打印单链表
int listLength(LinkList L);                     // 返回线性表的长度

int initList(LinkList *L)
{
    if (*L == NULL)
    {
        LinkList h;
        h = (LinkList)malloc(sizeof(Nobe));// 头节点
        h->next = h;
        *L = h;
        return OK;
    }
    else
    {
        return ERROR;
    }
}

int listPrint(LinkList L)
{
    // 打印单链表
    LinkList p = L->next->next;

    while (p != L->next)
    {
        printf("%d ", p->data);
        p = p->next;
    }
    putchar('\n');

    return OK;
}

int listAdd(LinkList *L, int e)
{
    // 在线性表尾部添加元素
    if (*L == NULL)
    {
        return ERROR;
    }
    LinkList p;
    LinkList cache = NULL;
    p = (LinkList)malloc(sizeof(Nobe));
    p->data = e;
    
    cache = (*L)->next;
    (*L)->next = p;
    *L = p;
    p->next = cache;
    return OK;
}

int listMontage(LinkList *L_main, LinkList *L_free)
{
    // 拼接链表
    LinkList p = (*L_free)->next;
    (*L_free)->next = (*L_main)->next->next;

    // free((*L_main)->next);
    (*L_main)->next = p;
    return OK;
}

int main(void)
{
    // (单)循环链表 + 终端结点 + 头节点 版
    LinkList tail = NULL;
    LinkList tail_two = NULL;
    initList(&tail);
    listAdd(&tail, 1);
    listAdd(&tail, 2);
    listAdd(&tail, 3);
    listAdd(&tail, 4);

    initList(&tail_two);
    listAdd(&tail_two, 5);
    listAdd(&tail_two, 6);
    listAdd(&tail_two, 7);
    listAdd(&tail_two, 8);
    listPrint(tail);
    listPrint(tail_two);

    listMontage(&tail_two, &tail);
    listPrint(tail);
    listPrint(tail_two);

    return 0;
}
```
