# 顺序存储结构-静态链表
## 代码实现

```C
#include <stdio.h>

#define MAXSIZE 600
#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

// --- 线性表 封装 --- (静态链表)
typedef int ElemType;
typedef struct{
    ElemType data;  // 数据
    int cur;        // 游标
} Component, StaticLinkList[MAXSIZE];

void initList(StaticLinkList L);                    // 初始化操作, 建立一个空的线性表
int listEmpty(StaticLinkList L);                    // 判断是否为空表 1是0否
void clearList(StaticLinkList L);                   // 将线性表清空
void getElem(StaticLinkList L, int i, int *e);      // 将线性表L第i个元素返回给e
int locateElem(StaticLinkList L, int e);            // 查找线性表L中是否有值为e的元素, 有则返回其在表中的序号, 否0
int listInsert(StaticLinkList L, int i, int e);     // 在线性表L中第i个位置插入新元素e
int listDelete(StaticLinkList L, int i, int *e);    // 删除线性表L中第i个位置的元素, 并返回其值给e

int listAdd(StaticLinkList L, int e);               // 在线性表尾部添加元素
int listLength(StaticLinkList L);                   // 返回线性表的长度
int printStaticLinkList(StaticLinkList L);          // 打印线性表的内容

void initList(StaticLinkList L)
{
    // 初始化操作, 建立一个空的线性表
    for (int i = 0; i < MAXSIZE - 1; i++)
    {
        L[i].cur = i + 1;
    }

    L[MAXSIZE - 1].cur = 0;
}

int listAdd(StaticLinkList L, int e)
{
    // 在线性表尾部添加元素
    int index = MAXSIZE - 1;
    L[L[0].cur].data = e;
    if (L[MAXSIZE - 1].cur == 0)// 修改头指针, 指向第一个有效元素
    {
        L[MAXSIZE - 1].cur = L[0].cur;
    }
    else
    {
        while (L[index].cur)
        {
            index = L[index].cur;
        }
        L[index].cur = L[0].cur;
    }
    L[L[0].cur].cur = 0;        // 结尾
    L[0].cur++;                 // 备用链表的第一个结点下标
}

int printStaticLinkList(StaticLinkList L)
{
    // 打印线性表的内容
    int i = L[MAXSIZE - 1].cur;
    while (i)
    {
        printf("[%d]: %d\n", i, L[i].data);
        i = L[i].cur;
    }
}

int listInsert(StaticLinkList L, int i, int e)
{
    // 在线性表L中第i个位置插入新元素e
    int j = 1;
    int index = MAXSIZE - 1;
    int index_cache = L[L[0].cur].cur;
    while (j < i && index)
    {   
        index = L[index].cur;
        ++j;
    }
    
    if (!index || i < 1)
    {
        return ERROR;
    }

    L[L[0].cur].data = e;               // 尾加
    L[L[0].cur].cur = L[index].cur;     // 尾标为目标前驱标
    L[index].cur = L[0].cur;            // 目标前驱表 赋值 尾加索引
    L[0].cur = index_cache;             // [0]游标 赋值 尾加原游标

    return OK;
}

int listDelete(StaticLinkList L, int i, int *e)
{
    // 删除线性表L中第i个位置的元素, 并返回其值给e
    int j = 1;
    int index = MAXSIZE - 1;
    
    while (j < i && index)
    {   
        index = L[index].cur;
        ++j;
    }

    if (!index || i < 1)
    {
        return ERROR;
    }
    
    int index_cache = L[index].cur;
    *e = L[L[index].cur].data;
    L[0].cur = L[index].cur;

    L[index].cur = L[L[index].cur].cur;
    L[index_cache].cur = L[L[0].cur].cur;

    return OK;
}

int main(void)
{
    // 单链表 - 静态链表
    // 这里是数据设计, 游标 == 0, 代表结束
    StaticLinkList list;
    int e;
    initList(list);
    listAdd(list, 233);
    listAdd(list, 666);
    listAdd(list, 123);
    listAdd(list, 456);
    listAdd(list, 789);
    listAdd(list, 100);
    printf("%d\n", listInsert(list, 3, 888));
    listDelete(list, 2, &e);
    printf("删除了%d\n", e);
    listAdd(list, 101);
    printStaticLinkList(list);

    return 0;
}
```
