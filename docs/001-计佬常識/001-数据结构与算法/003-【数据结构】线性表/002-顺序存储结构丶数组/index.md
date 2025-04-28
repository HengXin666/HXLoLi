# 线性存储结构-数组
## 代码实现

```C
#include <stdio.h>

#define MAXSIZE 20
#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

// --- 线性表 封装 --- (实际上是对数组进行封装)
typedef int ElemType;
typedef struct{
    ElemType data[MAXSIZE];
    int length;
} SqList;

void initList(SqList *L);                  // 初始化操作, 建立一个空的线性表
int listEmpty(SqList L);                   // 判断是否为空表 1是0否
void clearList(SqList *L);                 // 将线性表清空
void getElem(SqList L, int i, int *e);     // 将线性表L第i个元素返回给e
int locateElem(SqList L, int e);           // 查找线性表L中是否有值为e的元素, 有则返回其在表中的序号, 否0
int listInsert(SqList *L, int i, int e);   // 在线性表L中第i个位置插入新元素e
int listDelete(SqList *L, int i, int *e);  // 删除线性表L中第i个位置的元素, 并返回其值给e
int listAdd(SqList *L, int e);             // 在线性表尾部添加元素
int listLength(SqList L);                  // 返回线性表的长度

int printSqList(SqList L);                 // 打印线性表的内容

void initList(SqList *L)
{
    // 初始化操作, 建立一个空的线性表
    L->length = 0;
}

int listEmpty(SqList L)
{
    // 判断是否为空表 1是0否
    if (L.length == 0)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void clearList(SqList *L)
{
    // 将线性表清空
    L->length = 0;
}

void getElem(SqList L, int i, int *e)
{
    // 将线性表L第i个元素返回给e
    if (i <= L.length && i > 0)
    {
        *e = L.data[i - 1];
    }
}

int locateElem(SqList L, int e)
{
    // 查找线性表L中是否有值为e的元素, 有则返回其在表中的序号, 否0
    for (int i = 0; i < L.length; i++)
    {
        if (e == L.data[i])
        {
            return i + 1;
        }
    }

    return FALSE;
}

int listInsert(SqList *L, int i, int e)
{
    // 在线性表L中第i个位置插入新元素e
    if (i > 0 && i <= L->length + 1 && L->length < MAXSIZE)
    {
        int cache;// 可读性几乎为 O
        while (1)
        {
            cache = L->data[i - 1];
            L->data[i - 1] = e;

            if (++i <= L->length + 1)
            {
                e = L->data[i - 1];
                L->data[i++ - 1] = cache; 
            }
            else
            {
                L->length++;
                return OK;
            }
        }
    }
    else
    {
        return ERROR;
    }
}

int listDelete(SqList *L, int i, int *e)
{
    // 删除线性表L中第i个位置的元素, 并返回其值给e
    if (i > 0 && i <= L->length + 1 && L->length < MAXSIZE && L->length != 0)
    {
        *e = L->data[i - 1];
        while (1)
        {
            L->data[i - 1] = L->data[i];
            i++;
            if (i == L->length)
            {
                L->length--;
                return OK;
            }
        }
    }
    else
    {
        return ERROR;
    }
}

int listLength(SqList L)
{
    // 返回线性表的长度
    return L.length;
}

int printSqList(SqList L)
{
    // 打印线性表的内容
    for (int i = 0; i < L.length; i++)
    {
        printf("[%d]: %d\n", i + 1, L.data[i]);
    }
}

int listAdd(SqList *L, int e)
{
    if (L->length < MAXSIZE)
    {
        L->data[L->length++] = e;
        return OK;
    }
    return ERROR;
}

int main(void)
{
    // 线性表的数组版本 --- 下面是测试代码 ---
    SqList list;
    int e;
    initList(&list);
    printSqList(list);
    listInsert(&list, 1, 233);
    listAdd(&list, 2);
    listAdd(&list, 4);
    listAdd(&list, 6);
    listAdd(&list, 8);
    listInsert(&list, 2, 666);
    listDelete(&list, 3, &e);
    printf("删除了%d\n", e);
    printSqList(list);
    printf("线性表的长度是: %d\n", listLength(list));

    return 0;
}
```
