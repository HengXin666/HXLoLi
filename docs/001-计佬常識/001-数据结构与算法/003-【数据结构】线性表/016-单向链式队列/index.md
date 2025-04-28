# 单向链式队列
## 代码实现

```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

typedef char ElemType;
typedef int State;      // 状态 OK or ERROR

typedef struct QNode
{
    ElemType data;
    struct QNode *next;
} QNobe, *QueuePrt;

typedef struct
{
    QueuePrt front, rear;// 队头, 尾指针
} LinkQueue;

State initQueue(LinkQueue *Q);      // 初始化
State destroyQueue(LinkQueue *Q);   // 销毁队列
State enQueue(LinkQueue *Q, ElemType e);    // 队尾插入元素e, 并且成为新的队尾
State deQueue(LinkQueue *Q, ElemType *e);   // 删除队列元素e, 并且返回e的值

State printQueue(LinkQueue Q);              // 打印队列 从头到尾

State initQueue(LinkQueue *Q)
{
    // 初始化
    QueuePrt p;
    p = (QueuePrt)malloc(sizeof(QNobe));    // 申请元素作为头结点
    if (!p)
        return ERROR;
    
    p->next = NULL;
    Q->front = p;
    Q->rear = p;
    return OK;
}

State enQueue(LinkQueue *Q, ElemType e)
{
    // 队尾插入元素e, 并且成为新的队尾
    QueuePrt p;
    p = (QueuePrt)malloc(sizeof(QNobe));    // 申请元素作为头结点
    if (!p)
        return ERROR;
    p->data = e;

    p->next = NULL;
    Q->rear->next = p;
    Q->rear = p;
    return OK;
}

State deQueue(LinkQueue *Q, ElemType *e)
{
    // 删除队列元素e, 并且返回e的值
    QueuePrt p_free = NULL;
    if (!(p_free = Q->front->next))
        return ERROR;
    
    *e = Q->front->next->data;   
    Q->front->next = Q->front->next->next;
    free(p_free);
    return OK;
}

State destroyQueue(LinkQueue *Q)
{
    // 销毁队列 --- 简易方法 ---
    // char e;
    // while (deQueue(Q, &e))
    //     ;
    QueuePrt p_free = NULL;
    QueuePrt p = Q->front;
    while (p)
    {
        p_free = p;
        p = p->next;
        free(p_free);
    }
    Q->front = Q->rear = NULL;
    return OK;
}

State printQueue(LinkQueue Q)
{
    // 打印队列 从头到尾
    QueuePrt p = Q.front->next;
    while (p)
    {
        putchar(p->data);
        p = p->next;
    }

    return OK;
}

int main(void)
{
    // 队列 - 链式储存结构 - 带头节点版本
    LinkQueue queue;
    char str[32] = "I LOVE FishC !", e;
    initQueue(&queue);
    for (int i = 0; str[i] != '\0'; i++)
    {
        enQueue(&queue, str[i]);
    }

    deQueue(&queue, &e);
    printf("删除了 %c\n", e);
    printQueue(queue);

    destroyQueue(&queue);
    
    return 0;
}
```
