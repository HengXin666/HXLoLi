# 循环顺序队列
## 代码实现

```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

#define MAXSIZE 8

typedef int ElemType;
typedef int State;      // 状态 OK or ERROR

typedef struct
{
    ElemType deta[MAXSIZE];
    int front;  // 头
    int rear;   // 尾
} SqQueue;

State initQueue(SqQueue *Q);      // 初始化
State destroyQueue(SqQueue *Q);   // 销毁队列
State enQueue(SqQueue *Q, ElemType e);    // 队尾插入元素e, 并且成为新的队尾
State deQueue(SqQueue *Q, ElemType *e);   // 删除队列元素e, 并且返回e的值

State printQueue(SqQueue Q);              // 打印队列 从头到尾

State initQueue(SqQueue *Q)
{
    // 初始化
    Q->front = 0;
    Q->rear = 0;
    return OK;
}

State enQueue(SqQueue *Q, ElemType e)
{
    // 队尾插入元素e, 并且成为新的队尾
    if ((Q->rear + 1) % MAXSIZE == Q->front)
        return ERROR;// 已满
    
    Q->deta[Q->rear] = e;
    Q->rear = ++Q->rear % MAXSIZE;
    // 看来空余一个元素表示满是必需滴
    return OK;
}

State deQueue(SqQueue *Q, ElemType *e)
{
    // 删除队列元素e, 并且返回e的值
    if (Q->front == Q->rear)
        return ERROR;

    *e = Q->deta[Q->front];
    Q->front = ++Q->front % MAXSIZE;
    return OK;
}

State printQueue(SqQueue Q)
{
    // 打印队列 从头到尾
    int i = Q.front;
    while ((i + 1) % MAXSIZE != Q.front && i != Q.rear)
    {
        printf("%d ", Q.deta[i]);
        i = ++i % MAXSIZE;
    }

    return OK;
}

int main(void)
{
    // 循环队列 - 顺序存储结构
    SqQueue queue;
    int e;
    initQueue(&queue);
    for (int i = 0; i < 16; i++)
    {
        enQueue(&queue, i);
        
    }
    
    deQueue(&queue, &e);
    printf("出列%d\n", e);
    printQueue(queue);
    
    return 0;
}
```
