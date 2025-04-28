# (链式存储结构)链表的头插法的栈
## 代码实现

```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

typedef int State;      // 状态 OK or ERROR
typedef int SElemType;

typedef struct StackNobe
{
    SElemType data;
    struct StackNobe *next;
}StackNobe, *LinkStackPtr;

typedef struct LinkStack
{
    LinkStackPtr top;
    int count;
}LinkStack;

State initStack(LinkStack *S);        // 初始化操作, 建立一个空栈的S
State destroyStack(LinkStack *S);     // 销毁栈S
State pushStack(LinkStack *S, int e); // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
State popStack(LinkStack *S, int *e); // 删除栈顶元素, 并用e返回其值 

State printStack(LinkStack S);       // 先出先打印

State initStack(LinkStack *S)
{
    // 初始化操作, 建立一个空栈的S
    S->top = NULL;
    S->count = 0;
    return OK;
}

State destroyStack(LinkStack *S)
{
    // 销毁栈S
    LinkStackPtr p_free = NULL;
    while (S->top != NULL)
    {
        p_free = S->top;
        S->top = S->top->next;
        free(p_free);
    }
    
    return OK;
}

State pushStack(LinkStack *S, int e)
{
    // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
    LinkStackPtr p;
    p = (LinkStackPtr)malloc(sizeof(StackNobe));
    if (!p)
        return ERROR;
    
    p->data = e;
    p->next = S->top;
    S->top = p;
    S->count++;
    return OK;
}

State popStack(LinkStack *S, int *e)
{
    // 删除栈顶元素, 并用e返回其值 
    if (!S->count)
        return ERROR;
    
    *e = S->top->data;
    LinkStackPtr p_free = S->top;
    S->top = S->top->next;
    free(p_free);
    return OK;
}

State printStack(LinkStack S)
{
    // 先出先打印
    while (S.top != NULL)
    {
        printf("%d ", S.top->data);
        S.top = S.top->next;
    }
    
    return OK;
}

int main(void)
{
    // 栈 链式存储结构及实现
    LinkStack stack;
    int e;
    initStack(&stack);
    pushStack(&stack, 1);
    pushStack(&stack, 2);
    popStack(&stack, &e);
    pushStack(&stack, 3);
    pushStack(&stack, 4);
    printf("弹栈了 %d\n", e);
    printStack(stack);

    destroyStack(&stack);
    
    return 0;
}
```
