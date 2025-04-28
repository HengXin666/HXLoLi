# (顺序存储结构)动态长度的数组栈
## 代码实现

```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

#define STACK_INIT_SIZE 100
#define STACK_REALLOC_SIZE 4

typedef int State;      // 状态 OK or ERROR
typedef int SElemType;

typedef struct SqStack
{
    SElemType *top;     // 栈顶
    SElemType *base;    // 栈底
    int stackSize;      // 栈大小(字节)
}SqStack;

State initStack(SqStack *S);        // 初始化操作, 建立一个空栈的S
State destroyStack(SqStack *S);     // 销毁栈S
State pushStack(SqStack *S, int e); // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
State popStack(SqStack *S, int *e); // 删除栈顶元素, 并用e返回其值 
int StackLen(SqStack S);            // 返回栈的长度

State printStack(SqStack S);       // 先出先打印

int StackLen(SqStack S)
{
    // 返回栈的长度
    return S.top - S.base;// C语言の智能: 相同类型的内存地址相减, 得到的不是内存差 而是 内存差/sizeof(指针类型)
    // 大地址 - 小地址 正确
    // 小地址 - 大地址 也正确, 只是为负数罢了
}

State initStack(SqStack *S)
{
    // 初始化操作, 建立一个空栈的S
    S->base = (SElemType *)malloc(STACK_INIT_SIZE * sizeof(SElemType));
    if (!S->base)
        return ERROR;
    S->top = S->base;   // 栈顶就是栈底
    S->stackSize = STACK_INIT_SIZE * sizeof(SElemType);
    return OK;
}

State pushStack(SqStack *S, int e)
{
    // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
    if (S->top - S->base >= S->stackSize)
    {
        realloc(S->base, S->stackSize + STACK_REALLOC_SIZE * sizeof(SElemType));
        if (!S->base)
            return ERROR;
        S->stackSize = S->stackSize + STACK_REALLOC_SIZE * sizeof(SElemType);
    }

    *(S->top)++ = e;
    return OK;
}

State popStack(SqStack *S, int *e)
{
    // 删除栈顶元素, 并用e返回其值 
    if (S->top == S->base)
        return ERROR;
    
    *e = *--S->top;
    return OK;
}

State destroyStack(SqStack *S)
{
    // 销毁栈S
    free(S->base);// 因为内存是S->base申请的, 所以销毁它就OK了
    S->top = NULL;
    S->base = NULL;
    S->stackSize = 0;
    return OK;
}

State printStack(SqStack S)
{
    // 先出先打印
    while (S.top != S.base)
    {
        printf("%d ", *--S.top);
    }

    return OK;
}

int main(void)
{
    // 栈 小甲鱼表示: 更符合栈在操作系统中的样子 的说
    // 本质上和数组没有区别(只是现在使用指针访问数组, 然后数组可以变长)
    SqStack s;
    int e;
    initStack(&s);
    pushStack(&s, 1);
    pushStack(&s, 2);
    pushStack(&s, 3);
    popStack(&s, &e);
    pushStack(&s, 4);
    printf("弹栈了 %d, 目前栈长 %d\n", e, StackLen(s));
    printStack(s);

    destroyStack(&s);
    
    return 0;
}
```
