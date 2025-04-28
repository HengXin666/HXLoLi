# (顺序存储结构)固定大小的数组栈
## 代码实现

```C
#include <stdio.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

#define MAXSIZE 16// 栈的大小

typedef int State;      // 状态 OK or ERROR
typedef int SElemType;

typedef struct SqStack
{
    SElemType data[MAXSIZE];
    int top;// 用于栈顶的指针
}SqStack;

State initStack(SqStack *S);        // 初始化操作, 建立一个空栈的S
State destroyStack(SqStack *S);     // 销毁栈S
// 当声明为指针时候可以malloc开辟空间则可以使用这个函数free ?

State pushStack(SqStack *S, int e); // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
State popStack(SqStack *S, int *e); // 删除栈顶元素, 并用e返回其值 

State printStack(SqStack *S);       // 先出先打印

State initStack(SqStack *S)
{
    // 初始化操作, 建立一个空栈的S
    S->top = -1;
    return OK;
}

State pushStack(SqStack *S, int e)
{
    // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
    if (S->top == MAXSIZE - 1)
    {
        return ERROR;
    }

    S->data[++(S->top)] = e;
    return OK;
}

State popStack(SqStack *S, int *e)
{
    // 删除栈顶元素, 并用e返回其值 
    if (S->top == -1)
    {
        return ERROR;
    }

    *e = S->data[(S->top)--];
    return OK;
}

State printStack(SqStack *S)
{
    // 先出先打印

    for (int i = S->top; i > -1; i--)
    {
        printf("%d ", S->data[i]);
    }

    return OK;
}

int main(void)
{
    // 栈 的 数组(顺序存储结构)表示
    SqStack s;
    int e;
    initStack(&s);
    pushStack(&s, 1);
    pushStack(&s, 2);
    pushStack(&s, 3);
    popStack(&s, &e);
    pushStack(&s, 4);
    printStack(&s);
    printf("曾经弹栈了%d\n", e);
    
    return 0;
}
```
