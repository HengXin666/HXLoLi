## 课后习题
### 课堂练习: 进制转换 二进制 转为 十进制

```C++
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

int decimalism(SqStack S);          // 返回十进制数
State printStack(SqStack S);        // 先出先打印

int decimalism(SqStack S)
{
    // 返回十进制数
    int decimalism_num = 0;
    int len = StackLen(S);
    for (int i = 1; i <= len;  i++)
    {
        decimalism_num += *(S.top - i) * (1 << i - 1);// 使用位操作就不用数学库
    }

    return decimalism_num;
}

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
    // 栈 指针顺序表示
    // 课堂练习: 进制转换 二进制 转为 十进制
    SqStack s;
    char str_binary_system[128];
    initStack(&s);
    printf("请输入一个二进制数:");
    scanf("%s", str_binary_system);
    getchar();
    for (int i = 0; str_binary_system[i] != '\0'; ++i)
    {
        if (str_binary_system[i] == '1')
        {
            pushStack(&s, 1);
        }
        else
        {
            pushStack(&s, 0);
        }
    }
    printf("对应的十进制是: %d\n", decimalism(s));

    destroyStack(&s);

    return 0;
}
```

### 课堂练习: 进制转换 二进制 转为 八进制

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

int octal(SqStack S, SqStack *S_O); // 返回十进制数
State printStack(SqStack S);        // 先出后打印 awa

int octal(SqStack S, SqStack *S_O)
{
    // 压栈八进制数
    int len = StackLen(S);
    int octal_num;
    for (int i = 1; i <= len;)
    {
        octal_num = 0;
        for (int j = 2; j > -1 && i <= len; j--, i++)
        {
            octal_num += *(S.top - i) * (1 << j);// 使用位操作就不用数学库
        }
        pushStack(S_O, octal_num);
    }

    return OK;
}

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
    // 先出后打印
    for (int i = StackLen(S); i > 0; i--)
    {
        printf("%d ", *(S.top - i));
    }

    return OK;
}

int main(void)
{
    // 栈 指针顺序表示
    // 课堂练习: 进制转换 二进制 转为 八进制
    SqStack s, s_octal;
    char str_binary_system[128];
    initStack(&s);
    initStack(&s_octal);
    printf("请输入一个二进制数:");
    scanf("%s", str_binary_system);
    getchar();
    for (int i = 0; str_binary_system[i] != '\0'; ++i)
    {
        if (str_binary_system[i] == '1')
        {
            pushStack(&s, 1);
        }
        else
        {
            pushStack(&s, 0);
        }
    }
    octal(s, &s_octal);
    printf("对应的八进制是: ");
    printStack(s_octal);

    destroyStack(&s);
    destroyStack(&s_octal);
    
    return 0;
}
```

### 逆波兰表达式计算加减乘除

怎么用的? 我忘记了...

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
    S->count--;
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
    // 栈 链式存储结构及实现 + 逆波兰表达式
    LinkStack stack;
    int e, i, x, y;
    char str[8];
    initStack(&stack);
    do
    {
        scanf("%s", str);
        getchar();
        for (i = 0; str[i] != '\0'; i++)
        {
            switch(str[i])
            {
                case '+':
                popStack(&stack, &x);
                popStack(&stack, &y);
                pushStack(&stack, x + y);
                goto _continue;
                case '-':
                popStack(&stack, &x);
                popStack(&stack, &y);
                pushStack(&stack, y - x);
                goto _continue;
                case '*':
                popStack(&stack, &x);
                popStack(&stack, &y);
                pushStack(&stack, x * y);
                goto _continue;
                case '/':
                popStack(&stack, &x);
                popStack(&stack, &y);
                pushStack(&stack, y / x);
                goto _continue;
                case '#':
                goto exit;
                default:
                e = e * 10 + str[i] - '0';
                break;
            }
        }
        pushStack(&stack, e);
        e = 0;
        _continue:
        continue;
    } while (1);
    exit:
    printStack(stack);

    destroyStack(&stack);
    while (1)
        getchar();
    return 0;
}
```


### 输入中缀表达式输出逆波兰表达式

```C
#include <stdio.h>
#include <stdlib.h>

#define OK 1    // 程序正常运行
#define ERROR 0 // 程序运行报错
#define TRUE 1  //
#define FALSE 0 //

typedef int State;      // 状态 OK or ERROR
typedef char SElemType;

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
State pushStack(LinkStack *S, char e); // 若栈S存在, 插入新元素e到栈中并且成为新的栈顶元素
State popStack(LinkStack *S, char *e); // 删除栈顶元素, 并用e返回其值 

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

State pushStack(LinkStack *S, char e)
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

State popStack(LinkStack *S, char *e)
{
    // 删除栈顶元素, 并用e返回其值 
    if (!S->count)
        return ERROR;
    
    *e = S->top->data;
    LinkStackPtr p_free = S->top;
    S->top = S->top->next;
    S->count--;
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
    // 栈 链式存储结构及实现 + 输入中缀表达式输出逆波兰表达式
    LinkStack stack;
    char scan, cache;
    initStack(&stack);
    // 输入
    printf("请输入一个中缀表达式(数字之间紧密连接, 符号之间空格连接, 以 '#' 代表结束):\n");
    while (1)
    {
        scan = getchar();
        if (scan == '(')
        {   // 压栈 '('
            pushStack(&stack, scan);
            getchar();
            continue;
        }
        else
        {
            // 数字处理
            if (scan >= '0' && scan <= '9')
            {
                while (1)
                {
                    putchar(scan);
                    scan = getchar();
                    if (scan == ' ')
                    {
                        putchar(' ');
                        break;
                    }
                }
                continue;
            }
            else
            {
                getchar();
                if (scan == ')')
                {
                    while (1)
                    {
                        popStack(&stack, &cache);
                        if (cache == '(')
                            break;
                        putchar(cache);
                        putchar(' ');
                    }
                    continue;
                }

                while (1)
                {
                    if (!popStack(&stack, &cache))
                    {   // 空栈则压栈
                        pushStack(&stack, scan);
                        break;
                    }

                    if (cache == '(' || ((scan == '*' || scan == '/') && (cache == '+' || cache == '-')))
                    {   // '(' 并且无内容 或者 scan优先度高于cache 则压栈
                        pushStack(&stack, cache);
                        pushStack(&stack, scan);
                        break;
                    }
                    else if (scan != '#')
                    {   // 弹栈打印
                        putchar(cache);
                        putchar(' ');
                    }
                    else
                    {   // 结束
                        pushStack(&stack, cache);
                        while(stack.count)
                        {
                            popStack(&stack, &scan);
                            putchar(scan);
                            putchar(' ');
                        }
                        goto owari;
                    }
                }
            }
        }
    }
    owari:
    destroyStack(&stack);
    getchar();
    getchar();
    return 0;
}
```
