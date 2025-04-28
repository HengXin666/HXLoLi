# 线程函数
## 获取线程id
### pthread_self()
每一个线程都有一个唯一的线程ID，ID类型为`pthread_t`，这个ID是一个无符号长整形数，如果想要得到当前线程的线程ID，可以调用如下函数:

```C++
pthread_t pthread_self(void);    // 返回当前线程的线程ID
```
## 创建线程
### pthread_create()

在一个进程中调用线程创建函数，就可得到一个子线程，和进程不同，需要给每一个创建出的线程指定一个处理函数，否则这个线程无法工作。

```C++
#include <pthread.h>
int pthread_create(pthread_t *thread, const pthread_attr_t *attr,
                   void *(*start_routine) (void *), void *arg);
// Compile and link with -pthread, 线程库的名字叫pthread, 全名: libpthread.so libptread.a
```

- **参数**:

    - `thread`: 传出参数，是无符号长整形数，线程创建成功, 会将线程ID写入到这个指针指向的内存中

    - `attr`: 线程的属性, 一般情况下使用默认属性即可, 写NULL

    - `start_routine`: 函数指针，创建出的子线程的处理动作，也就是该函数在子线程中执行。

    - `arg`: 作为实参传递到 start_routine 指针指向的函数内部

- **返回值**: 线程创建成功返回0，创建失败返回对应的错误号

### 示例
下面是创建线程的示例代码，在创建过程中一定要保证编写的线程函数与规定的函数指针类型一致：`void *(*start_routine) (void *)`:

```C++
#include <pthread.h>
#include <stdio.h>

void for_demo_fun(const char * str)
{
    for (int i = 0; i < 5; ++i)
    {
        printf("[%s]: %d\n", str, i);
    }
    printf("\n");
}

void *thread_fun(void *)
{
    // 子线程也可以调用主线程的东西(准确的说应该是全局的东西, 比如这个 函数(实际上可以看做是全局变量函数指针的执行))
    for_demo_fun("子线程");
    printf("子线程id: %ld\n", pthread_self());
    return NULL;
}

int main()
{
    pthread_t th;
    // 1. 创建子线程
    pthread_create(&th, NULL, thread_fun, NULL);
    printf("子线程创建成功: id 为 %ld\n", th);

    for_demo_fun("父线程");
    printf("父线程id: %ld\n", pthread_self());

    return 0;
}
```

编译的时候要链接线程库文件（动态库）

动态库名为 `libpthread.so`需要使用的参数为 `-l`，根据规则掐头去尾最终形态应该写成：`-lpthread`（参数和参数值中间可以有空格）
```Bash
g++ 01_start.cpp -l pthread -o app
```

```Shell
[root@localhost hx_thread]# ./app 
子线程创建成功: id 为 140304202069760
[父线程]: 0
[子线程]: 0
[子线程]: 1
[子线程]: 2
[子线程]: 3
[子线程]: 4

子线程id: 140304202069760
[父线程]: 1
[父线程]: 2
[父线程]: 3
[父线程]: 4

父线程id: 140304224134976
```

(每次运行的结果都是不同的, 输出顺序什么的, 如果出现重复, 可能是由于主线程和子线程共用了标准输出缓冲区，导致输出的结果混乱(GPT说得上锁), 如果是只输出父而没有输出子, 那么实际上是因为父线程执行完就程序结束了, 未执行的子线程就被释放了(可以使用sleep等待))

## 线程退出
### pthread_exit()

在编写多线程程序的时候，如果想要让线程退出，但是不会导致虚拟地址空间的释放（针对于主线程），我们就可以调用线程库中的线程退出函数，<span style="color:red">只要调用该函数当前线程就马上退出了，并且不会影响到其他线程的正常运行，不管是在子线程或者主线程中都可以使用</span>。

```C++
#include <pthread.h>
void pthread_exit(void *retval);
```

- **参数**: 线程退出的时候携带的数据，当前子线程的主线程会得到该数据。如果不需要使用，指定为`NULL`

等价于 线程函数`return`时候的返回值

### 示例
下面是线程退出的示例代码，可以在任意线程的需要的位置调用该函数:

```C++
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

// 子线程的处理代码
void *working(void *arg)
{
    sleep(1);
    printf("我是子线程, 线程ID: %ld\n", pthread_self());
    for (int i = 0; i < 9; ++i)
    {
        if (i == 6)
        {
            pthread_exit(NULL); // 直接退出子线程
        }
        printf("child == i: = %d\n", i);
    }
    return NULL;
}

int main()
{
    // 1. 创建一个子线程
    pthread_t tid;
    pthread_create(&tid, NULL, working, NULL);

    printf("子线程创建成功, 线程ID: %ld\n", tid);
    // 2. 子线程不会执行下边的代码, 主线程执行
    printf("我是主线程, 线程ID: %ld\n", pthread_self());
    for (int i = 0; i < 3; ++i)
    {
        printf("i = %d\n", i);
    }

    // 主线程调用退出函数退出, 地址空间不会被释放(即 子线程仍然可以继续运行)
    pthread_exit(NULL);

    return 0;
}
```

## 线程回收
### pthread_join()

线程和进程一样，子线程退出的时候其内核资源主要由主线程回收，线程库中提供的线程回收函叫做`pthread_join()`，这个函数是一个 **`阻塞函数`**，<span style="color:red">如果还有子线程在运行，调用该函数就会阻塞，子线程退出函数解除阻塞进行资源的回收，函数被调用一次，只能回收一个子线程，如果有多个子线程则需要循环进行回收</span>。

另外通过线程回收函数还可以获取到子线程退出时传递出来的数据，函数原型如下:

```C++
#include <pthread.h>
// 这是一个阻塞函数, 子线程在运行这个函数就阻塞
// 子线程退出, 函数解除阻塞, 回收对应的子线程资源, 类似于回收进程使用的函数 wait()
int pthread_join(pthread_t thread, void **retval);
```

- **参数**:

    - `thread`: 要被回收的子线程的线程ID
    
    - `retval`: 二级指针, 指向一级指针的地址, 是一个传出参数, 这个地址中存储了`pthread_exit()`传递出的数据，如果不需要这个参数，可以指定为`NULL`

- **返回值**: 线程回收成功返回`0`，回收失败返回错误号。

注意区别:
- `pthread_join`的 `void **retval`参数是取得该线程函数`返回值`

- `pthread_create`的第四个参数是给函数指针 `void *(*)(void *)`的函数参数!

#### 回收子线程数据
在子线程退出的时候可以使用`pthread_exit()`的参数将数据传出，在回收这个子线程的时候可以通过`phread_join()`的第二个参数来接收子线程传递出的数据。接收数据有很多种处理方式，下面来列举几种:

##### 使用子线程栈 (間違い)

```C++
// pthread_join.c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

// 定义结构
struct Persion
{
    int id;
    char name[36];
    int age;
};

// 子线程的处理代码
void* working(void* arg)
{
    printf("我是子线程, 线程ID: %ld\n", pthread_self());
    for(int i = 0; i < 9; ++i)
    {
        printf("child == i: = %d\n", i);
        if(i == 6)
        {
            struct Persion p;
            p.age  =12;
            strcpy(p.name, "tom");
            p.id = 100;
            // 该函数的参数将这个地址传递给了主线程的pthread_join()
            pthread_exit(&p);
        }
    }
    return NULL;    // 代码执行不到这个位置就退出了
}

int main()
{
    // 1. 创建一个子线程
    pthread_t tid;
    pthread_create(&tid, NULL, working, NULL);

    printf("子线程创建成功, 线程ID: %ld\n", tid);
    // 2. 子线程不会执行下边的代码, 主线程执行
    printf("我是主线程, 线程ID: %ld\n", pthread_self());
    for(int i = 0; i < 3; ++i)
    {
        printf("i = %d\n", i);
    }

    // 阻塞等待子线程退出
    void* ptr = NULL;
    // ptr是一个传出参数, 在函数内部让这个指针指向一块有效内存
    // 这个内存地址就是pthread_exit() 参数指向的内存
    pthread_join(tid, &ptr);
    // 打印信息
    struct Persion* pp = (struct Persion*)ptr;
    printf("子线程返回数据: name: %s, age: %d, id: %d\n", pp->name, pp->age, pp->id);
    printf("子线程资源被成功回收...\n");
    
    return 0;
}
```

输出:

```Shell
# 编译代码
$ gcc pthread_join.c -lpthread
# 执行程序
$ ./a.out 
子线程创建成功, 线程ID: 140652794640128
我是主线程, 线程ID: 140652803008256
i = 0
i = 1
i = 2
我是子线程, 线程ID: 140652794640128
child == i: = 0
child == i: = 1
child == i: = 2
child == i: = 3
child == i: = 4
child == i: = 5
child == i: = 6
子线程返回数据: name: , age: 0, id: 0
子线程资源被成功回收...
```
> [!TIP]
> 通过打印的日志可以发现，在主线程中没有没有得到子线程返回的数据信息，具体原因是这样的:
>
> <span style="color:red">如果多个线程共用同一个虚拟地址空间，每个线程在栈区都有一块属于自己的内存，相当于栈区被这几个线程平分了，当线程退出，线程在栈区的内存也就被回收了，因此随着子线程的退出，写入到栈区的数据也就被释放了。</span>

##### 使用全局变量 (必要が無い)
位于同一虚拟地址空间中的线程，虽然不能共享栈区数据，但是可以共享全局数据区和堆区数据，因此在子线程退出的时候可以将传出数据存储到全局变量、静态变量或者堆内存中。在下面的例子中将数据存储到了全局变量中：

```C++
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

// 定义结构
struct Persion
{
    int id;
    char name[36];
    int age;
};

struct Persion p;    // 定义全局变量

// 子线程的处理代码
void* working(void* arg)
{
    printf("我是子线程, 线程ID: %ld\n", pthread_self());
    for(int i=0; i<9; ++i)
    {
        printf("child == i: = %d\n", i);
        if(i == 6)
        {
            // 使用全局变量
            p.age  =12;
            strcpy(p.name, "tom");
            p.id = 100;
            // 该函数的参数将这个地址传递给了主线程的pthread_join()
            pthread_exit(&p);
        }
    }
    return NULL;
}

int main()
{
    // 1. 创建一个子线程
    pthread_t tid;
    pthread_create(&tid, NULL, working, NULL);

    printf("子线程创建成功, 线程ID: %ld\n", tid);
    // 2. 子线程不会执行下边的代码, 主线程执行
    printf("我是主线程, 线程ID: %ld\n", pthread_self());
    for(int i=0; i<3; ++i)
    {
        printf("i = %d\n", i);
    }

    // 阻塞等待子线程退出
    void* ptr = NULL;
    // ptr是一个传出参数, 在函数内部让这个指针指向一块有效内存
    // 这个内存地址就是pthread_exit() 参数指向的内存
    pthread_join(tid, &ptr);
    // 打印信息
    struct Persion* pp = (struct Persion*)ptr;
    printf("name: %s, age: %d, id: %d\n", pp->name, pp->age, pp->id);
    printf("子线程资源被成功回收...\n");
    
    return 0;
}
```

##### 使用主线程栈 「お勧め」

虽然每个线程都有属于自己的栈区空间，但是<span style="color:red">位于同一个地址空间的多个线程是可以相互访问对方的栈空间上的数据的</span>。由于很多情况下还需要在主线程中回收子线程资源，所以主线程一般都是最后退出，基于这个原因在下面的程序中将子线程返回的数据保存到了主线程的栈区内存中:

```C++
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

// 定义结构
struct Persion
{
    int id;
    char name[36];
    int age;
};

// 子线程的处理代码
void *working(void *arg)
{
    struct Persion *p = (struct Persion *)arg;
    printf("我是子线程, 线程ID: %ld\n", pthread_self());
    for (int i = 0; i < 9; ++i)
    {
        printf("child == i: = %d\n", i);
        if (i == 6)
        {
            // 使用主线程的栈内存
            p->age = 12;
            strcpy(p->name, "tom");
            p->id = 100;
            // 该函数的参数将这个地址传递给了主线程的pthread_join()
            pthread_exit(p);
        }
    }
    return p;
}

int main()
{
    // 1. 创建一个子线程
    pthread_t tid;

    struct Persion p;
    // 主线程的栈内存传递给子线程
    pthread_create(&tid, NULL, working, &p);

    printf("子线程创建成功, 线程ID: %ld\n", tid);
    // 2. 子线程不会执行下边的代码, 主线程执行
    printf("我是主线程, 线程ID: %ld\n", pthread_self());
    for (int i = 0; i < 3; ++i)
    {
        printf("i = %d\n", i);
    }

    // 阻塞等待子线程退出
    pthread_join(tid, NULL);
    // 打印信息
    printf("name: %s, age: %d, id: %d\n", p.name, p.age, p.id);
    printf("子线程资源被成功回收...\n");

    return 0;
}
```

在上面的程序中，调用`pthread_create()`创建子线程，并将主线程中栈空间变量`p`的地址传递到了子线程中，在子线程中将要传递出的数据写入到了这块内存中。也就是说在程序的`main()`函数中，通过指针变量`ptr`或者通过结构体变量`p`都可以读出子线程传出的数据。

##### 使用堆空间

如下, 我写的, up没说这种

```C++
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

typedef struct damo
{
    int d_a;
    int d_b;
    int d_sum;
} Damo;

void *fun_th(void * op)
{
    Damo *p = (Damo *)malloc(sizeof(Damo));
    if (!p)
    {
        printf("错误: malloc error!\n");
        exit(-1);
    }

    scanf("%d %d", &p->d_a, &p->d_b);
    p->d_sum = p->d_a + p->d_b;

    pthread_exit(p);

    return NULL;
}

int main() 
{
    pthread_t th;
    pthread_create(&th, NULL, fun_th, NULL);
    printf("创建子线程成功: %ld\n", th);
    Damo *p = NULL;
    pthread_join(th, (void **)&p);

    if (p)
    {
        printf("sum => %d\n", p->d_sum);
        free(p);
    }
    else
    {
        // 如果线程函数出现异常或者提前退出，就无法释放分配的内存，从而导致内存泄漏
        printf("子线程出错!");
        exit(-1);
    }
    return 0;
}
```

## 线程分离
### pthread_detach()
在某些情况下，程序中的主线程有属于自己的业务处理流程，如果让主线程负责子线程的资源回收，调用`pthread_join()`只要子线程不退出主线程就会一直被阻塞，主要线程的任务也就不能被执行了。

在线程库函数中为我们提供了线程分离函数`pthread_detach()`，调用这个函数之后指定的<span style="color:red">子线程就可以和主线程分离，当子线程退出的时候，其占用的内核资源就被系统的其他进程接管并回收了</span>。线程分离之后在主线程中使用`pthread_join()`就**回收不到子线程资源**了。

```C++
#include <pthread.h>
// 参数就子线程的线程ID, 主线程就可以和这个子线程分离了
int pthread_detach(pthread_t thread);
```

### 示例
```C++
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

void *th_fun(void *arg)
{
    printf("子线程: %ld\n", pthread_self());
    sleep(1);
    for (int i = 0; i < 10; ++i)
        printf("%d ", i);
    return NULL;
}

int main()
{
    pthread_t th;
    pthread_create(&th, NULL, th_fun, NULL);
    pthread_detach(th); // 线程分离
    printf("父线程: %ld\n", pthread_self());
    // 让主线程自己退出即可(不然还是全部资源都给你释放了(子线程无了))
    pthread_exit(NULL);
    return 0;
}
```

分析: 

1. `pthread_detach(th)`使得线程分离, 以替代会阻塞的`pthread_join()`, 并且资源也会自动回收.

2. `pthread_exit(NULL)`让主线程单独退出, 不然依旧会释放虚拟空间, 然后子线程就没了.

## 其他函数
### 线程取消
#### pthread_cancel()
线程取消的意思就是在某些特定情况下在一个线程中杀死另一个线程。使用这个函数杀死一个线程需要分两步：

1. 在线程A中调用线程取消函数pthread_cancel，指定杀死线程B，这时候线程B是死不了的
2. 在线程B中进程一次系统调用（从用户区切换到内核区）(比如运行`pintf()`就是系统调用)，否则线程B可以一直运行。

```C++
#include <pthread.h>
// 参数是子线程的线程ID
int pthread_cancel(pthread_t thread);
```

- **参数**: 要杀死的线程的线程ID
- **返回值**: 函数调用成功返回0，调用失败返回非0错误号。

#### 示例
在下面的示例代码中，主线程调用线程取消函数，只要在子线程中进行了系统调用，当子线程执行到这个位置就挂掉了。

```C++
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

// 子线程的处理代码
void* working(void* arg)
{
    int j=0;
    for(int i=0; i<9; ++i)
    {
        j++;
    }
    // 这个函数会调用系统函数, 因此这是个间接的系统调用
    printf("我是子线程, 线程ID: %ld\n", pthread_self());
    for(int i=0; i<9; ++i)
    {
        printf(" child i: %d\n", i);
    }

    return NULL;
}

int main()
{
    // 1. 创建一个子线程
    pthread_t tid;
    pthread_create(&tid, NULL, working, NULL);

    printf("子线程创建成功, 线程ID: %ld\n", tid);
    // 2. 子线程不会执行下边的代码, 主线程执行
    printf("我是主线程, 线程ID: %ld\n", pthread_self());
    for(int i=0; i<3; ++i)
    {
        printf("i = %d\n", i);
    }

    // 杀死子线程, 如果子线程中做系统调用, 子线程就结束了
    pthread_cancel(tid);

    // 让主线程自己退出即可
    pthread_exit(NULL);
    
    return 0;
}
```
> [!TIP]
> 关于系统调用有两种方式：
>
> - 直接调用Linux系统函数
> - 调用标准C库函数，为了实现某些功能，在Linux平台下标准C库函数会调用相关的系统函数

### 线程ID比较
#### pthread_equal()

在Linux中线程ID本质就是一个无符号长整形，因此可以直接使用比较操作符比较两个线程的ID，但是线程库是可以跨平台使用的，在某些平台上`pthread_t`可能不是一个单纯的整形，这中情况下比较两个线程的ID必须要使用比较函数，函数原型如下：

```C++
#include <pthread.h>
int pthread_equal(pthread_t t1, pthread_t t2);
```
- **参数**: t1 和 t2 是要比较的线程的线程ID
- **返回值**: 如果两个线程ID相等返回非0值，如果不相等返回0

## 注解
### [参考链接]

本文是学习笔记, 是跟随 [BiLiBiLi: 多线程和线程同步-C/C++](https://www.bilibili.com/video/BV1sv41177e4) + [爱编程的大丙的博客](https://subingwen.cn/linux/thread/#2-1-%E7%BA%BF%E7%A8%8B%E5%87%BD%E6%95%B0) 学习的!

### [C++11 thread库]
[std::thread](https://subingwen.cn/cpp/thread/)

### [主线程和子线程有不同的退出机制和行为]

原因不明(暂时: 待更新)

考虑以下代码:

```C++
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

void *th_fun(void *arg)
{
    printf("子线程: %ld\n", pthread_self());
    sleep(1);
    for (int i = 0; i < 10; ++i)
        printf("%d_", i);

    return NULL;
}

int main()
{
    pthread_t th;
    pthread_create(&th, NULL, th_fun, NULL);
    printf("父线程: %ld\n", pthread_self());
    return 0;
}
```
上述代码, 执行`main`函数return 后, 子线程就不执行了(因为虚拟空间已经被释放)


```C++
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

void *th_fun2(void *arg)
{
    printf("子子线程: %ld\n", pthread_self());
    sleep(1);
    for (int i = 0; i < 10; ++i)
        printf("%d_", i);
    return NULL;
}

void *th_fun(void *arg)
{
    printf("子线程: %ld\n", pthread_self());
    pthread_t th;
    pthread_create(&th, NULL, th_fun2, NULL);
    return NULL;
}

int main()
{
    pthread_t th;
    pthread_create(&th, NULL, th_fun, NULL);
    pthread_detach(th); // 线程分离
    printf("父线程: %ld\n", pthread_self());
    // 让主线程自己退出即可(不然还是全部资源都给你释放了(子线程无了))
    pthread_exit(NULL);
    return 0;
}
```
上述代码, 主线程使用线程分离`pthread_detach`来防止资源泄漏. 然后就退出了(但不释放虚拟空间); 而子线程与子子线程的关系, 现在如同 上上述 代码.

但是, 它却会: `子线程`等待`子子线程`结束再释放虚拟内存空间

