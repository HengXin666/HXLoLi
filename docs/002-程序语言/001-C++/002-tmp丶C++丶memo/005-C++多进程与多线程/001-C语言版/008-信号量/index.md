# 信号量
## 信号量函数
信号量用在多线程多任务同步的，一个线程完成了某一个动作就通过信号量告诉别的线程，别的线程再进行某些动作。信号量不一定是锁定某一个资源，而是流程上的概念，比如：有A，B两个线程，B线程要等A线程完成某一任务以后再进行自己下面的步骤，这个任务并不一定是锁定某一资源，还可以是进行一些计算或者数据处理之类。

`信号量（信号灯）`与互斥锁和条件变量的主要不同在于”灯”的概念，灯亮则意味着资源可用，灯灭则意味着不可用。信号量主要阻塞线程, 不能完全保证线程安全，如果要保证线程安全, 需要信号量和互斥锁一起使用。

### 类型声明
信号量和条件变量一样用于处理生产者和消费者模型，用于阻塞生产者线程或者消费者线程的运行。信号的类型为`sem_t`对应的头文件为`<semaphore.h>`:

```C++
#include <semaphore.h>
sem_t sem;
```

### 初始化与释放
> Linux提供的信号量操作函数原型如下:

```C++
#include <semaphore.h>
// 初始化信号量/信号灯
int sem_init(sem_t *sem, int pshared, unsigned int value);
// 资源释放, 线程销毁之后调用这个函数即可
// 参数 sem 就是 sem_init() 的第一个参数            
int sem_destroy(sem_t *sem);
```

- **参数**:
    - `sem`: 信号量变量地址
    - `pshared`:
        - `0`: 线程同步
        - `非0`: 进程同步
    - `value`: 初始化当前信号量拥有的资源数（>=0），如果资源数为0，线程就会被阻塞了。
 
### 消耗一个信号量
#### sem_wait()

```C++
// 参数 sem 就是 sem_init() 的第一个参数  
// 函数被调用sem中的资源就会被消耗1个, 资源数-1
int sem_wait(sem_t *sem);
```
当线程调用这个函数，并且`sem`中的**资源数>0**，线程**不会阻塞**，线程会占用`sem`中的一个资源，因此**资源数-1**，直到`sem`中的资源数**减为0**时，资源被**耗尽**，因此线程也就被**阻塞**了。

#### sem_trywait()

```C++
// 参数 sem 就是 sem_init() 的第一个参数  
// 函数被调用sem中的资源就会被消耗1个, 资源数-1
int sem_trywait(sem_t *sem);
```
当线程调用这个函数，并且`sem`中的**资源数>0**，线程**不会阻塞**，线程会占用`sem`中的一个资源，因此**资源数-1**，直到sem中的资源数**减为0**时，资源被**耗尽**，**但是线程不会被阻塞，直接返回错误号**，*因此可以在程序中添加判断分支，用于处理获取资源失败之后的情况*。

#### sem_timedwait()

```C++
// 表示的时间是从1971.1.1到某个时间点的时间, 总长度使用秒/纳秒表示
struct timespec {
    time_t tv_sec;      /* Seconds */
    long   tv_nsec;     /* Nanoseconds [0 .. 999999999] */
};
// 调用该函数线程获取sem中的一个资源，当资源数为0时，线程阻塞，在阻塞abs_timeout对应的时长之后，解除阻塞。
// abs_timeout: 阻塞的时间长度, 单位是s, 是从1970.1.1开始计算的
int sem_timedwait(sem_t *sem, const struct timespec *abs_timeout);
```
该函数的参数`abs_timeout`和`pthread_cond_timedwait`的最后一个参数是一样的，使用方法不再过多赘述。当线程调用这个函数，并且`sem`中的**资源数>0**，线程**不会阻塞**，线程会占用`sem`中的一个资源，因此**资源数-1**，直到`sem`中的资源数**减为0**时，资源被**耗尽**，线程被**阻塞**，**当阻塞指定的时长之后，线程解除阻塞**。

### 资源数加一
#### sem_post()

```C++
// 调用该函数给sem中的资源数+1
int sem_post(sem_t *sem);
```

调用该函数会将`sem`中的资源数+1，如果有线程在调用`sem_wait`、`sem_trywait`、`sem_timedwait`时因为`sem`中的资源数为0被阻塞了，这时这些线程会**解除阻塞**，获取到资源之后继续向下运行。

### 查看资源数
#### sem_getvalue()

```C++
// 查看信号量 sem 中的整形数的当前值, 这个值会被写入到sval指针对应的内存中
// sval是一个传出参数
int sem_getvalue(sem_t *sem, int *sval);
```

通过这个函数可以查看`sem`中现在拥有的资源个数，通过第二个参数`sval`将数据传出，也就是说**第二个参数的作用和返回值是一样的**。

## 生产者和消费者

由于生产者和消费者是两类线程，并且在还没有生成之前是不能进行消费的，在使用信号量处理这类问题的时候可以定义两个信号量，分别用于记录生产者和消费者线程拥有的总资源数。

```C++
// 生产者线程 
sem_t psem;
// 消费者线程
sem_t csem;

// 信号量初始化
sem_init(&psem, 0, 5);    // 5个生产者可以同时生产
sem_init(&csem, 0, 0);    // 消费者线程没有资源, 因此不能消费

// 生产者线程
// 在生产之前, 从信号量中取出一个资源
sem_wait(&psem);    
// 生产者商品代码, 有商品了, 放到任务队列
......     
......
......
// 通知消费者消费，给消费者信号量添加资源，让消费者解除阻塞
sem_post(&csem);
    
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

// 消费者线程
// 消费者需要等待生产, 默认启动之后应该阻塞
sem_wait(&csem);
// 开始消费
......
......
......
// 消费完成, 通过生产者生产，给生产者信号量添加资源
sem_post(&psem);
```
通过上面的代码可以知道，初始化信号量的时候没有消费者分配资源，消费者线程启动之后由于没有资源自然就被阻塞了，等生产者生产出产品之后，再给消费者分配资源，这样二者就可以配合着完成生产和消费流程了。

## 信号量使用
> 场景描述：使用信号量实现生产者和消费者模型，生产者有5个，往链表头部添加节点，消费者也有5个，删除链表头部的节点。

### 总资源数为1
如果生产者和消费者线程使用的信号量对应的总资源数为1，那么不管线程有多少个，可以工作的线程只有一个，其余线程由于拿不到资源，都被迫阻塞了。(所以可以不用互斥锁)

```C++
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

typedef struct _node
{
    int number;
    struct _node *next;
} Node;

// 定义互斥锁与信号量
pthread_mutex_t mutrx;
sem_t sem_do; // 生产者的信号量
sem_t sem_fk; // 消费者的信号量

// 定义一个头插链表
Node *head = NULL;

void *th_do_fun(void *arg)
{
    // 生产者
    while (1)
    {
        sem_wait(&sem_do);
        Node *p = (Node *)malloc(sizeof(Node));
        p->number = rand() % 100;
        p->next = head;
        head = p;
        printf("[%ld]: do [%d] OK!\n", pthread_self(), head->number);
        sem_post(&sem_fk);
        sleep(rand() % 3);
    }

    return NULL;
}

void *th_fk_fun(void *arg)
{
    while (1)
    {
        sem_wait(&sem_fk);  
        Node *p = head;
        head = head->next;
        printf("[%ld]: fk [%d] OK!\n", pthread_self(), p->number);
        free(p);
        sem_post(&sem_do);
        sleep(rand() % 2);
    }
    
    return NULL;
}

int main()
{
    // 初始化
    pthread_mutex_init(&mutrx, NULL);
    sem_init(&sem_do, 0, 1);
    sem_init(&sem_fk, 0, 0); // 消费者一开始没有生产, 所以不能消费

    // 定义生产者(do)与消费者(fk)线程
    pthread_t do_arr[5], fk_arr[5];
    for (int i = 0; i < 5; ++i)
    {
        pthread_create(&do_arr[i], NULL, th_do_fun, NULL);
        printf("The do Man (%ld) Go !\n", do_arr[i]);
    }

    for (int i = 0; i < 5; ++i)
    {
        pthread_create(&fk_arr[i], NULL, th_fk_fun, NULL);
        printf("The fk Man (%ld) Go !\n", fk_arr[i]);
    }

    for (int i = 0; i < 5; ++i)
    {
        pthread_join(do_arr[i], NULL);
        pthread_join(fk_arr[i], NULL);
    }

    // 释放
    while (head)
    {
        Node *tmp = head;
        head = head->next;
        free(tmp);
    }
  
    pthread_mutex_destroy(&mutrx);
    sem_destroy(&sem_do);
    sem_destroy(&sem_fk);
    return 0;
}
```

**通过测试代码可以得到如下结论：如果生产者和消费者使用的信号量总资源数为1，那么不会出现生产者线程和消费者线程同时访问共享资源的情况，不管生产者和消费者线程有多少个，它们都是顺序执行的。**

### 总资源数大于1

如果生产者和消费者线程使用的信号量对应的**总资源数为大于1**，这种场景下出现的情况就比较多了：

- 多个生产者线程同时生产
- 多个消费者同时消费
- 生产者线程和消费者线程同时生产和消费

以上不管哪一种情况都可能会出现多个线程访问共享资源的情况，如果想防止共享资源出现数据混乱，那么就需要使用互斥锁进行线程同步，处理代码如下:

```C++
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

typedef struct _node
{
    int number;
    struct _node *next;
} Node;

// 定义互斥锁与信号量
pthread_mutex_t mutrx;
sem_t sem_do; // 生产者的信号量
sem_t sem_fk; // 消费者的信号量

// 定义一个头插链表
Node *head = NULL;

void *th_do_fun(void *arg)
{
    // 生产者
    while (1)
    {
        sem_wait(&sem_do);
        pthread_mutex_lock(&mutrx);
        Node *p = (Node *)malloc(sizeof(Node));
        p->number = rand() % 100;
        p->next = head;
        head = p;
        printf("[%ld]: do [%d] OK!\n", pthread_self(), head->number);
        pthread_mutex_unlock(&mutrx);
        sem_post(&sem_fk);
        sleep(rand() % 3);
    }

    return NULL;
}

void *th_fk_fun(void *arg)
{
    while (1)
    {
        sem_wait(&sem_fk);
        pthread_mutex_lock(&mutrx);
        Node *p = head;
        printf("[%ld]: fk [%d] OK!\n", pthread_self(), head->number);
        head = p->next;
        free(p);
        pthread_mutex_unlock(&mutrx);
        sem_post(&sem_do);
        sleep(rand() % 3);
    }
    
    return NULL;
}

int main()
{
    // 初始化
    pthread_mutex_init(&mutrx, NULL);
    sem_init(&sem_do, 0, 5);
    sem_init(&sem_fk, 0, 0); // 消费者一开始没有生产, 所以不能消费

    // 定义生产者(do)与消费者(fk)线程
    pthread_t do_arr[5], fk_arr[5];
    for (int i = 0; i < 5; ++i)
    {
        pthread_create(&do_arr[i], NULL, th_do_fun, NULL);
        printf("The do Man (%ld) Go !\n", do_arr[i]);
    }

    for (int i = 0; i < 5; ++i)
    {
        pthread_create(&fk_arr[i], NULL, th_fk_fun, NULL);
        printf("The fk Man (%ld) Go !\n", fk_arr[i]);
    }

    for (int i = 0; i < 5; ++i)
    {
        pthread_join(do_arr[i], NULL);
        pthread_join(fk_arr[i], NULL);
    }

    // 释放
    while (head)
    {
        Node *tmp = head;
        head = head->next;
        free(tmp);
    }

    pthread_mutex_destroy(&mutrx);
    sem_destroy(&sem_do);
    sem_destroy(&sem_fk);
    return 0;
}
```
#### 注意顺序, 小心死锁
在编写上述代码的时候还有一个需要注意是事项，不管是消费者线程的处理函数还是生产者线程的处理函数内部有这么两行代码:

```C++
// 生产者
sem_wait(&sem_do);
pthread_mutex_lock(&mutrx);

// 消费者
sem_wait(&sem_fk);
pthread_mutex_lock(&mutrx);
```

这两行代码的调用顺序是不能颠倒的，如果颠倒过来就有可能会造成死锁，下面来分析一种死锁的场景: (就是顺序调换的啦)
```C++
// 生产者
pthread_mutex_lock(&mutrx);
sem_wait(&sem_do);

// 消费者
pthread_mutex_lock(&mutrx);
sem_wait(&sem_fk);
```

那么会可能会发生这样的情况:

消费者 占用 线程 (`mutrx`锁上), 然后执行到 `sem_wait(&sem_do);` 又没有资源数, 即在`sem_wait(&sem_do);`处挂起(阻塞), 以希望生产者搞生产先; 但是又因为`mutrx`互斥锁没有解除. 所以其他线程也无法访问共享资源, 故**死锁**了!

## 参考文献
[爱编程的大丙-信号量](https://subingwen.cn/linux/thread-sync/)