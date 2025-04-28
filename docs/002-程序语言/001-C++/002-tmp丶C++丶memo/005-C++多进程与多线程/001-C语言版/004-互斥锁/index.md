# 互斥锁
## 互斥锁函数
互斥锁是线程同步最常用的一种方式，通过互斥锁可以锁定一个代码块, 被锁定的这个代码块, 所有的线程只能顺序执行(不能并行处理)，这样多线程访问共享资源数据混乱的问题就可以被解决了，需要付出的代价就是执行效率的降低，因为默认临界区多个线程是可以并行处理的，现在只能串行处理。

在Linux中互斥锁的类型为`pthread_mutex_t`，创建一个这种类型的变量就得到了一把互斥锁:
```C++
pthread_mutex_t  mutex;
```

在创建的锁对象中保存了当前这把锁的状态信息：锁定还是打开，如果是锁定状态还记录了给这把锁加锁的线程信息（线程ID）。一个互斥锁变量只能被一个线程锁定，被锁定之后其他线程再对互斥锁变量加锁就会被阻塞，直到这把互斥锁被解锁，被阻塞的线程才能被解除阻塞。<b style="color:red">一般情况下，每一个共享资源对应一个把互斥锁，锁的个数和线程的个数无关</b>。

### 互斥锁的初始化与释放
> Linux 提供的互斥锁操作函数如下，如果函数调用成功会返回0，调用失败会返回相应的错误号:

```C++
// 初始化互斥锁
// restrict: 是一个关键字, 用来修饰指针, 只有这个关键字修饰的指针可以访问指向的内存地址, 其他指针是不行的
int pthread_mutex_init(pthread_mutex_t *restrict mutex,
           const pthread_mutexattr_t *restrict attr);
// 释放互斥锁资源            
int pthread_mutex_destroy(pthread_mutex_t *mutex);
```

- 参数:
    - `mutex`: 互斥锁变量的地址
    - `attr`: 互斥锁的属性, 一般使用默认属性即可, 这个参数指定为`NULL`

### 上锁
#### pthread_mutex_lock()
```C++
// 修改互斥锁的状态, 将其设定为锁定状态, 这个状态被写入到参数 mutex 中
int pthread_mutex_lock(pthread_mutex_t *mutex);
```

这个函数被调用, 首先会判断参数`mutex`互斥锁中的状态是不是锁定状态:

- 没有被锁定, 是打开的, 这个线程可以加锁成功, 这个这个锁中会记录是哪个线程加锁成功了
- 如果被锁定了, 其他线程加锁就失败了, 这些线程都会**阻塞**在这把锁上
- 当这把锁被解开之后, 这些阻塞在锁上的线程就解除阻塞了，并且这些线程是通过竞争的方式对这把锁加锁，没抢到锁的线程继续阻塞

#### pthread_mutex_trylock()

```C++
// 尝试加锁
int pthread_mutex_trylock(pthread_mutex_t *mutex);
```
调用这个函数对互斥锁变量加锁还是有两种情况:
- 如果这把锁没有被锁定是打开的，线程加锁成功
- 如果锁变量被锁住了，调用这个函数加锁的线程，**不会被阻塞**，加锁失败直接返回错误号

### 解锁
#### pthread_mutex_unlock()

```C++
// 对互斥锁解锁
int pthread_mutex_unlock(pthread_mutex_t *mutex);
```

**不是所有的线程都可以对互斥锁解锁，哪个线程加的锁, 哪个线程才能解锁成功。**

## 互斥锁使用

我们可以将上面多线程交替数数的例子修改一下，使用互斥锁进行线程同步。两个线程一共操作了同一个全局变量，因此需要添加一互斥锁，来控制这两个线程。
```C++
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <pthread.h>

#define MAX 50
// 全局变量
int number;
pthread_mutex_t mutex; // 互斥锁

// 线程处理函数
void* funcA_num(void* arg)
{
    for(int i=0; i<MAX; ++i)
    {
        pthread_mutex_lock(&mutex);
        int cur = number;
        cur++;
        usleep(10); // 这里之所以它在里面, 是因为原本就是这样设计的, (当然你放在外面也没问题(只是它本来就是这个顺序的, 而已(不过这样确实不好)))
        number = cur;
        printf("Thread A, id = %lu, number = %d\n", pthread_self(), number);
        pthread_mutex_unlock(&mutex);
    }

    return NULL;
}

void* funcB_num(void* arg)
{
    for(int i=0; i<MAX; ++i)
    {
        pthread_mutex_lock(&mutex);
        int cur = number;
        cur++;
        number = cur;
        printf("Thread B, id = %lu, number = %d\n", pthread_self(), number);
        pthread_mutex_unlock(&mutex); // 注意只包裹共享数据(全局变量number有关的)
        usleep(5); // 没有必要让usleep参与
    }

    return NULL;
}

int main(int argc, const char* argv[])
{
    pthread_mutex_init(&mutex, NULL); // 初始化锁
    pthread_t p1, p2;

    // 创建两个子线程
    pthread_create(&p1, NULL, funcA_num, NULL);
    pthread_create(&p2, NULL, funcB_num, NULL);

    // 阻塞，资源回收
    pthread_join(p1, NULL);
    pthread_join(p2, NULL);

    pthread_mutex_destroy(&mutex); // 释放锁
    return 0;
}
```

## 参考链接
### [1]
[爱编程的大丙-互斥锁](https://subingwen.cn/linux/thread-sync/#1-1-%E4%B8%BA%E4%BB%80%E4%B9%88%E8%A6%81%E5%90%8C%E6%AD%A5)