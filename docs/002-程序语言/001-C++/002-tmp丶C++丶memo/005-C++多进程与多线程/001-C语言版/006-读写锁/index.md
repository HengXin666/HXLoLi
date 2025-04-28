# 读写锁
## 区别
读写锁是互斥锁的升级版, <span style="color:red">在做读操作的时候可以提高程序的执行效率，如果所有的线程都是做读操作, 那么读是并行的</span>，但是使用互斥锁，读操作也是串行的。

## 读写锁函数

读写锁是 $一把锁$，锁的类型为`pthread_rwlock_t`，有了类型之后就可以创建一把互斥锁了: (*注意是一把锁, 但是有两个功能; 而不是两把锁!!!*)

```C++
pthread_rwlock_t rwlock;
```

之所以称其为读写锁，是因为这把锁既可以锁定读操作，也可以锁定写操作。为了方便理解，可以大致认为在这把锁中记录了这些信息：

- `锁的状态`: 锁定/打开
- `锁定的是什么操作`: 读操作/写操作，使用读写锁锁定了读操作，需要先解锁才能去锁定写操作，反之亦然。
- `哪个线程将这把锁锁上了`

读写锁的使用方式也互斥锁的使用方式是完全相同的：找共享资源, 确定临界区，在临界区的开始位置加锁（读锁/写锁），临界区的结束位置解锁。

因为通过一把读写锁可以锁定读或者写操作，下面介绍一下关于读写锁的特点：

1. 使用读写锁的读锁锁定了临界区，线程对临界区的访问是并行的，读锁是<span style="color:red">共享的</span>。
2. 使用读写锁的写锁锁定了临界区，线程对临界区的访问是串行的，写锁是<span style="color:red">独占的</span>。
3. **使用读写锁分别对两个临界区加了读锁和写锁，两个线程要同时访问者两个临界区**，访问写锁临界区的线程继续运行，访问`读锁`临界区的线程`阻塞`，因为<span style="color:red">`写锁`比读锁的`优先级高`</span>。

<span style="color:red">如果说程序中所有的线程都对共享资源做写操作，使用读写锁没有优势，和互斥锁是一样的，如果说程序中所有的线程都对共享资源有写也有读操作，并且对共享资源读的操作越多，读写锁更有优势。</span>

### 初始化与释放

> Linux提供的读写锁操作函数原型如下，如果函数调用成功返回0，失败返回对应的错误号：

```C++
#include <pthread.h>
pthread_rwlock_t rwlock;
// 初始化读写锁
int pthread_rwlock_init(pthread_rwlock_t *restrict rwlock,
           const pthread_rwlockattr_t *restrict attr);
// 释放读写锁占用的系统资源
int pthread_rwlock_destroy(pthread_rwlock_t *rwlock);
```

- **参数**:
    - `rwlock`: 读写锁的地址，传出参数
    - `attr`: 读写锁属性，一般使用默认属性，指定为NULL

### 加读锁
#### pthread_rwlock_rdlock()
```C++
// 在程序中对读写锁加读锁, 锁定的是读操作
int pthread_rwlock_rdlock(pthread_rwlock_t *rwlock);
```

调用这个函数，如果读写锁是打开的，那么加锁成功；<span style="color:red">如果读写锁已经锁定了**读操作**，调用这个函数依然可以`加锁成功`，因为读锁是**共享的**</span>；如果读写锁已经锁定了**写操作**，调用这个函数的线程会被**阻塞**。

#### pthread_rwlock_tryrdlock()

```C++
// 这个函数可以有效的避免死锁
// 如果加读锁失败, 不会阻塞当前线程, 直接返回错误号
int pthread_rwlock_tryrdlock(pthread_rwlock_t *rwlock);
```
调用这个函数，如果读写锁是打开的，那么加锁成功；如果读写锁已经锁定了读操作，调用这个函数依然可以加锁成功，因为读锁是共享的；如果读写锁已经锁定了写操作，调用这个函数加锁失败，对应的线程**不会被阻塞**，可以在程序中对函数返回值进行判断，添加加锁失败之后的处理动作。


### 加写锁
#### pthread_rwlock_wrlock()

```C++
// 在程序中对读写锁加写锁, 锁定的是写操作
int pthread_rwlock_wrlock(pthread_rwlock_t *rwlock);
```

调用这个函数，如果读写锁是打开的，那么加锁成功；如果读写锁已经锁定了**读操作或者锁定了写操作**，调用这个函数的线程会被**阻塞**。

#### pthread_rwlock_trywrlock()
```C++
// 这个函数可以有效的避免死锁
// 如果加写锁失败, 不会阻塞当前线程, 直接返回错误号
int pthread_rwlock_trywrlock(pthread_rwlock_t *rwlock);
```

调用这个函数，如果读写锁是打开的，那么加锁成功；如果读写锁已经锁定了读操作或者锁定了写操作，调用这个函数**加锁失败**，但是线程**不会阻塞**，可以在程序中对函数返回值进行判断，添加加锁失败之后的处理动作。

### 解锁
#### pthread_rwlock_unlock()

```C++
// 解锁, 不管锁定了读还是写都可用解锁
int pthread_rwlock_unlock(pthread_rwlock_t *rwlock);
```

## 读写锁使用
题目要求: 8个线程操作同一个全局变量，3个线程不定时写同一全局资源，5个线程不定时读同一全局资源。(因为 读的线程数 > 写的, 使用可以使用`读写锁`)

```C++
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#define MAX 50

pthread_rwlock_t rwlock; // 读写锁
int number;              // 需要访问的变量

void *th_r_fun(void *arg)
{
    // 不定时读
    for (int i = 0; i < MAX; ++i)
    {
        pthread_rwlock_rdlock(&rwlock);
        printf("The r By %ld numbre is %d\n", pthread_self(), number);
        pthread_rwlock_unlock(&rwlock);
        usleep(rand() % 500);
    }
    return NULL;
}

void *th_w_fun(void *arg)
{
    // 不定时写
    for (int i = 0; i < MAX; ++i)
    {
        pthread_rwlock_wrlock(&rwlock);
        int tmp = number;
        ++tmp;
        number = tmp;
        printf("The w By %ld numbre is %d\n", pthread_self(), number);
        pthread_rwlock_unlock(&rwlock);
        usleep(rand() % 600);
    }
    return NULL;
}

int main()
{
    pthread_rwlock_init(&rwlock, NULL); // 初始化读写锁
    pthread_t th_w[3], th_r[5];

    // 启动线程
    for (int i = 0; i < 3; ++i)
    {
        pthread_create(&th_w[i], NULL, th_w_fun, NULL);
        printf("The w th_uid %ld is Go!\n", th_w[i]);
    }

    for (int i = 0; i < 5; ++i)
    {
        pthread_create(&th_r[i], NULL, th_r_fun, NULL);
        printf("The r th_uid %ld is Go!\n", th_r[i]);
    }

    // 等待线程
    for (int i = 0; i < 3; ++i)
    {
        pthread_join(th_w[i], NULL);
    }

    for (int i = 0; i < 5; ++i)
    {
        pthread_join(th_r[i], NULL);
    }

    pthread_rwlock_destroy(&rwlock); // 释放读写锁
    return 0;
}
```

运行:

```Shell
[root@localhost hx_thread]# ./app 
The w th_uid 140488386541312 is Go!
The w By 140488386541312 numbre is 1
The w th_uid 140488378148608 is Go!
The w th_uid 140488369755904 is Go!
The w By 140488378148608 numbre is 2
The w By 140488369755904 numbre is 3
The r th_uid 140488361363200 is Go!
The r th_uid 140488352970496 is Go!
The r By 140488352970496 numbre is 3
The r By 140488361363200 numbre is 3
The r th_uid 140488344577792 is Go!
The w By 140488378148608 numbre is 4
The r th_uid 140488336185088 is Go!
The r By 140488344577792 numbre is 4
The r By 140488327792384 numbre is 4
The r By 140488336185088 numbre is 4
The r By 140488352970496 numbre is 4
The r th_uid 140488327792384 is Go!
The r By 140488361363200 numbre is 4
The r By 140488352970496 numbre is 4
The w By 140488369755904 numbre is 5
The r By 140488327792384 numbre is 5
The r By 140488336185088 numbre is 5
The r By 140488352970496 numbre is 5
The r By 140488344577792 numbre is 5
The w By 140488386541312 numbre is 6
The w By 140488378148608 numbre is 7
The r By 140488327792384 numbre is 7
The r By 140488344577792 numbre is 7
The r By 140488336185088 numbre is 7
The r By 140488361363200 numbre is 7
The r By 140488352970496 numbre is 7
The r By 140488327792384 numbre is 7
The r By 140488336185088 numbre is 7
The r By 140488361363200 numbre is 7
The r By 140488336185088 numbre is 7
The r By 140488352970496 numbre is 7
The r By 140488344577792 numbre is 7
The r By 140488327792384 numbre is 7
The r By 140488336185088 numbre is 7
The r By 140488352970496 numbre is 7
The r By 140488361363200 numbre is 7
The r By 140488344577792 numbre is 7
The r By 140488352970496 numbre is 7
The r By 140488344577792 numbre is 7
The r By 140488336185088 numbre is 7
The r By 140488327792384 numbre is 7
The r By 140488361363200 numbre is 7
The r By 140488344577792 numbre is 7
The r By 140488327792384 numbre is 7
The r By 140488336185088 numbre is 7
The r By 140488361363200 numbre is 7
The r By 140488352970496 numbre is 7
The r By 140488344577792 numbre is 7
The w By 140488378148608 numbre is 8
The r By 140488336185088 numbre is 8
The r By 140488361363200 numbre is 8
The r By 140488327792384 numbre is 8
The r By 140488352970496 numbre is 8
The r By 140488336185088 numbre is 8
The r By 140488344577792 numbre is 8
The w By 140488369755904 numbre is 9
The r By 140488352970496 numbre is 9
The r By 140488361363200 numbre is 9
The r By 140488344577792 numbre is 9
The r By 140488327792384 numbre is 9
The r By 140488336185088 numbre is 9
The w By 140488378148608 numbre is 10
The r By 140488352970496 numbre is 10
The r By 140488361363200 numbre is 10
The r By 140488327792384 numbre is 10
The r By 140488344577792 numbre is 10
The w By 140488386541312 numbre is 11
The r By 140488336185088 numbre is 11
The w By 140488369755904 numbre is 12
The r By 140488361363200 numbre is 12
The r By 140488336185088 numbre is 12
The r By 140488352970496 numbre is 12
The w By 140488378148608 numbre is 13
The r By 140488327792384 numbre is 13
The r By 140488344577792 numbre is 13
The w By 140488369755904 numbre is 14
The r By 140488336185088 numbre is 14
The r By 140488344577792 numbre is 14
The r By 140488361363200 numbre is 14
The r By 140488327792384 numbre is 14
The r By 140488352970496 numbre is 14
The r By 140488336185088 numbre is 14
The w By 140488386541312 numbre is 15
The r By 140488327792384 numbre is 15
The r By 140488344577792 numbre is 15
The r By 140488352970496 numbre is 15
The r By 140488336185088 numbre is 15
The r By 140488361363200 numbre is 15
The w By 140488378148608 numbre is 16
The r By 140488327792384 numbre is 16
The r By 140488352970496 numbre is 16
The r By 140488344577792 numbre is 16
The r By 140488336185088 numbre is 16
The r By 140488361363200 numbre is 16
The w By 140488369755904 numbre is 17
The r By 140488352970496 numbre is 17
The r By 140488327792384 numbre is 17
The w By 140488386541312 numbre is 18
The r By 140488344577792 numbre is 18
The r By 140488361363200 numbre is 18
The w By 140488369755904 numbre is 19
The r By 140488352970496 numbre is 19
The r By 140488336185088 numbre is 19
The w By 140488378148608 numbre is 20
The r By 140488327792384 numbre is 20
The r By 140488336185088 numbre is 20
The r By 140488344577792 numbre is 20
The r By 140488361363200 numbre is 20
The r By 140488352970496 numbre is 20
The w By 140488386541312 numbre is 21
The r By 140488327792384 numbre is 21
The r By 140488361363200 numbre is 21
The r By 140488352970496 numbre is 21
The r By 140488336185088 numbre is 21
The r By 140488344577792 numbre is 21
The r By 140488361363200 numbre is 21
The w By 140488369755904 numbre is 22
The r By 140488336185088 numbre is 22
The r By 140488352970496 numbre is 22
The r By 140488327792384 numbre is 22
The r By 140488344577792 numbre is 22
The r By 140488361363200 numbre is 22
The w By 140488378148608 numbre is 23
The r By 140488327792384 numbre is 23
The r By 140488336185088 numbre is 23
The r By 140488352970496 numbre is 23
The r By 140488361363200 numbre is 23
The r By 140488344577792 numbre is 23
The w By 140488386541312 numbre is 24
The r By 140488336185088 numbre is 24
The r By 140488361363200 numbre is 24
The r By 140488352970496 numbre is 24
The r By 140488327792384 numbre is 24
The w By 140488369755904 numbre is 25
The r By 140488344577792 numbre is 25
The r By 140488327792384 numbre is 25
The w By 140488378148608 numbre is 26
The r By 140488352970496 numbre is 26
The r By 140488361363200 numbre is 26
The w By 140488386541312 numbre is 27
The r By 140488336185088 numbre is 27
The w By 140488378148608 numbre is 28
The r By 140488361363200 numbre is 28
The r By 140488327792384 numbre is 28
The r By 140488344577792 numbre is 28
The w By 140488386541312 numbre is 29
The r By 140488327792384 numbre is 29
The r By 140488336185088 numbre is 29
The r By 140488352970496 numbre is 29
The r By 140488344577792 numbre is 29
The r By 140488361363200 numbre is 29
The w By 140488369755904 numbre is 30
The r By 140488336185088 numbre is 30
The r By 140488352970496 numbre is 30
The r By 140488361363200 numbre is 30
The r By 140488327792384 numbre is 30
The w By 140488386541312 numbre is 31
The r By 140488352970496 numbre is 31
The r By 140488336185088 numbre is 31
The r By 140488344577792 numbre is 31
The r By 140488327792384 numbre is 31
The r By 140488361363200 numbre is 31
The w By 140488378148608 numbre is 32
The r By 140488352970496 numbre is 32
The r By 140488361363200 numbre is 32
The r By 140488336185088 numbre is 32
The w By 140488386541312 numbre is 33
The r By 140488327792384 numbre is 33
The r By 140488344577792 numbre is 33
The w By 140488369755904 numbre is 34
The r By 140488336185088 numbre is 34
The r By 140488352970496 numbre is 34
The r By 140488361363200 numbre is 34
The r By 140488336185088 numbre is 34
The r By 140488344577792 numbre is 34
The r By 140488361363200 numbre is 34
The r By 140488352970496 numbre is 34
The r By 140488327792384 numbre is 34
The w By 140488378148608 numbre is 35
The r By 140488336185088 numbre is 35
The r By 140488361363200 numbre is 35
The r By 140488352970496 numbre is 35
The r By 140488336185088 numbre is 35
The r By 140488361363200 numbre is 35
The r By 140488327792384 numbre is 35
The r By 140488336185088 numbre is 35
The r By 140488344577792 numbre is 35
The w By 140488386541312 numbre is 36
The w By 140488369755904 numbre is 37
The r By 140488352970496 numbre is 37
The r By 140488336185088 numbre is 37
The r By 140488344577792 numbre is 37
The r By 140488361363200 numbre is 37
The w By 140488378148608 numbre is 38
The w By 140488369755904 numbre is 39
The r By 140488327792384 numbre is 39
The r By 140488361363200 numbre is 39
The w By 140488386541312 numbre is 40
The r By 140488336185088 numbre is 40
The r By 140488327792384 numbre is 40
The r By 140488352970496 numbre is 40
The r By 140488344577792 numbre is 40
The r By 140488361363200 numbre is 40
The w By 140488378148608 numbre is 41
The w By 140488369755904 numbre is 42
The r By 140488344577792 numbre is 42
The r By 140488327792384 numbre is 42
The r By 140488352970496 numbre is 42
The r By 140488361363200 numbre is 42
The r By 140488327792384 numbre is 42
The r By 140488352970496 numbre is 42
The r By 140488336185088 numbre is 42
The r By 140488361363200 numbre is 42
The r By 140488344577792 numbre is 42
The w By 140488378148608 numbre is 43
The r By 140488352970496 numbre is 43
The r By 140488336185088 numbre is 43
The r By 140488344577792 numbre is 43
The r By 140488361363200 numbre is 43
The r By 140488327792384 numbre is 43
The w By 140488386541312 numbre is 44
The w By 140488369755904 numbre is 45
The r By 140488352970496 numbre is 45
The w By 140488378148608 numbre is 46
The r By 140488327792384 numbre is 46
The r By 140488336185088 numbre is 46
The r By 140488361363200 numbre is 46
The r By 140488344577792 numbre is 46
The w By 140488386541312 numbre is 47
The r By 140488352970496 numbre is 47
The r By 140488361363200 numbre is 47
The r By 140488336185088 numbre is 47
The r By 140488344577792 numbre is 47
The r By 140488327792384 numbre is 47
The w By 140488378148608 numbre is 48
The r By 140488361363200 numbre is 48
The r By 140488327792384 numbre is 48
The w By 140488386541312 numbre is 49
The r By 140488352970496 numbre is 49
The r By 140488344577792 numbre is 49
The r By 140488336185088 numbre is 49
The w By 140488369755904 numbre is 50
The r By 140488327792384 numbre is 50
The r By 140488352970496 numbre is 50
The w By 140488378148608 numbre is 51
The r By 140488361363200 numbre is 51
The r By 140488336185088 numbre is 51
The r By 140488327792384 numbre is 51
The r By 140488344577792 numbre is 51
The w By 140488386541312 numbre is 52
The r By 140488327792384 numbre is 52
The r By 140488336185088 numbre is 52
The r By 140488352970496 numbre is 52
The r By 140488361363200 numbre is 52
The w By 140488378148608 numbre is 53
The w By 140488369755904 numbre is 54
The r By 140488344577792 numbre is 54
The r By 140488336185088 numbre is 54
The r By 140488327792384 numbre is 54
The r By 140488352970496 numbre is 54
The w By 140488378148608 numbre is 55
The r By 140488361363200 numbre is 55
The w By 140488386541312 numbre is 56
The w By 140488369755904 numbre is 57
The r By 140488336185088 numbre is 57
The r By 140488327792384 numbre is 57
The r By 140488344577792 numbre is 57
The r By 140488361363200 numbre is 57
The r By 140488352970496 numbre is 57
The w By 140488378148608 numbre is 58
The r By 140488336185088 numbre is 58
The w By 140488386541312 numbre is 59
The r By 140488361363200 numbre is 59
The r By 140488344577792 numbre is 59
The r By 140488327792384 numbre is 59
The r By 140488336185088 numbre is 59
The w By 140488369755904 numbre is 60
The r By 140488361363200 numbre is 60
The r By 140488344577792 numbre is 60
The r By 140488352970496 numbre is 60
The w By 140488386541312 numbre is 61
The r By 140488344577792 numbre is 61
The r By 140488327792384 numbre is 61
The w By 140488378148608 numbre is 62
The r By 140488361363200 numbre is 62
The r By 140488336185088 numbre is 62
The r By 140488344577792 numbre is 62
The r By 140488352970496 numbre is 62
The w By 140488369755904 numbre is 63
The r By 140488327792384 numbre is 63
The r By 140488344577792 numbre is 63
The r By 140488352970496 numbre is 63
The r By 140488361363200 numbre is 63
The r By 140488336185088 numbre is 63
The w By 140488386541312 numbre is 64
The r By 140488344577792 numbre is 64
The r By 140488327792384 numbre is 64
The w By 140488369755904 numbre is 65
The r By 140488344577792 numbre is 65
The r By 140488352970496 numbre is 65
The r By 140488336185088 numbre is 65
The r By 140488361363200 numbre is 65
The r By 140488327792384 numbre is 65
The r By 140488344577792 numbre is 65
The w By 140488378148608 numbre is 66
The w By 140488386541312 numbre is 67
The w By 140488369755904 numbre is 68
The r By 140488327792384 numbre is 68
The r By 140488352970496 numbre is 68
The r By 140488344577792 numbre is 68
The r By 140488361363200 numbre is 68
The r By 140488336185088 numbre is 68
The r By 140488327792384 numbre is 68
The w By 140488369755904 numbre is 69
The w By 140488386541312 numbre is 70
The r By 140488344577792 numbre is 70
The w By 140488378148608 numbre is 71
The r By 140488352970496 numbre is 71
The w By 140488386541312 numbre is 72
The r By 140488344577792 numbre is 72
The r By 140488352970496 numbre is 72
The r By 140488327792384 numbre is 72
The w By 140488369755904 numbre is 73
The r By 140488344577792 numbre is 73
The w By 140488386541312 numbre is 74
The r By 140488327792384 numbre is 74
The w By 140488369755904 numbre is 75
The w By 140488378148608 numbre is 76
The w By 140488378148608 numbre is 77
The w By 140488369755904 numbre is 78
The w By 140488386541312 numbre is 79
The w By 140488369755904 numbre is 80
The w By 140488369755904 numbre is 81
The w By 140488378148608 numbre is 82
The w By 140488386541312 numbre is 83
The w By 140488369755904 numbre is 84
The w By 140488386541312 numbre is 85
The w By 140488378148608 numbre is 86
The w By 140488369755904 numbre is 87
The w By 140488378148608 numbre is 88
The w By 140488386541312 numbre is 89
The w By 140488378148608 numbre is 90
The w By 140488369755904 numbre is 91
The w By 140488386541312 numbre is 92
The w By 140488378148608 numbre is 93
The w By 140488369755904 numbre is 94
The w By 140488378148608 numbre is 95
The w By 140488386541312 numbre is 96
The w By 140488369755904 numbre is 97
The w By 140488369755904 numbre is 98
The w By 140488378148608 numbre is 99
The w By 140488386541312 numbre is 100
The w By 140488369755904 numbre is 101
The w By 140488378148608 numbre is 102
The w By 140488378148608 numbre is 103
The w By 140488386541312 numbre is 104
The w By 140488369755904 numbre is 105
The w By 140488378148608 numbre is 106
The w By 140488369755904 numbre is 107
The w By 140488386541312 numbre is 108
The w By 140488378148608 numbre is 109
The w By 140488369755904 numbre is 110
The w By 140488378148608 numbre is 111
The w By 140488369755904 numbre is 112
The w By 140488386541312 numbre is 113
The w By 140488369755904 numbre is 114
The w By 140488386541312 numbre is 115
The w By 140488378148608 numbre is 116
The w By 140488386541312 numbre is 117
The w By 140488369755904 numbre is 118
The w By 140488378148608 numbre is 119
The w By 140488369755904 numbre is 120
The w By 140488386541312 numbre is 121
The w By 140488378148608 numbre is 122
The w By 140488386541312 numbre is 123
The w By 140488369755904 numbre is 124
The w By 140488378148608 numbre is 125
The w By 140488386541312 numbre is 126
The w By 140488386541312 numbre is 127
The w By 140488378148608 numbre is 128
The w By 140488369755904 numbre is 129
The w By 140488378148608 numbre is 130
The w By 140488378148608 numbre is 131
The w By 140488386541312 numbre is 132
The w By 140488369755904 numbre is 133
The w By 140488378148608 numbre is 134
The w By 140488369755904 numbre is 135
The w By 140488386541312 numbre is 136
The w By 140488378148608 numbre is 137
The w By 140488378148608 numbre is 138
The w By 140488369755904 numbre is 139
The w By 140488378148608 numbre is 140
The w By 140488386541312 numbre is 141
The w By 140488386541312 numbre is 142
The w By 140488369755904 numbre is 143
The w By 140488386541312 numbre is 144
The w By 140488369755904 numbre is 145
The w By 140488386541312 numbre is 146
The w By 140488369755904 numbre is 147
The w By 140488386541312 numbre is 148
The w By 140488386541312 numbre is 149
The w By 140488386541312 numbre is 150
```

## 参考链接
[爱编程的大丙-读写锁](https://subingwen.cn/linux/thread-sync/#1-1-%E4%B8%BA%E4%BB%80%E4%B9%88%E8%A6%81%E5%90%8C%E6%AD%A5)