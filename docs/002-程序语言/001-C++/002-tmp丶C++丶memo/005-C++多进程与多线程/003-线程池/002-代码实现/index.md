# 线程池代码实现
## 代码 版本V1.0.1 STL + 函数式

头文件
```C++
#ifndef _HX_POOL_H_
#define _HX_POOL_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <queue>
#include <vector>

#define REDUCE_THREAD_NUMER 2   // 一次 减少线程数

// 任务结构体
typedef struct
{
    void *(*fun)(void *arg);    // 任务函数指针
    void *arg;                  // 该函数的参数包
} Task;

// 线程池结构体
typedef struct
{
    // --- 任务队列及管理与使用 ---
    std::queue<Task> TP_task;           // 定义任务队列
    pthread_t TP_op;                    // 管理者线程
    std::vector<pthread_t> TP_consumer; // 消费者线程列表
    int TP_exitNum;                     // 需要销毁的线程个数

    // --- 线程安全(线程同步) ---
    pthread_mutex_t TP_m_all;           // 整个线程池的互斥锁
    pthread_mutex_t TP_m_busy;          // 繁忙线程的互斥锁 (因为任务可能会频繁加减, 线程也会切换任务/执行新任务)
    pthread_cond_t TP_c_taskFull;       // 任务队列满了 (不能再加了)
    pthread_cond_t TP_c_taskEmpty;      // 任务队列空了 (看是否需要减少空闲线程)

    // --- 线程池信息 ---
    int TP_taskMaxSize;                 // 最大任务数
    int TP_max;                         // 最大线程数
    int TP_min;                         // 最小线程数 (初始线程数)
    int TP_busy;                        // 繁忙线程数 (正在执行任务)
    int TP_idle;                        // 空闲线程数 (已挂起)
    int TP_live;                        // 存活线程数 == 空闲线程数 + 繁忙线程数

    // --- 线程池开关 ---
    bool TP_free;                       // 是否释放线程池 (1是, 0否)
} ThreadPool;

/**
 * @brief 创建线程池并初始化
 * @param TP_min 最小线程数
 * @param TP_max 最大线程数
 * @return 线程池指针
 */
ThreadPool *initThreadPool(int TP_min, int TP_max, int TP_taskMaxSize);

// 销毁线程池
int freeThreadPool(ThreadPool *pool);

/**
 * @brief 给线程池添加任务
 * @param pool 线程池指针
 * @param fun 任务函数指针
 * @param arg 参数包指针
 * @return 1 是成功; -1 是失败
 */
int addTask(ThreadPool *pool, void *(*fun)(void *arg), void *arg);

// 获取线程池中工作的线程的个数
int getTP_busyThreadNumer(ThreadPool *pool);

// 获取线程池中活着的线程的个数
int getTP_liveThreadNumer(ThreadPool *pool);

//////////////////////
// 工作的线程(消费者线程)任务函数
void *consumerThreadFun(void *arg);

// 管理者线程 任务函数
void *OpThreadFun(void *arg);

// 单个线程退出
//

#endif // _HX_POOL_H_
```

实现
```C++
#include "hx_pool.h"

ThreadPool *initThreadPool(int TP_min, int TP_max, int TP_taskMaxSize)
{
    ThreadPool *pool = (ThreadPool *)malloc(sizeof(ThreadPool));
    if (!pool)
    {
        printf("ERROR: Malloc error!\n");
        return NULL;
    }

    do
    {
        // 初始化线程同步变量
        if ( pthread_mutex_init(&pool->TP_m_all, NULL) < 0      ||
             pthread_mutex_init(&pool->TP_m_busy, NULL) < 0     ||
             pthread_cond_init(&pool->TP_c_taskEmpty, NULL) < 0 ||
             pthread_cond_init(&pool->TP_c_taskFull, NULL) < 0 )
        {
            printf("ERROR: init error!\n");
            break;
        }

        printf("[INFO]: init 同步变量 OK!\n");

        // 初始化线程信息
        pool->TP_task = std::queue<Task>();
        pool->TP_exitNum = 0;
        pool->TP_max = TP_max;
        pool->TP_min = TP_min;
        pool->TP_taskMaxSize = TP_taskMaxSize;
        pool->TP_free = 0;
        pool->TP_live = pool->TP_idle = pool->TP_min;
        pool->TP_busy = 0;
        std::vector<pthread_t>(TP_max, 0).swap(pool->TP_consumer);

        printf("[INFO]: init 线程信息 OK!\n");

        // 启动生产者(管理者)与消费者
        pthread_create(&pool->TP_op, NULL, OpThreadFun, pool);

        printf("[INFO]: start OP Thread OK!\n");

        for (int i = 0; i < pool->TP_live; ++i)
        {
            pthread_create(&pool->TP_consumer[i], NULL, consumerThreadFun, pool);
        }

        printf("[INFO]: start consumer Thread OK!\n");

        return pool;
    } while (0);

    free(pool);
    return NULL;
}

void *consumerThreadFun(void *arg)
{
    ThreadPool *pool = (ThreadPool *)arg;

    printf("[INFO]: The Thread uid = %ld, Start!\n", pthread_self());

    while (!pool->TP_free)
    {
        pthread_mutex_lock(&pool->TP_m_all);
        // 判断是否有任务
        while (pool->TP_task.empty() && !pool->TP_free && !pool->TP_exitNum)
        {
            // 没有任务则挂起
            printf("[INFO]: uid = %ld, 没有任务睡大觉!\n", pthread_self());
            pthread_cond_wait(&pool->TP_c_taskEmpty, &pool->TP_m_all);
        }

        // 销毁线程 (自杀)
        if (pool->TP_exitNum > 0 || pool->TP_free)
        {
            --pool->TP_exitNum;
            for (auto& it : pool->TP_consumer)
            {
                if (it == pthread_self())
                {
                    it = 0;
                    break;
                }
            }
            --pool->TP_idle;
            --pool->TP_live;
            pthread_mutex_unlock(&pool->TP_m_all);
            printf("[INFO]: The Thread tid: %ld Eixt!\n", pthread_self());
            pthread_exit(NULL);
        }

        // 接任务
        Task task = pool->TP_task.front();
        pool->TP_task.pop();
        ++pool->TP_busy;
        --pool->TP_idle;
        pthread_mutex_unlock(&pool->TP_m_all);
        printf("[INFO]: The uid = %ld, Start Do Task!\n", pthread_self());

        // 做任务
        task.fun(task.arg);

        // 做完了
        pthread_mutex_lock(&pool->TP_m_all);
        --pool->TP_busy;
        ++pool->TP_idle;
        pthread_cond_signal(&pool->TP_c_taskFull);
        pthread_mutex_unlock(&pool->TP_m_all);
    }

    pthread_exit(NULL);
}

void *OpThreadFun(void *arg)
{
    ThreadPool *pool = (ThreadPool *)arg;
    while (!pool->TP_free || pool->TP_live > 0)
    {
        sleep(3);   // 每间隔3秒检查一下线程
        printf("[INFO]: 检查消费者线程中...\n");


        // 安全的获取数据
        pthread_mutex_lock(&pool->TP_m_all);
        int queueLen = pool->TP_task.size();
        int tLive = pool->TP_live;
        int tBusy = pool->TP_busy;
        pthread_mutex_unlock(&pool->TP_m_all); 

        printf("当前任务队列剩有 %d 个任务, 繁忙线程: %d, 存活线程: %d\n", queueLen, tBusy, tLive);

        // 添加线程
        // 任务的个数 > 存活的线程个数 && 存活的线程数 < 最大线程数
        if (queueLen > tLive && tLive < pool->TP_max)
        {
            // 增加
            for (int i = 0; i < REDUCE_THREAD_NUMER; ++i)
            {
                pthread_mutex_lock(&pool->TP_m_all);
                for (auto& it : pool->TP_consumer)
                {
                    if (!it)
                    {
                        pthread_create(&it, NULL, consumerThreadFun, pool);
                        break;
                    }
                }
                ++pool->TP_idle;
                ++pool->TP_live;
                pthread_mutex_unlock(&pool->TP_m_all); 
            }
            printf("[INFO]: 线程过少, 添加线程!\n");
        }

        // 销毁线程
        // 忙的线程 * 2 < 存活的线程数 && 存活的线程 > 最小线程数
        if (tBusy * 2 < tLive && tLive > pool->TP_min || pool->TP_free)
        {
            // 减少
            pthread_mutex_lock(&pool->TP_m_all);
            pool->TP_exitNum = REDUCE_THREAD_NUMER;
            pthread_mutex_unlock(&pool->TP_m_all);

            // 让消费者线程自杀
            for (int i = 0; i < REDUCE_THREAD_NUMER; ++i)
            {
                pthread_cond_signal(&pool->TP_c_taskEmpty);
            }
            printf("[INFO]: 线程过多, 减少线程!\n");
        }
    }
    
    pthread_exit(NULL);
}

int addTask(ThreadPool *pool, void *(*fun)(void *arg), void *arg)
{
    Task task;
    task.fun = fun;
    task.arg = arg;
    pthread_mutex_lock(&pool->TP_m_all);
    do
    {
        if (pool->TP_free)
            break;

        while (pool->TP_task.size() == pool->TP_taskMaxSize)
        {
            // 任务队列已满, 等待添加
            pthread_cond_wait(&pool->TP_c_taskFull, &pool->TP_m_all);
        }

        if (pool->TP_task.size() < pool->TP_taskMaxSize)
        {
            pool->TP_task.push(task);
            pthread_cond_signal(&pool->TP_c_taskEmpty);
        }
        
        pthread_mutex_unlock(&pool->TP_m_all);
        printf("[INFO]: Add Task Ok!\n");
        return 1;
    } while(0);

    pthread_mutex_unlock(&pool->TP_m_all);
    printf("ERROR: Add Task Error!\n");
    return -1;
}

int getTP_busyThreadNumer(ThreadPool *pool)
{
    int res = 0;
    pthread_mutex_lock(&pool->TP_m_busy);
    res = pool->TP_busy;
    pthread_mutex_unlock(&pool->TP_m_busy);
    return res;
}

int getTP_liveThreadNumer(ThreadPool *pool)
{
    int res = 0;
    pthread_mutex_lock(&pool->TP_m_all);
    res = pool->TP_live;
    pthread_mutex_unlock(&pool->TP_m_all);
    return res;
}

int freeThreadPool(ThreadPool *pool)
{
    pthread_mutex_lock(&pool->TP_m_all);
    pool->TP_free = 1;
    pthread_mutex_unlock(&pool->TP_m_all);

    // 阻塞 等待线程退出
    pthread_join(pool->TP_op, NULL);

    pthread_mutex_destroy(&pool->TP_m_all);
    pthread_mutex_destroy(&pool->TP_m_busy);
    pthread_cond_destroy(&pool->TP_c_taskEmpty);
    pthread_cond_destroy(&pool->TP_c_taskFull);

    // 释放资源
    free(pool);

    printf("[INFO]: Free ThreadPool OK!\n");
    return 1;
}
```

使用

```C++
#include <stdio.h>
#include "hx_pool.h"

// 任务函数
void* taskFunc(void* arg)
{
    int num = *(int*)arg;
    printf("thread %ld is working, number = %d\n",
        pthread_self(), num);
    sleep(1);
    free(arg);
    return NULL;
}

int main()
{
    // 创建线程池
    ThreadPool *pool = initThreadPool(2, 10, 20);
    for (int i = 0; i < 100; ++i)
    {
        int *arg = (int *)malloc(sizeof(int));
        *arg = i + 100;
        addTask(pool, taskFunc, arg);
    }

    sleep(15); // 等待线程执行
    printf("释放线程中...\n");
    freeThreadPool(pool);
    return 0;
}
```

运行结果:

```Shell
[root@localhost pool]# g++ hx_pool.cpp main.cpp -l pthread -o app2
[root@localhost pool]# ./app2
[INFO]: init 同步变量 OK!
[INFO]: init 线程信息 OK!
[INFO]: start OP Thread OK!
[INFO]: start consumer Thread OK!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The Thread uid = 140582403958528, Start!
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 100
[INFO]: The Thread uid = 140582395565824, Start!
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 101
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 102
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 103
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582403958528, Start Do Task!
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 105
thread 140582403958528 is working, number = 104
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: 检查消费者线程中...
当前任务队列剩有 20 个任务, 繁忙线程: 2, 存活线程: 2
[INFO]: The Thread uid = 140582387173120, Start!
[INFO]: 线程过少, 添加线程!
[INFO]: The uid = 140582387173120, Start Do Task!
thread 140582387173120 is working, number = 106
[INFO]: The Thread uid = 140582378780416, Start!
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 107
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 108
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 109
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 110
[INFO]: The uid = 140582387173120, Start Do Task!
thread 140582387173120 is working, number = 111
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 112
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 113
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 114
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582387173120, Start Do Task!
thread 140582387173120 is working, number = 115
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 116
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 117
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: 检查消费者线程中...
当前任务队列剩有 20 个任务, 繁忙线程: 4, 存活线程: 4
[INFO]: The Thread uid = 140582370387712, Start!
[INFO]: The Thread uid = 140582361995008, Start!
[INFO]: The uid = 140582361995008, Start Do Task!
thread 140582361995008 is working, number = 118
[INFO]: 线程过少, 添加线程!
[INFO]: The uid = 140582370387712, Start Do Task!
thread 140582370387712 is working, number = 119
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 120
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582387173120, Start Do Task!
thread 140582387173120 is working, number = 121
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 122
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 123
[INFO]: The uid = 140582361995008, Start Do Task!
thread 140582361995008 is working, number = 124
[INFO]: The uid = 140582370387712, Start Do Task!
thread 140582370387712 is working, number = 125
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 126
[INFO]: Add Task Ok!
[INFO]: The uid = 140582387173120, Start Do Task!
thread 140582387173120 is working, number = 127
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 128
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 129
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582361995008, Start Do Task!
thread 140582361995008 is working, number = 130
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582370387712, Start Do Task!
thread 140582370387712 is working, number = 131
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 132
[INFO]: Add Task Ok!
[INFO]: The uid = 140582387173120, Start Do Task!
thread 140582387173120 is working, number = 133
[INFO]: Add Task Ok!
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 135
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 134
[INFO]: 检查消费者线程中...
当前任务队列剩有 20 个任务, 繁忙线程: 6, 存活线程: 6
[INFO]: The Thread uid = 140582353602304, Start!
[INFO]: The uid = 140582353602304, Start Do Task!
thread 140582353602304 is working, number = 136
[INFO]: 线程过少, 添加线程!
[INFO]: The Thread uid = 140582345209600, Start!
[INFO]: The uid = 140582345209600, Start Do Task!
thread 140582345209600 is working, number = 137
[INFO]: The uid = 140582361995008, Start Do Task!
thread 140582361995008 is working, number = 138
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582370387712, Start Do Task!
thread 140582370387712 is working, number = 139
[INFO]: Add Task Ok!
[INFO]: The uid = 140582378780416, Start Do Task!
[INFO]: Add Task Ok!
thread 140582378780416 is working, number = 140
[INFO]: The uid = 140582387173120, Start Do Task!
thread 140582387173120 is working, number = 141
[INFO]: Add Task Ok!
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 142
[INFO]: Add Task Ok!
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 143
[INFO]: Add Task Ok!
[INFO]: The uid = 140582345209600, Start Do Task!
[INFO]: The uid = 140582353602304, Start Do Task!
thread 140582353602304 is working, number = 146
thread 140582345209600 is working, number = 144
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582361995008, Start Do Task!
thread 140582361995008 is working, number = 145
[INFO]: Add Task Ok!
[INFO]: The uid = 140582370387712, Start Do Task!
thread 140582370387712 is working, number = 147
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 148
[INFO]: Add Task Ok!
[INFO]: The uid = 140582387173120, Start Do Task!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
thread 140582387173120 is working, number = 149
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 150
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 151
[INFO]: Add Task Ok!
[INFO]: The uid = 140582353602304, Start Do Task!
thread 140582353602304 is working, number = 152
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582345209600, Start Do Task!
thread 140582345209600 is working, number = 153
[INFO]: The uid = 140582361995008, Start Do Task!
thread 140582361995008 is working, number = 154
[INFO]: The uid = 140582370387712, Start Do Task!
thread 140582370387712 is working, number = 155
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 156
[INFO]: Add Task Ok!
[INFO]: The uid = 140582387173120, Start Do Task!
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 158
thread 140582387173120 is working, number = 157
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 159
[INFO]: 检查消费者线程中...
当前任务队列剩有 20 个任务, 繁忙线程: 8, 存活线程: 8
[INFO]: The Thread uid = 140582131332864, Start!
[INFO]: The Thread uid = 140582122940160, Start!
[INFO]: The uid = 140582122940160, Start Do Task!
thread 140582122940160 is working, number = 161
[INFO]: The uid = 140582131332864, Start Do Task!
[INFO]: 线程过少, 添加线程!
[INFO]: The uid = 140582353602304, Start Do Task!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
thread 140582353602304 is working, number = 162
thread 140582131332864 is working, number = 160
[INFO]: The uid = 140582370387712, Start Do Task!
[INFO]: The uid = 140582361995008, Start Do Task!
thread 140582361995008 is working, number = 165
thread 140582370387712 is working, number = 163
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582345209600, Start Do Task!
thread 140582345209600 is working, number = 164
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 166
[INFO]: Add Task Ok!
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 167
[INFO]: The uid = 140582387173120, Start Do Task!
[INFO]: The uid = 140582395565824, Start Do Task!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
thread 140582395565824 is working, number = 169
thread 140582387173120 is working, number = 168
[INFO]: The uid = 140582122940160, Start Do Task!
thread 140582122940160 is working, number = 170
[INFO]: Add Task Ok!
[INFO]: The uid = 140582353602304, Start Do Task!
thread 140582353602304 is working, number = 171
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582131332864, Start Do Task!
thread 140582131332864 is working, number = 172
[INFO]: The uid = 140582361995008, Start Do Task!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582370387712, Start Do Task!
thread 140582370387712 is working, number = 174
[INFO]: The uid = 140582345209600, Start Do Task!
thread 140582361995008 is working, number = 173
[INFO]: Add Task Ok!
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 176
thread 140582345209600 is working, number = 175
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 177
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: Add Task Ok!
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 178
[INFO]: Add Task Ok!
[INFO]: The uid = 140582387173120, Start Do Task!
thread 140582387173120 is working, number = 179
[INFO]: Add Task Ok!
[INFO]: The uid = 140582122940160, Start Do Task!
thread 140582122940160 is working, number = 180
[INFO]: The uid = 140582131332864, Start Do Task!
thread 140582131332864 is working, number = 181
[INFO]: The uid = 140582353602304, Start Do Task!
thread 140582353602304 is working, number = 182
[INFO]: The uid = 140582370387712, Start Do Task!
thread 140582370387712 is working, number = 183
[INFO]: The uid = 140582361995008, Start Do Task!
thread 140582361995008 is working, number = 184
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 185
[INFO]: The uid = 140582345209600, Start Do Task!
thread 140582345209600 is working, number = 186
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 187
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 188
[INFO]: The uid = 140582387173120, Start Do Task!
thread 140582387173120 is working, number = 189
[INFO]: 检查消费者线程中...
当前任务队列剩有 10 个任务, 繁忙线程: 10, 存活线程: 10
[INFO]: The uid = 140582122940160, Start Do Task!
thread 140582122940160 is working, number = 190
[INFO]: The uid = 140582131332864, Start Do Task!
thread 140582131332864 is working, number = 191
[INFO]: The uid = 140582353602304, Start Do Task!
thread 140582353602304 is working, number = 192
[INFO]: The uid = 140582370387712, Start Do Task!
thread 140582370387712 is working, number = 193
[INFO]: The uid = 140582378780416, Start Do Task!
thread 140582378780416 is working, number = 194
[INFO]: The uid = 140582361995008, Start Do Task!
thread 140582361995008 is working, number = 195
[INFO]: The uid = 140582403958528, Start Do Task!
thread 140582403958528 is working, number = 196
[INFO]: The uid = 140582345209600, Start Do Task!
thread 140582345209600 is working, number = 197
[INFO]: The uid = 140582395565824, Start Do Task!
thread 140582395565824 is working, number = 198
[INFO]: The uid = 140582387173120, Start Do Task!
thread 140582387173120 is working, number = 199
[INFO]: uid = 140582122940160, 没有任务睡大觉!
[INFO]: uid = 140582353602304, 没有任务睡大觉!
[INFO]: uid = 140582131332864, 没有任务睡大觉!
[INFO]: uid = 140582370387712, 没有任务睡大觉!
[INFO]: uid = 140582378780416, 没有任务睡大觉!
[INFO]: uid = 140582361995008, 没有任务睡大觉!
[INFO]: uid = 140582403958528, 没有任务睡大觉!
[INFO]: uid = 140582345209600, 没有任务睡大觉!
[INFO]: uid = 140582387173120, 没有任务睡大觉!
[INFO]: uid = 140582395565824, 没有任务睡大觉!
[INFO]: 检查消费者线程中...
当前任务队列剩有 0 个任务, 繁忙线程: 0, 存活线程: 10
[INFO]: 线程过多, 减少线程!
[INFO]: The Thread tid: 140582122940160 Eixt!
[INFO]: The Thread tid: 140582353602304 Eixt!
[INFO]: 检查消费者线程中...
当前任务队列剩有 0 个任务, 繁忙线程: 0, 存活线程: 8
[INFO]: 线程过多, 减少线程!
[INFO]: The Thread tid: 140582131332864 Eixt!
[INFO]: The Thread tid: 140582370387712 Eixt!
[INFO]: 检查消费者线程中...
当前任务队列剩有 0 个任务, 繁忙线程: 0, 存活线程: 6
[INFO]: The Thread tid: 140582378780416 Eixt!
[INFO]: The Thread tid: 140582361995008 Eixt!
[INFO]: 线程过多, 减少线程!
[INFO]: 检查消费者线程中...
当前任务队列剩有 0 个任务, 繁忙线程: 0, 存活线程: 4
[INFO]: 线程过多, 减少线程!
[INFO]: The Thread tid: 140582403958528 Eixt!
[INFO]: The Thread tid: 140582345209600 Eixt!
释放线程中...
[INFO]: 检查消费者线程中...
当前任务队列剩有 0 个任务, 繁忙线程: 0, 存活线程: 2
[INFO]: 线程过多, 减少线程!
[INFO]: The Thread tid: 140582395565824 Eixt!
[INFO]: The Thread tid: 140582387173120 Eixt!
[INFO]: Free ThreadPool OK!
```

## 代码 V2.0.1 STL + 类封装

修改使用的STL为set因为是红黑树所以时间复杂度为O(lngN);

```C++
#ifndef _HX_POOL_H_
#define _HX_POOL_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <queue>
#include <set>

#define REDUCE_THREAD_NUMER 2   // 一次 减少线程数

// 任务结构体
typedef struct
{
    void *(*fun)(void *arg);    // 任务函数指针
    void *arg;                  // 该函数的参数包
} Task;

// 线程池结构体
typedef struct
{
    // --- 任务队列及管理与使用 ---
    std::queue<Task> TP_task;           // 定义任务队列
    pthread_t TP_op;                    // 管理者线程
    std::set<pthread_t> TP_consumer; // 消费者线程列表
    int TP_exitNum;                     // 需要销毁的线程个数

    // --- 线程安全(线程同步) ---
    pthread_mutex_t TP_m_all;           // 整个线程池的互斥锁
    pthread_mutex_t TP_m_busy;          // 繁忙线程的互斥锁 (因为任务可能会频繁加减, 线程也会切换任务/执行新任务)
    pthread_cond_t TP_c_taskFull;       // 任务队列满了 (不能再加了)
    pthread_cond_t TP_c_taskEmpty;      // 任务队列空了 (看是否需要减少空闲线程)

    // --- 线程池信息 ---
    int TP_taskMaxSize;                 // 最大任务数
    int TP_max;                         // 最大线程数
    int TP_min;                         // 最小线程数 (初始线程数)
    int TP_busy;                        // 繁忙线程数 (正在执行任务)
    int TP_idle;                        // 空闲线程数 (已挂起)
    int TP_live;                        // 存活线程数 == 空闲线程数 + 繁忙线程数

    // --- 线程池开关 ---
    bool TP_free;                       // 是否释放线程池 (1是, 0否)
} ThreadPool;

class HX_ThreadPool
{
public:
    /**
     * @brief 创建线程池并初始化
     * @param TP_min 最小线程数
     * @param TP_max 最大线程数
     * @param TP_taskMaxSize 最大任务数
     */
    HX_ThreadPool(int TP_min, int TP_max, int TP_taskMaxSize);

    ~HX_ThreadPool();

    // 销毁线程池
    int freeThreadPool();

    /**
     * @brief 给线程池添加任务
     * @param fun 任务函数指针
     * @param arg 参数包指针
     * @return 1 是成功; -1 是失败
     */
    int addTask(void *(*fun)(void *arg), void *arg);

    // 获取线程池中工作的线程的个数
    int getTP_busyThreadNumer();

    // 获取线程池中活着的线程的个数
    int getTP_liveThreadNumer();

private:
    // 不使用静态函数/全局函数, 不能使用其类函数函数指针

    // 工作的线程(消费者线程)任务函数
    static void *consumerThreadFun(void *arg);

    // 管理者线程 任务函数
    static void *OpThreadFun(void *arg);

    // 单个线程退出
    
    ThreadPool* pool;
};
#endif // _HX_POOL_H_
```

实现
```C++
#include "hx_pool.h"

HX_ThreadPool::HX_ThreadPool(int TP_min, int TP_max, int TP_taskMaxSize)
{
    this->pool = new ThreadPool[1];

    do
    {
        // 初始化线程同步变量
        if ( pthread_mutex_init(&pool->TP_m_all, NULL) < 0      ||
             pthread_mutex_init(&pool->TP_m_busy, NULL) < 0     ||
             pthread_cond_init(&pool->TP_c_taskEmpty, NULL) < 0 ||
             pthread_cond_init(&pool->TP_c_taskFull, NULL) < 0 )
        {
            printf("ERROR: init error!\n");
            break;
        }

        printf("[INFO]: init 同步变量 OK!\n");

        // 初始化线程信息
        pool->TP_consumer = std::set<pthread_t>();
        pool->TP_task = std::queue<Task>();
        pool->TP_exitNum = 0;
        pool->TP_max = TP_max;
        pool->TP_min = TP_min;
        pool->TP_taskMaxSize = TP_taskMaxSize;
        pool->TP_free = 0;
        pool->TP_live = pool->TP_idle = pool->TP_min;
        pool->TP_busy = 0;

        printf("[INFO]: init 线程信息 OK!\n");

        // 启动生产者(管理者)与消费者
        pthread_create(&pool->TP_op, NULL, this->OpThreadFun, pool);

        printf("[INFO]: start OP Thread OK!\n");

        for (int i = 0; i < pool->TP_live; ++i)
        {
            pthread_t c_t;
            pthread_create(&c_t, NULL, this->consumerThreadFun, pool);
            pool->TP_consumer.insert(c_t);
        }

        printf("[INFO]: start consumer Thread OK!\n");
        return;
    } while (0);

    delete pool;
}

void *HX_ThreadPool::consumerThreadFun(void *arg)
{
    ThreadPool *pool = (ThreadPool *)arg;

    printf("[INFO]: The Thread uid = %ld, Start!\n", pthread_self());

    while (!pool->TP_free)
    {
        pthread_mutex_lock(&pool->TP_m_all);
        // 判断是否有任务
        while (pool->TP_task.empty() && !pool->TP_free && !pool->TP_exitNum)
        {
            // 没有任务则挂起
            printf("[INFO]: uid = %ld, 没有任务睡大觉!\n", pthread_self());
            pthread_cond_wait(&pool->TP_c_taskEmpty, &pool->TP_m_all);
        }

        // 销毁线程 (自杀)
        if (pool->TP_exitNum > 0 || pool->TP_free)
        {
            --pool->TP_exitNum;
            pool->TP_consumer.emplace(pthread_self());
            --pool->TP_idle;
            --pool->TP_live;
            pthread_mutex_unlock(&pool->TP_m_all);
            printf("[INFO]: The Thread tid: %ld Eixt!\n", pthread_self());
            pthread_exit(NULL);
        }

        // 接任务
        Task task = pool->TP_task.front();
        pool->TP_task.pop();
        ++pool->TP_busy;
        --pool->TP_idle;
        pthread_mutex_unlock(&pool->TP_m_all);
        printf("[INFO]: The uid = %ld, Start Do Task!\n", pthread_self());

        // 做任务
        task.fun(task.arg);

        // 做完了
        pthread_mutex_lock(&pool->TP_m_all);
        --pool->TP_busy;
        ++pool->TP_idle;
        pthread_cond_signal(&pool->TP_c_taskFull);
        pthread_mutex_unlock(&pool->TP_m_all);
    }

    pthread_exit(NULL);
}

void *HX_ThreadPool::OpThreadFun(void *arg)
{
    ThreadPool *pool = (ThreadPool *)arg;
    while (!pool->TP_free || pool->TP_live > 0)
    {
        sleep(3);   // 每间隔3秒检查一下线程
        printf("[INFO]: 检查消费者线程中...\n");

        // 安全的获取数据
        pthread_mutex_lock(&pool->TP_m_all);
        int queueLen = pool->TP_task.size();
        int tLive = pool->TP_live;
        int tBusy = pool->TP_busy;
        pthread_mutex_unlock(&pool->TP_m_all); 

        printf("当前任务队列剩有 %d 个任务, 繁忙线程: %d, 存活线程: %d\n", queueLen, tBusy, tLive);

        // 添加线程
        // 任务的个数 > 存活的线程个数 && 存活的线程数 < 最大线程数
        if (queueLen > tLive && tLive < pool->TP_max)
        {
            // 增加
            for (int i = 0; i < REDUCE_THREAD_NUMER; ++i)
            {
                pthread_mutex_lock(&pool->TP_m_all);
                pthread_t c_t;
                pthread_create(&c_t, NULL, consumerThreadFun, pool);
                pool->TP_consumer.insert(c_t);
                ++pool->TP_idle;
                ++pool->TP_live;
                pthread_mutex_unlock(&pool->TP_m_all); 
            }
            printf("[INFO]: 线程过少, 添加线程!\n");
        }

        // 销毁线程
        // 忙的线程 * 2 < 存活的线程数 && 存活的线程 > 最小线程数
        if (tBusy * 2 < tLive && tLive > pool->TP_min || pool->TP_free)
        {
            // 减少
            pthread_mutex_lock(&pool->TP_m_all);
            if (pool->TP_free)
                pool->TP_exitNum = tLive;
            else
                pool->TP_exitNum = REDUCE_THREAD_NUMER;
            pthread_mutex_unlock(&pool->TP_m_all);

            // 让消费者线程自杀
            for (int i = 0; i < (pool->TP_free ? tLive : REDUCE_THREAD_NUMER); ++i)
            {
                pthread_cond_signal(&pool->TP_c_taskEmpty);
            }

            if (pool->TP_free)
                printf("[INFO]: 正在关闭线程池\n");
            else
                printf("[INFO]: 线程过多, 减少线程!\n");
        }
    }
    
    pthread_exit(NULL);
}

int HX_ThreadPool::addTask(void *(*fun)(void *arg), void *arg)
{
    Task task;
    task.fun = fun;
    task.arg = arg;
    pthread_mutex_lock(&pool->TP_m_all);
    do
    {
        if (pool->TP_free)
            break;

        while (pool->TP_task.size() == pool->TP_taskMaxSize)
        {
            // 任务队列已满, 等待添加
            pthread_cond_wait(&pool->TP_c_taskFull, &pool->TP_m_all);
        }

        if (pool->TP_task.size() < pool->TP_taskMaxSize)
        {
            pool->TP_task.push(task);
            pthread_cond_signal(&pool->TP_c_taskEmpty);
        }
        
        pthread_mutex_unlock(&pool->TP_m_all);
        printf("[INFO]: Add Task Ok!\n");
        return 1;
    } while(0);

    pthread_mutex_unlock(&pool->TP_m_all);
    printf("ERROR: Add Task Error!\n");
    return -1;
}

int HX_ThreadPool::getTP_busyThreadNumer()
{
    int res = 0;
    pthread_mutex_lock(&pool->TP_m_busy);
    res = pool->TP_busy;
    pthread_mutex_unlock(&pool->TP_m_busy);
    return res;
}

int HX_ThreadPool::getTP_liveThreadNumer()
{
    int res = 0;
    pthread_mutex_lock(&pool->TP_m_all);
    res = pool->TP_live;
    pthread_mutex_unlock(&pool->TP_m_all);
    return res;
}

int HX_ThreadPool::freeThreadPool()
{
    pthread_mutex_lock(&pool->TP_m_all);
    pool->TP_free = 1;
    pthread_mutex_unlock(&pool->TP_m_all);

    // 阻塞 等待线程退出
    pthread_join(pool->TP_op, NULL);

    pthread_mutex_destroy(&pool->TP_m_all);
    pthread_mutex_destroy(&pool->TP_m_busy);
    pthread_cond_destroy(&pool->TP_c_taskEmpty);
    pthread_cond_destroy(&pool->TP_c_taskFull);

    // 释放资源
    delete []pool;

    pool = nullptr;
    printf("[INFO]: Free ThreadPool OK!\n");
    return 1;
}

HX_ThreadPool::~HX_ThreadPool()
{
    if (this->pool)
    {
        freeThreadPool();
    }
}
```

使用

```C++
#include <stdio.h>
#include "hx_pool.h"

// 任务函数
void* taskFunc(void* arg)
{
    int num = *(int*)arg;
    printf("thread %ld is working, number = %d\n",
        pthread_self(), num);
    sleep(1);
    free(arg);
    return NULL;
}

int main()
{
    // 创建线程池
    HX_ThreadPool pool(2, 10, 20);
    for (int i = 0; i < 100; ++i)
    {
        int *arg = (int *)malloc(sizeof(int));
        *arg = i + 100;
        pool.addTask(taskFunc, arg);
    }

    sleep(5);
    printf("释放线程中...\n");
    pool.freeThreadPool(); // 如果不写则在类出它的作用域的时候进行析构释放
    return 0;
}
```

## C++11库实现线程池 std::thread 原子变量, 支持可变参数可调用对象!

附带实现了一个可替换的单例类, 专门用于日志输出, 可通过函数指针替换!


```C++
#pragma once
#ifndef _HX_THREAD_POOL_H_
#define _HX_THREAD_POOL_H_
#include <thread>                // C++线程
#include <functional>            // 可调用对象封装
#include <atomic>                // 原子变量
#include <mutex>                // 互斥锁
#include <condition_variable>    // 条件变量
#include <cstdarg>                // C 的可变参数
#include <chrono>                // 时间日期库
#include <map>
#include <queue>

#define SINGLE_NUME 2                // 单次增加的线程数量
#define CACHE_STR_ARR_SIZE_MAX 1024 // 缓冲字符串最大长度

using namespace std;

namespace HX {
    namespace tools {
        // 单例-饿汉-输出类
        class HXprint {
            HXprint();
        private:
            HXprint(const HXprint&) = delete;
            HXprint& operator =(const HXprint&) = delete;

            // 自带输出正常
            static void printInfo(const char* str, ...);

            // 自带输出异常
            static void printError(const char* str, ...);
        public:
            static HXprint* getHXprint();
            
            // 设置info输出函数
            void setPrintInfoFun(void (*fun)(const char* str, ...));

            // 设置error输出函数
            void setPrintErrorFun(void (*fun)(const char* str, ...));
            
            void (*ptr_pInfo)(const char* str, ...);
            void (*ptr_pError)(const char* str, ...);
        };
    }

    // 线程池
    class ThreadPool {
    public:
        // 为什么需要核心数 - 1?, 因为 还有主线程 (-1); 还有一个管理者线程 (-1), 但是因为有时间间隔, 所以可以忽略...
        /**
         * @brief 创建线程池并初始化, [-1] 代表使用 (当前CPU的核心数 - 1)
         * @param t_min 最小线程数
         * @param t_max 最大线程数
         * @param taskMaxSize 最大任务数
         * @param opTime 管理者线程 检查 消费者线程 的时间间隔, 单位 ms
         */
        ThreadPool(int t_min, int t_max = -1, int taskMaxSize = -1, int opTime = 2000);

        /**
         * @brief 给线程池添加任务
         * @param func 子线程需要执行的任务可调用对象
         * @param ... 可调用对象分别对应的参数, 若没有可以为空 (只写参数func)
         * @return 添加成功 1, 出错 0
         */
        template<typename Function, typename... Args>
        bool addTask(Function&& func, Args&&... args);

        template<typename Function>
        bool addTask(Function&& func);

        /**
         * @brief 设置单次增减线程数量
         * @param add 单次增加的线程数量
         * @param sub 单次减小的线程数量
         * @return 无
         */
        inline void setSinglNumer(int add = SINGLE_NUME, int sub = SINGLE_NUME);

        /**
         * @brief 设置管理者线程判断函数
         * @param addPtr 单次增加的线程的判断函数
         * @param subPtr 单次减小的线程的判断函数
         * @return 无
         */
        void setSinglFunPtr(bool (*addPtr)(int, int, int, int, int, int, int) = ifAddThread ,
                            bool (*subPtr)(int, int, int, int, int, int, int) = ifSubThread );

        /**
         * @brief 获取当前线程池忙碌的线程数
         * @param 无
         * @return 忙碌的线程数
         */
        int getPoolBusySize();

        /**
         * @brief 获取当前线程池存活的线程数
         * @param 无
         * @return 存活的线程数
         */
        int getPoolLiveSize();

        /**
         * @brief 获取当前线程池的任务队列的长度
         * @param 无
         * @return 任务队列的长度
         */
        int getTaskSize();

        // 销毁线程池
        void freeThreadPool();
        ~ThreadPool();
    private:
        // 工作的线程(消费者线程)任务函数
        static void consumerThreadFun(ThreadPool& pool);

        // 管理者线程 任务函数
        static void OpThreadFun(ThreadPool& pool);

        // 任务的个数>存活的线程个数 && 存活的线程数<最大线程数
        /**
         * @brief 自带的增加线程数量 判断函数
         * @param now_taskSize 当前任务数
         * @param now_busy 当前繁忙的线程
         * @param now_idle 当前闲置的线程
         * @param now_live 当前存活的线程
         * @param t_min    线程池的最小线程数
         * @param t_max       线程池的最大线程数
         * @param taskMaxSize 任务队列最大长度
         * @return 0 不进行增加 / 1 进行增加
         */
        static bool ifAddThread(int now_taskSize, int now_busy, int now_idle, int now_live, int t_min, int t_max, int taskMaxSize);

        // 忙的线程*2 < 存活的线程数 && 存活的线程>最小线程数
        /**
         * @brief 自带的减少线程数量 判断函数
         * @param now_taskSize 当前任务数
         * @param now_busy 当前繁忙的线程
         * @param now_idle 当前闲置的线程
         * @param now_live 当前存活的线程
         * @param t_min    线程池的最小线程数
         * @param t_max       线程池的最大线程数
         * @param taskMaxSize 任务队列最大长度
         * @return 0 不进行减少 / 1 进行减少
         */
        static bool ifSubThread(int now_taskSize, int now_busy, int now_idle, int now_live, int t_min, int t_max, int taskMaxSize);

        // --- 任务队列 组 ---
        queue<std::function<void()>> TP_threadTaskQueue; // 任务队列
        int taskMaxSize;            // 任务数量
        int opTime;                    // 间隔检查时间
        thread* TP_op;                // 管理者线程

        mutex TP_mutex_all;                        // 整个线程池的互斥锁

        condition_variable taskQueueFull;        // 条件变量: 任务队列满了
        condition_variable taskQueueEmpty;        // 条件变量: 任务队列空了
        condition_variable freeConsumer_cond;    // 条件变量: 需要释放的线程

        bool (*ifAddFunPtr)(int, int, int, int, int, int, int); // 函数指针指向是否增加线程的判断函数
        bool (*ifSubFunPtr)(int, int, int, int, int, int, int); // 函数指针指向是否减少线程的判断函数

        // --- 线程池 信息 ---
        int t_min;                    // 最小线程数
        int t_max;                    // 最大线程数
        int singleAdd;                // 单次添加的线程数
        int singleSub;                // 单次销毁的线程数

        atomic_int TP_busy;         // 繁忙线程数 (正在执行任务)
        atomic_int TP_idle;         // 空闲线程数 (已挂起)
        atomic_int TP_live;         // 存活线程数 == 空闲线程数 + 繁忙线程数
        atomic_int del_t_num;        // 目前需要删除的线程数

        // --- 线程池 社畜 ---
        map<thread::id, thread *> TP_consumer;    // 消费者红黑树 (添加/查找/删除 O(logN) 时间复杂度)
        queue<thread*> free_consumer;            // 需要释放的线程

        // --- 线程池 开关 ---
        atomic_bool TP_free;                    // 线程池是否释放
    };
}

template<typename Function, typename... Args>
bool HX::ThreadPool::addTask(Function&& func, Args&&... args)
{
    auto print = HX::tools::HXprint::getHXprint();

    unique_lock<mutex> lock(this->TP_mutex_all);
    do
    {
        if (this->TP_free)
            break;

        while (this->TP_threadTaskQueue.size() == this->taskMaxSize)
        {
            // 任务队列已满, 等待添加
            this->taskQueueFull.wait(lock);
        }

        if (this->TP_threadTaskQueue.size() < this->taskMaxSize)
        {
            this->TP_threadTaskQueue.push(std::bind(func, std::forward<Args>(args)...));
            this->taskQueueEmpty.notify_one();
        }

        lock.unlock();

        print->ptr_pInfo("添加任务成功!");
        return 1;
    } while (0);

    lock.unlock();
    print->ptr_pError("添加任务出错: 线程池在添加时已经在销毁!");
    return 0;
}

template<typename Function>
bool HX::ThreadPool::addTask(Function&& func)
{
    auto print = HX::tools::HXprint::getHXprint();

    unique_lock<mutex> lock(this->TP_mutex_all);
    do
    {
        if (this->TP_free)
            break;

        while (this->TP_threadTaskQueue.size() == this->taskMaxSize)
        {
            // 任务队列已满, 等待添加
            this->taskQueueFull.wait(lock);
        }

        if (this->TP_threadTaskQueue.size() < this->taskMaxSize)
        {
            this->TP_threadTaskQueue.push(func);
            this->taskQueueEmpty.notify_one();
        }
        lock.unlock();

        print->ptr_pInfo("添加任务成功!");
        return 1;
    } while (0);

    lock.unlock();
    print->ptr_pError("添加任务出错: 线程池在添加时已经在销毁!");
    return 0;
}

#endif // _HX_THREAD_POOL_H_
```


```C++
#include "HXThreadPool.h"

void HX::tools::HXprint::printInfo(const char* str, ...)
{
    va_list args;
    va_start(args, str);

    // 构建输出字符串
    char buffer[CACHE_STR_ARR_SIZE_MAX];
    vsnprintf(buffer, sizeof(buffer), str, args);

    printf("[INFO]: %s\n", buffer);
    va_end(args);
}

HX::tools::HXprint::HXprint()
{
    this->ptr_pInfo = printInfo;
    this->ptr_pError = printError;
}

void HX::tools::HXprint::printError(const char* str, ...)
{
    va_list args;
    va_start(args, str);

    // 构建输出字符串
    char buffer[CACHE_STR_ARR_SIZE_MAX];
    vsnprintf(buffer, sizeof(buffer), str, args);

    printf("[ERROR]: %s\n", buffer);
    va_end(args);
}

HX::tools::HXprint* HX::tools::HXprint::getHXprint()
{
    static HXprint hxPrint;
    return &hxPrint;
}

void HX::tools::HXprint::setPrintInfoFun(void(*fun)(const char* str, ...))
{
    this->ptr_pInfo = fun;
}

void HX::tools::HXprint::setPrintErrorFun(void(*fun)(const char* str, ...))
{
    this->ptr_pError = fun;
}


/////////////////////////////////////////////////////////////////////////////
// 正文                                                                        /
/////////////////////////////////////////////////////////////////////////////

HX::ThreadPool::ThreadPool(int t_min, int t_max, int taskMaxSize, int opTime) : TP_mutex_all(), taskQueueFull(), taskQueueEmpty(), TP_threadTaskQueue(), TP_consumer(), free_consumer(), freeConsumer_cond()
{
    auto print = HX::tools::HXprint::getHXprint();

    do 
    {
        this->TP_free = false;
        this->setSinglNumer();
        this->setSinglFunPtr();

        this->t_min = t_min;
        this->t_max = (t_max == -1) ? thread::hardware_concurrency() - 1: t_max;

        this->taskMaxSize = (taskMaxSize == -1) ? thread::hardware_concurrency() - 1 : taskMaxSize;
        this->opTime = opTime;

        this->TP_busy = 0;
        this->del_t_num = 0;
        this->TP_idle = t_min;
        this->TP_live = t_min;

        print->ptr_pInfo("初始化成员变量成功!");

        // 启动子线程
        for (int i = 0; i < this->TP_live; ++i)
        {
            auto p = new thread(consumerThreadFun, std::ref(*this));
            this->TP_consumer.insert(make_pair(p->get_id(), p));
        }

        print->ptr_pInfo("已经启动 %d 个消费者线程!", t_min);

        // 启动管理者线程
        this->TP_op = new thread(OpThreadFun, std::ref(*this));

        print->ptr_pInfo("已经启动管理者线程");

        return;
    } while (0);
    
    print->ptr_pError("初始化线程池出错!");
}

void HX::ThreadPool::consumerThreadFun(ThreadPool& pool)
{
    auto print = HX::tools::HXprint::getHXprint();
    print->ptr_pInfo("子线程: %ld, 启动!", this_thread::get_id());

    unique_lock<mutex> lock(pool.TP_mutex_all);
    lock.unlock();

    while (!pool.TP_free)
    {
        lock.lock();
        // 判断是否有任务
        while (pool.TP_threadTaskQueue.empty() && !pool.TP_free && !pool.del_t_num.load())
        {
            // 没有任务则挂起
            print->ptr_pInfo("uid = %ld, 没有任务睡大觉!", this_thread::get_id());
            pool.taskQueueEmpty.wait(lock);
        }

        // 销毁线程 (自杀)
        if (pool.del_t_num > 0 || pool.TP_free)
        {
            --pool.del_t_num;
            auto p = pool.TP_consumer.find(this_thread::get_id());
            pool.free_consumer.push(p->second);
            pool.TP_consumer.erase(p);
            lock.unlock();
            --pool.TP_idle;
            --pool.TP_live;
            print->ptr_pInfo("线程: %ld 已退出!", this_thread::get_id());
            pool.freeConsumer_cond.notify_one();
            return;
        }

        // 接任务
        auto task = pool.TP_threadTaskQueue.front();
        pool.TP_threadTaskQueue.pop();
        lock.unlock();

        ++pool.TP_busy;
        --pool.TP_idle;
        print->ptr_pInfo("线程: %ld, 开工!", this_thread::get_id());

        // 做任务
        task();

        // 做完了
        --pool.TP_busy;
        ++pool.TP_idle;
        lock.lock();
        pool.taskQueueFull.notify_one(); // 通知任务添加函数, 如果在排队可以放一个人过来了
        lock.unlock();
    }

    print->ptr_pError("线程创建时, 线程池已在销毁!");
    
    lock.lock();
    auto p = pool.TP_consumer.find(this_thread::get_id());
    pool.free_consumer.push(p->second);
    pool.TP_consumer.erase(p);
    lock.unlock();
    pool.freeConsumer_cond.notify_one();
    
    --pool.del_t_num;
    --pool.TP_idle;
    --pool.TP_live;
}

void HX::ThreadPool::OpThreadFun(ThreadPool& pool)
{
    auto print = HX::tools::HXprint::getHXprint();
    print->ptr_pInfo("管理者线程: %ld, 启动!", this_thread::get_id());

    while (!pool.TP_free || pool.TP_live > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(pool.opTime));   // 每间隔{opTime}毫秒检查一下线程
        //print->ptr_pInfo("检查消费者线程中...");

        // 安全的获取数据
        unique_lock<mutex> lock(pool.TP_mutex_all);
        int queueLen = pool.TP_threadTaskQueue.size();
        lock.unlock();
        int now_busy = pool.TP_busy;
        int now_idle = pool.TP_idle;
        int now_live = pool.TP_live;

        print->ptr_pInfo("当前任务队列剩有 %d 个任务, 繁忙线程: %d, 存活线程: %d", queueLen, now_busy, now_live);

        // 添加线程
        if (!pool.TP_free && pool.ifAddFunPtr(queueLen, now_busy, now_idle, now_live, pool.t_min, pool.t_max, pool.taskMaxSize))
        {
            // 增加
            for (int i = 0; i < pool.singleAdd; ++i)
            {
                lock.lock();
                auto p = new thread(consumerThreadFun, std::ref(pool));
                pool.TP_consumer.insert(make_pair(p->get_id(), p));
                lock.unlock();

                ++pool.TP_idle;
                ++pool.TP_live;
            }
            print->ptr_pInfo("线程过少, 添加线程!");
            continue;
        }

        // 销毁线程
        if (pool.TP_free || pool.ifSubFunPtr(queueLen, now_busy, now_idle, now_live, pool.t_min, pool.t_max, pool.taskMaxSize))
        {
            // 减少
            if (pool.TP_free)
                pool.del_t_num = now_live;
            else
                pool.del_t_num = pool.singleSub;

            if (pool.TP_free)
                print->ptr_pInfo("正在关闭线程池...");
            else
                print->ptr_pInfo("线程过多, 减少线程!");

            // 让消费者线程自杀
            int n = pool.del_t_num;
            for (int i = 0; i < n; ++i)
            {
                lock.lock();
                pool.taskQueueEmpty.notify_one();
                print->ptr_pInfo("<<<<<<<<<<<<<<");
                pool.freeConsumer_cond.wait(lock);
                print->ptr_pInfo(">>>>>>>>>>>>>>");
                auto p = pool.free_consumer.front();
                pool.free_consumer.pop();
                p->join();
                delete p;
                lock.unlock();
            }
            continue;
        }
    }

    while (!pool.free_consumer.empty())
    {
        print->ptr_pInfo("发现内存泄漏!");
        auto p = pool.free_consumer.front();
        pool.free_consumer.pop();
        p->join();
        delete p;
    }
}

void HX::ThreadPool::freeThreadPool()
{
    this->TP_free = true;

    this->TP_op->join();
    delete this->TP_op;
    HX::tools::HXprint::getHXprint()->ptr_pInfo("已释放线程池!");
}

inline void HX::ThreadPool::setSinglNumer(int add, int sub)
{
    this->singleAdd = add;
    this->singleSub = sub;
}

// 任务的个数>存活的线程个数 && 存活的线程数<最大线程数
bool HX::ThreadPool::ifAddThread(int now_taskSize, int now_busy, int now_idle, int now_live, int t_min, int t_max, int taskMaxSize)
{
    return now_taskSize > now_live && now_live < t_max;
}

// 忙的线程*2 < 存活的线程数 && 存活的线程>最小线程数
bool HX::ThreadPool::ifSubThread(int now_taskSize, int now_busy, int now_idle, int now_live, int t_min, int t_max, int taskMaxSize)
{
    return (now_busy << 1) < now_live && now_live > t_min;
}

HX::ThreadPool::~ThreadPool()
{
    if (!this->TP_free)
    {
        this->freeThreadPool();
    }
}

void HX::ThreadPool::setSinglFunPtr(bool(*addPtr)(int, int, int, int, int, int, int), bool(*subPtr)(int, int, int, int, int, int, int))
{
    this->ifAddFunPtr = addPtr;
    this->ifSubFunPtr = subPtr;
}

int HX::ThreadPool::getPoolBusySize()
{
    return this->TP_busy.load();
}

int HX::ThreadPool::getPoolLiveSize()
{
    return this->TP_live.load();
}

int HX::ThreadPool::getTaskSize()
{
    return this->TP_threadTaskQueue.size();
}
```

使用示例

```C++
#include "HXThreadPool.h"
#include "vld.h" // 内存泄漏检测
void fun(int a);

void fun(int a)
{
    printf("%ld 线程已运行 %d\n", this_thread::get_id(), a);
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void fun_2()
{
    printf("%ld 线程已运行 !!!\n", this_thread::get_id());
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

struct MyStruct
{
    void operator() (int i) {
        printf("%ld 线程 [%d] 已运行 awa\n", this_thread::get_id(), i);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
};

int main()
{
    HX::ThreadPool pool(2);
    pool.setSinglNumer(10, 2);
    for (int i = 0; i < 50; ++i)
        pool.addTask(MyStruct(), i);
        
    std::this_thread::sleep_for(std::chrono::seconds(6));
    return 0;
}
```
