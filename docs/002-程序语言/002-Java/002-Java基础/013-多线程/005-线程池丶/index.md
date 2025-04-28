# 线程池
## 执行器
> 在前面的所有示例中，由新线程（由其`Runnable`对象定义）执行的任务与由`Thread`对象定义的线程本身之间存在紧密的联系。 这对于小型应用程序非常有效，但是在大型应用程序中，将线程管理和创建与其余应用程序分开是有意义的。 封装这些功能的对象称为执行器。

### Executor接口方法

```java
void execute(Runnable command); // 将任务添加到线程池中，等待线程池调度执行
```

### ExecutorService接口常用方法

```java
void shutdown();                 // 有序关闭线程池，不再接收新的线程任务，但池中已有任务会执行
List<Runnable> shutdownNow();    // 关闭线程池，尝试停止所有正在执行的任务，并将池中等待执行的任务返回
boolean isShutdown();            // 检测线程池是否已经关闭
boolean isTerminated();          // 检测线程池是否已经终止
Future<?> submit(Runnable task); // 提交一个任务至线程池中(返回值是该任务执行的结果)
```

## 线程池
> `java.util.concurrent`中的大多数执行程序实现都使用线程池，该线程池由工作线程组成。这种线程与它执行的`Runnable`和`Callable`任务分开存在，通常用于执行多个任务。
>
> 使用工作线程可以最大程度地减少线程创建所带来的开销。线程对象占用大量内存，在大型应用程序中，分配和取消分配许多线程对象会产生大量内存管理开销。
>
> 线程池的一种常见类型是固定线程池。这种类型的池始终具有指定数量的正在运行的线程。如果某个线程在仍在使用时以某种方式终止，则它将自动替换为新线程。任务通过内部队列提交到池中，该内部队列在活动任务多于线程时容纳额外的任务。
>
> 固定线程池的一个重要优点是使用该线程池的应用程序可以正常降级

### 线程池构造方法

```java
public ThreadPoolExecutor(int corePoolSize,                  // 核心线程数
                          int maximumPoolSize,               // 最大线程数
                          long keepAliveTime,                // 工作线程存活时间
                          TimeUnit unit,                     // 时间单位
                          BlockingQueue<Runnable> workQueue, // 任务队列
                          ThreadFactory threadFactory,       // 线程工厂
                          RejectedExecutionHandler handler)  // 拒绝处理器
```

- `threadFactory`（线程工厂）：
    - 用于创建新线程。由同一个`threadFactory`创建的线程，属于同一个`ThreadGroup`，创建的线程优先级都为`Thread.NORM_PRIORITY`，以及是非守护进程状态。`threadFactory`创建的线程也是采用`new Thread()`方式，`threadFactory`创建的线程名都具有统一的风格：`pool-m-thread-n`（m为线程池的编号，n为线程池内的线程编号）;

- `handler`（线程饱和策略）：当线程池和队列都满了，则表明该线程池已达饱和状态。
    - `ThreadPoolExecutor.AbortPolicy`：处理程序遭到拒绝，则直接抛出运行时异常 `RejectedExecutionException`。(默认策略)
    - `ThreadPoolExecutor.CallerRunsPolicy`：调用者所在线程来运行该任务，此策略提供简单的反馈控制机制，能够减缓新任务的提交速度。
    - `ThreadPoolExecutor.DiscardPolicy`：无法执行的任务将被删除。
    - `ThreadPoolExecutor.DiscardOldestPolicy`：如果执行程序尚未关闭，则位于工作队列头部的任务将被删除，然后重新尝试执行任务（如果再次失败，则重复此过程）。

示例

```java
import java.util.Queue;
import java.util.concurrent.*;

public class ThreadPoolText {
    public static void main(String[] args) {
        BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>(10);
        ThreadPoolExecutor pool = new ThreadPoolExecutor(
                5,                                   // 最小线程数
                10,                                  // 最大线程数
                2,                                   // 空闲线程消失的时间
                TimeUnit.SECONDS,                    // 单位
                queue,                               // 任务队列
                Executors.defaultThreadFactory(),    // 线程创建方式(使用默认的)
                new ThreadPoolExecutor.CallerRunsPolicy() // 任务队列过长后的拒绝策略
        );

        for (int i = 0; i < 50; ++i) {
            pool.execute(new ThreadPoolTesk(i));
            System.out.printf("当前线程数: %d, 任务队列长度: %d, 线程池完成任务个数: %d\n",
                    pool.getPoolSize(),
                    pool.getQueue().size(),
                    pool.getCompletedTaskCount());

            try {
                Thread.sleep(100L);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        pool.shutdown();
    }

    static class ThreadPoolTesk implements Runnable {
        private int id;

        ThreadPoolTesk(int id) {
            this.id = id;
        }

        @Override
        public void run() {
            System.out.printf("%d 任务已运行!\n", this.id);

            try {
                Thread.sleep(200L);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
```

### 线程池监控
利用线程池提供的参数进行监控，参数如下：

- `taskCount`：线程池需要执行的任务数量。
- `completedTaskCount`：线程池在运行过程中已完成的任务数量，小于或等于taskCount。
- `largestPoolSize`：线程池曾经创建过的最大线程数量，通过这个数据可以知道线程池是否满过。如等于线程池的最大大小，则表示线程池曾经满了。
- `getPoolSize`：线程池的线程数量。如果线程池不销毁的话，池里的线程不会自动销毁，所以这个大小只增不减。
- `getActiveCount`：获取活动的线程数。

通过扩展线程池进行监控：继承线程池并重写线程池的`beforeExecute()`，`afterExecute()`和`terminated()`方法，可以在任务执行前、后和线程池关闭前自定义行为。如监控任务的平均执行时间，最大执行时间和最小执行时间等。


### 线程池工作流程
<span style="margin-left: 30px;">线程池启动后，核心线程就已经启动，当一个新的任务提交到线程池时，首先会检测当前是否存在空闲的核心线程，如果存在，就将该任务交给这个空闲核心线程执行。如果不存在，那么就将该任务交给队列，在队列中排队等候。如果队列满了，此时线程池会检测当前工作线程数是否达到最大线程数，如果没有达到最大线程数，那么将由线程工厂创建新的工作线程来执行队列中的任务，这样，队列中就有空间能够容纳这个新任务。如果创建的工作线程在执行完任务后，在给定的时间范围内没有新的任务执行，这些工作线程将死亡。如果已经达到最大线程数，那么线程池将采用提供的拒绝处理策略来拒绝这个新任务。

### 线程池创建方式
简单的
```java
public class ExecutorTest {
    public static void main(String[] args) {
        // 创建一个给定核心线程数以及最大线程数的线程池，该线程池队列非常大
        ExecutorService pool1 = Executors.newFixedThreadPool(5);
        // 创建只有一个核心线程数以及最大线程数的线程池，该线程池队列非常大
        ExecutorService pool2 = Executors.newSingleThreadExecutor();
        // 创建一个核心线程为0,最大线程数为整数的最大值的可缓存的线程池
        ExecutorService pool3 = Executors.newCachedThreadPool();
        // 创建一个给定核心线程数，最大线程数为整数的最大值的可调度的线程池
        ExecutorService pool4 = Executors.newScheduledThreadPool(5);
    }
}
```

### 线程池的使用

```java
public class ExecutorTaskTest {
    public static void main(String[] args) {
        ExecutorService service = Executors.newFixedThreadPool(5);
        for(int i = 0; i < 100; i++) {
            int order = i;
            service.submit(()-> System.out.println("正在执行任务" + order)）;
        }
        service.shutdown();
    }
}
```

## 参考
### [1]
[java线程池使用最全详解](https://blog.csdn.net/qq_40093255/article/details/116990431)

[Java 多线程：彻底搞懂线程池](https://blog.csdn.net/u013541140/article/details/95225769)