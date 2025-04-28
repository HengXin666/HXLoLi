# 死锁
## 死锁的概念
> 死锁描述了一种情况，其中两个或多个线程永远被阻塞，互相等待

## 死锁发生条件
- 互斥条件

    线程要求对所分配的资源进行排他性控制，即在一段时间内某资源仅为一个线程所占有。此时若有其他线程请求该资源，则请求线程只能等待。

- 不可剥夺条件

    线程所获得的资源在未使用完毕之前，不能被其他线程强行夺走，即只能由获得该资源的线程自己来释放（只能是主动释放)。

- 请求与保持条件

    线程已经保持了至少一个资源，但又提出了新的资源请求，而该资源已被其他线程占有，此时请求线程被阻塞，但对自己已获得的资源保持不放。

- 循环等待

    存在一种线程资源的循环等待链，链中每一个线程已获得的资源同时被链中下一个线程所请求。

示例

```java
public class DaiLock {
    public static void main(String[] args) {
        Object o1 = new Object();
        Object o2 = new Object();
        DaiLcokTask task1 = new DaiLcokTask(o1, o2, true);
        DaiLcokTask task2 = new DaiLcokTask(o1, o2, false);
        Thread t1 = new Thread(task1, "线程一");
        Thread t2 = new Thread(task2, "线程二");
        t1.start();
        t2.start();
    }
}

class DaiLcokTask implements Runnable {
    private Object o1;
    private Object o2;
    private boolean tag; // 模式

    DaiLcokTask(Object o1, Object o2, boolean tag) {
        this.o1 = o1;
        this.o2 = o2;
        this.tag = tag;
    }

    @Override
    public void run() {
        while (true) {
            String name = Thread.currentThread().getName();
            if (tag) {
                synchronized (this.o1) {
                    System.out.println(name + "锁上o1");

                    synchronized (this.o2) {
                        System.out.println(name + "锁上o2");
                    }
                }
            }
            else {
                synchronized (this.o2) {
                    System.out.println(name + "锁上o2");

                    synchronized (this.o1) {
                        System.out.println(name + "锁上o1");
                    }
                }
            }
        }
    }
}
```

更多: 为什么不问一下隔壁的C/C++? [死锁](../../../../001-C++/002-tmp丶C++丶memo/005-C++多进程与多线程/001-C语言版/005-死锁丶/index.md)