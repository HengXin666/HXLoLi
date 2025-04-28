# Queue 接口
## 特性描述
> 队列是用于在处理之前保存元素的集合. 除了基本的收集操作外, 队列还提供其他插入, 移除和检查操作.
>
> 队列通常但不是必须以`FIFO(先进先出)`的方式对元素进行排序. 优先队列除外,它们根据元素的值对元素进行排序(有关详细信息,请参见“对象排序”部分). 无论使用哪种排序,队列的开头都是将通过调用`remove`或`poll`删除的元素. 在`FIFO`队列中,所有新元素都插入到队列的尾部. 其他种类的队列可能使用不同的放置规则. 每个`Queue`实现必须指定其排序属性.
>
> 队列实现有可能限制其持有的元素数量; 这样的队列称为有界队列. `java.util.concurrent`中的某些`Queue`实现是有界的,但`java.util`中的某些实现不受限制.

## Queue 接口常用方法

```java
boolean add(E e);   // 向队列中添加一个元素，如果出现异常，则直接抛出异常
boolean offer(E e); // 向队列中添加一个元素，如果出现异常，则返回false
E remove();         // 移除队列中第一个元素，如果队列中没有元素，则将抛出异常
E poll();           // 移除队列中第一个元素，如果队列中没有元素，则返回null
E element();        // 获取队列中的第一个元素，但不会移除。如果队列为空，则将抛出异常
E peek();           // 获取队列中的第一个元素，但不会移除。如果队列为空，则返回null
```

# LinkedBlockingQueue
`LinkedBlockingQueue`是一个`FIFO`队列，队列有长度，超出长度范围的元素将无法存储进队列. **实际上这个就是java的堆!(默认是小根堆 ([堆排序](../../../../../001-计佬常識/001-数据结构与算法/006-【算法】排序算法/004-堆排序/index.md)))**

示例

```java
import java.util.Iterator;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

public class QueueText {
    public static void main(String[] args) {
        Queue<String> queue = new LinkedBlockingQueue<>(6); // 不初始化长度, 会被分配 2^31 - 1 个容量
        queue.add("data_1"); // 添加元素, 如果超过容积会抛出异常
        System.out.println(queue.offer("data_2")); // 安全的添加元素, 如果超过容积只会返回false
        queue.remove(); // 删除队尾, 为空则抛出异常
        queue.poll(); // 删除队尾, 为空则返回空
        queue.add("data_11");
        queue.add("data_45");
        queue.add("data_14");

        // 使用迭代器
        Iterator<String> iterator = queue.iterator();
        while (iterator.hasNext()) {
            System.out.println(iterator.next());
        }
      
        // 遍历(实际上是一边遍历一边删除)
        while (!queue.isEmpty()) {
            System.out.println(queue.poll());
        }
    }
}
```

# PriorityQueue
`PriorityQueue`是一个有排序规则的队列，存入进去的元素是无序的，队列有长度，超出长度范围的元素将无法存储进队列。<span style="color:red">需要注意的是，如果存储的元素如果不能进行比较排序，也未提供任何对元素进行排序的方式，运行时会抛出异常.</span>

示例

```java
public class QueueText {
    public static void main(String[] args) {
        Queue<Integer> queue = new PriorityQueue<>(); // 默认长度是11
        queue.offer(1);
        queue.offer(4);
        queue.offer(3);
        queue.offer(2);
        queue.offer(6);
        Iterator<Integer> iterator = queue.iterator();
        while (iterator.hasNext()) {
            System.out.println(iterator.next()); // 1, 2, 3, 4, 6
        }
    }
}
```

思考：如果 `PriorityQueue` 队列中存储的是对象，会怎么排序？

- 答: 如果对象不能进行比较，则不能排序，运行时会报异常。要解决这个问题，需要使用Java平台提供的**比较器接口**。
