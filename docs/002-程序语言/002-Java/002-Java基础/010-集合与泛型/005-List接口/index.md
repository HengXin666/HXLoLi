# List 接口
## 特性描述
> 列表是有序的集合（有时称为序列）。 列表可能包含重复的元素。
>
> Java平台包含两个常用的List实现。`ArrayList`通常是性能较好的实现，而`LinkedList`在某些情况下可以提供更好的性能。

## List 接口常用方法
```java
public interface List<E> extends Collection<E> {
    E get(int index);                            // 获取给定位置的元素
    E set(int index, E element);                 // 修改给定位置的元素
    void add(int index, E element);              // 在给定位置插入一个元素
    E remove(int index);                         // 移除给定位置的元素
    int indexOf(Object o);                       // 获取给定元素第一次出现的下标
    int lastIndexOf(Object o);                   // 获取给定元素最后一次出现的下标
    ListIterator<E> listIterator();              // 获取List集合专有的迭代器
    ListIterator<E> listIterator(int index);     // 获取List集合专有的迭代器，从给定的下标位置开始的迭代器
    List<E> subList(int fromIndex, int toIndex); // 获取List集合的一个子集合
    // ...
}
```

# ArrayList
类似于C++的vector

使用示例:
```java
import java.util.ArrayList;
import java.util.Iterator;
import java.util.ListIterator;

public class ListText {
    public static void main(String[] args) {
        // 初始化
        ArrayList<Integer> list = new ArrayList<>(/*可以指定默认大小(不指定则为10)*/);

        // 添加元素
        for (int i = 1; i <= 6; ++i)
            list.add(i);

        // 在指定下标处添加元素
        list.add(0, 0);

        // 修改某下标的元素的值
        list.set(1, -1);

        // 删除为某(2)下标的元素
        list.remove(2);

        // 删除List中第一个值为obj的对象
        list.remove((Integer) 5);
        
        // 支持基于范围的for遍历
        for (Integer it : list)
            System.out.printf("%d ", it);

        System.out.println("");
        // 使用迭代器
        Iterator<Integer> ite = list.iterator();

        // 只能进行前向遍历
        while (ite.hasNext()) {
            Integer it = ite.next();
            System.out.printf("%d ", it);
        }
        System.out.println("");

        // 使用List专用的迭代器, 可以双向迭代
        ListIterator<Integer> l_ite = list.listIterator(list.size()); // 指定获取尾迭代器类似于C++STL的 .end() 位置
        while (l_ite.hasPrevious()) {
            Integer it = l_ite.previous();
            System.out.printf("%d ", it);
        }
    }
}
```

`ArrayList`继承于`AbstractList`，`AbstractList`继承于`AbstractColletion`

`ensureCapacityInternal(int minCapacity)`确保数组有足够的容量来存储新添加的数据

`void grow(int minCapacity)`实现数组扩容，扩容至原来的1.5倍

`l_ite`可以从前到后对集合进行遍历，也可以从后往前对集合进行遍历,还可以向集合中添加元素，修改元素。而`ite`只能从前到后对集合进行遍历。

<span style="color:red">`ArrayList`底层采用的是数组来存储元素，根据数组的特性，`ArrayList`在随机访问时效率极高，在增加和删除元素时效率偏低，因为在增加和删除元素时会涉及到数组中元素位置的移动。`ArrayList`在扩容时每次扩容到原来的1.5倍</span>

# LinkedList
## 手写
写一个单向链表:

```java
/**
 * 单向链表
 * */
class MyList<T> {
    private MyList<T> next;

    private T data;

    MyList(T data, MyList<T> next) {
        this.data = data;
        this.next = next;
    }

    public T getData() {
        return this.data;
    }

    public void setData(T data) {
        this.data = data;
    }

    public MyList<T> getNext() {
        return this.next;
    }

    public void setNext(MyList<T> next) {
        this.next = next;
    }
}

public class MyListText {
    public static void main(String[] args) {
        MyList<String> node_1 = new MyList<>("第一个结点", null);
        MyList<String> node_2 = new MyList<>("第二个结点", null);
        MyList<String> node_3 = new MyList<>("第三个结点", null);
        node_1.setNext(node_2);
        node_2.setNext(node_3);

        for (MyList<String> it = node_1; it != null; it = it.getNext()) {
            System.out.println(it.getData());
        }
    }
}
```

## LinkedList
由java提供的双向链表, 类似于C++STL的std::list

继承链: `LinkedList`->`AbstractSequentialList`->`AbstractList`->`AbstractCollection`->`Collection`->`Iterable`

```java extends
public interface Iterable<T>

public interface Collection<E> extends Iterable<E>

public abstract class AbstractCollection<E> implements Collection<E>

public abstract class AbstractList<E> extends AbstractCollection<E> implements List<E>

public abstract class AbstractSequentialList<E> extends AbstractList<E>

public class LinkedList<E>
    extends AbstractSequentialList<E>
    implements List<E>, Deque<E>, Cloneable, java.io.Serializable
```

常用方法:
- `void addFirst(E e)` 将数据存储在链表的头部
- `void addLast(E e)` 将数据存储在链表的尾部
- `E removeFirst()` 移除链表头部数据
- `E removeLast()` 移除链表尾部数据

示例:
```java
public class MyListText {
    public static void main(String[] args) {
        LinkedList<Integer> list = new LinkedList<>();
        list.add(1);
        list.add(2);
        list.add(3);
        for (ListIterator<Integer> it = list.listIterator(); it.hasNext(); ) {
            System.out.println(it.next());
        }
        list.remove((Integer) 2); // 删除值为2的obj (如果单纯是数字则是对应索引)
        for (ListIterator<Integer> it = list.listIterator(list.size()); it.hasPrevious(); ) {
            System.out.println(it.previous());
        }
    }
}
```

<span style="color:red">`LinkedList`底层采用的是双向链表来存储数据，根据链表的特性可知，`LinkedList`在增加和删除元素时效率极高，只需要链之间进行衔接即可。在随机访问时效率较低，因为需要从链的一端遍历至链的另一端。</span>
