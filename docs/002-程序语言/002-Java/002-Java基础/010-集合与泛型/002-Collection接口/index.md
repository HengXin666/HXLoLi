# Collection 接口
## Collection 接口常用方法

```java
int size();                                // 获取集合的大小
boolean isEmpty();                         // 判断集合是否存有元素
boolean contains(Object o);                // 判断集合中是否包含给定的元素
Iterator<E> iterator();                    // 获取集合的迭代器
Object[] toArray();                        // 将集合转换为数组
<T> T[] toArray(T[] a);                    // 将集合转换为给定类型的数组并将该数组返回
boolean add(E e);                          // 向集合中添加元素
boolean remove(Object o);                  // 从集合中移除给定的元素
void clear();                              // 清除集合中的元素
boolean containsAll(Collection<?> c);      // 判断集合中是否包含给定的集合中的所有元素
boolean addAll(Collection<? extends E> c); // 将给定的集合的所有元素添加到集合中
```
## AbstractCollection
`AbstractCollection`实现了`Collection`接口，属于单列集合的顶层抽象类。

源码解读:

`AbstractCollection`类并没有定义存储元素的容器，因此，其核心的方法都是空实现。这些空实现的方法都交给其子类来实现。

例如
```java
public abstract class AbstractCollection<E> implements Collection<E> {
    public abstract Iterator<E> iterator();
    
    public abstract int size();

    public boolean add(E e) { // 子类需要重写
        throw new UnsupportedOperationException();
    }
}
```
