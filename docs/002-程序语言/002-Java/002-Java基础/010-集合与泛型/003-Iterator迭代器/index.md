# Iterator 迭代器
## 什么是迭代器
集合是用来存储元素的，存储元素的目的是为了对元素进行操作，最常用的操作就是检索元素。为了满足这种需要，JDK 提供了一个 Iterable 接口（表示可迭代的），供所有单列集合来实现。

```java
public interface Collection<E> extends Iterable<E>
```

可以看出，`Collection`接口是`Iterable`接口的子接口，表示所有的单列集合都是可迭代的。`Iterable`接口中有一个约定:

```java
Iterator<T> iterator(); // 获取集合的迭代器
```
因此所有单列集合必须提供一个迭代元素的迭代器。而迭代器 Iterator 也是一个接口。其中约定如下:

```java
boolean hasNext();     // 判断迭代器中是否有下一个元素
E next();              // 获取迭代器中的下一个元素
default void remove(); // 将元素从迭代器中移除，默认是空实现
```

可能你对迭代器还是没有印象。那么你可以对比如下场景来理解迭代器:

一位顾客在超市买了许多商品，当他提着购物篮去结算时，收银员并没有数商品有多少件，只需要看购物篮中还有没有下一个商品，有就取出来结算，直到购物篮中没有商品为止。收银员的操作就是一个迭代过程，购物篮就相当于一个迭代器。

## 自定义 Collection 集合

```java
import java.util.AbstractCollection;
import java.util.Arrays;
import java.util.Iterator;

class MyCollection extends AbstractCollection {
    private Object[] elements;
  
    private int size;
  
    public MyCollection(){
        this(16);
    }
  
    public MyCollection(int capacity){
        elements = new Object[capacity];
    }
  
    @Override
    public boolean add(Object o) {
    //数组中存储满了，数组需要扩容才能存储新的元素
        if(size == elements.length){
            //4 >> 1 0100 >> 1 => 010 = 2
            int length = elements.length + elements.length >> 1;
            elements = Arrays.copyOf(elements, length);
        }
        elements[size++] = o;
        return true;
    }
  
    @Override
    public Iterator iterator() {
        return new CollectionIterator();
    }
  
    @Override
    public int size() {
        return size;
    }
  
    class CollectionIterator implements Iterator {
        private int cursor; //光标，实际上就是下标
      
        @Override
        public boolean hasNext() {
            return cursor < size;
        }
      
        @Override
        public Object next() {
            if(cursor >= size || cursor < 0){
                throw new ArrayIndexOutOfBoundsException("下标越界了");
            }
            return elements[cursor++];
        }
      
        @Override
        public void remove() {
            if(cursor >= size || cursor < 0){
                throw new ArrayIndexOutOfBoundsException("下标越界了");
            }
            System.arraycopy(elements, cursor, elements, cursor-1, size -
                    cursor);
            if(cursor == size){//表示移除的是最后一个元素，光标就会向前移动一位
                cursor--;
            }
            size--;//存储个数减去1
        }
    }
}

class MyCollectionTest {
    public static void text() {
        MyCollection collection = new MyCollection();
        collection.add("a");
        collection.add("b");
        collection.add("c");
        collection.add("d");
        collection.add("e");
        System.out.println("集合大小："+ collection.size());
        //遍历方式一
        Iterator iterator = collection.iterator();
        while (iterator.hasNext()){
            String s = (String) iterator.next();
            System.out.println(s);
        }
        System.out.println("==============================");
        collection.remove("c");
        System.out.println("集合大小："+ collection.size());
        //遍历方式二
        for(Object o: collection){
            System.out.println(o);
        }
        boolean exists = collection.contains("c");
        System.out.println("集合中是否包含元素c：" + exists);
        MyCollection c = new MyCollection();
        c.add(5);
        c.add(4);
        c.add(3);
        c.add(2);
        c.add(1);
        //遍历方式三
        for(Iterator iter = c.iterator(); iter.hasNext();){
            Integer i = (Integer) iter.next();
            System.out.println(i);
        }
    }
}

public class HXCollection {
    public static void main(String[] args) {
        MyCollectionTest.text();
    }
}
```

上面的集合中，存储的都是字符串，在使用迭代器遍历元素时，将元素强制转换为字符串，这时程序能够正常运行。当集合中存储多种数据类型时，强制类型转换将出现异常。比如:

```java
MyCollection c1 = new MyCollection();
c1.add(5);
c1.add(4.0);
c1.add("3");
c1.add(2.0f);
c1.add(new Object());
for(Iterator iter = c1.iterator(); iter.hasNext();) {
    Integer i = (Integer) iter.next();
    System.out.println(i);
}

/* 改为如下就ok, 不过next的返回值是 <T>
for(Iterator iter = c1.iterator(); iter.hasNext();) {
    Object i = (Object) iter.next();
    System.out.println(i);
}
*/
```

运行程序时就将得到如下异常:
```java Error
Exception in thread "main" java.lang.ClassCastException: java.lang.Double cannot be cast to java.lang.Integer
    at MyCollectionTest.text(HXCollection.java:106)
    at HXCollection.main(HXCollection.java:114)
```

如果集合中只能存储同一种数据，那么这种强制类型转换的异常将得到解决。如何限制集合只能存储同一种数据呢？这就需要使用到泛型。