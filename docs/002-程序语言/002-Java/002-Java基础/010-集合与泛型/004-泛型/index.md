# 泛型
## 什么是泛型
简言之，泛型在定义类，接口和方法时使类型（类和接口）成为参数。与方法声明中使用的更熟悉的形式参数非常相似，类型参数为你提供了一种使用不同输入重复使用相同代码的方法。区别在于形式参数的输入是值，而类型参数的输入是类型。

从上面的描述中可以看出：<span style="color:red">泛型就是一个变量，只是该变量只能使用引用数据类型来赋值，这样能够使同样的代码被不同数据类型的数据重用。</span>

## 与C++的比较

有点像C++的模版, 但又不完全是: [C++模板和Java泛型之间的不同](https://blog.csdn.net/coding_is_fun/article/details/81564512)

- Java的泛型的实现根植于“类型消除”这一概念。当源代码被转换成Java虚拟机字节码时，这种技术会消除参数化类型。
- 这点跟C++的模板截然不同。

    在C++中，模板本质上就是一套宏指令集，只是换了个名头，编译器会针对每种类型创建一份模板代码的副本。

    有个证据可以证明这一点：`MyClass<Foo>`不会与`MyClass<Bar>`共享静态变量。然而，两个`MyClass<Foo>`实例则会共享静态变量。

## 如何使用泛型

### 包含泛型的类定义语法

通常使用的泛型变量： E T K V

```java
访问修饰符 class 类名<泛型变量> {

}
```

### 包含泛型的接口定义语法

```java
访问修饰符 interface 接口名<泛型变量> {

}
```

### 方法中使用新的泛型语法

```java
访问修饰符 泛型变量 返回值类型 方法名(泛型变量 变量名,数据类型1, 变量名1,...,数据类型n, 变量名n){

}
```

### 示例
#### 使用泛型改造自定义集合MyCollection

```java
import java.util.AbstractCollection;
import java.util.Arrays;
import java.util.Iterator;

class _MyCollection<T> extends AbstractCollection<T> {
    private Object[] elements;
    private int size;
    public _MyCollection(){
        this(16);
    }
    public _MyCollection(int capacity){
        elements = new Object[capacity];
    }
    @Override
    public boolean add(T o) {
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
    public Iterator<T> iterator() {
        return new CollectionIterator();
    }
    @Override
    public int size() {
        return size;
    }
    class CollectionIterator implements Iterator<T> {
        private int cursor; //光标，实际上就是下标
        @Override
        public boolean hasNext() {
            return cursor < size;
        }
        @Override
        public T next() {
            if(cursor >= size || cursor < 0){
                throw new ArrayIndexOutOfBoundsException("下标越界了");
            }
            return (T)elements[cursor++];
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

class _MyCollectionTest {
    public static void text() {
        //在JDK7及以上版本，new 对象时如果类型带有泛型，可以不写具体的泛型。
        //在JDK7以下版本，在new 对象时如果类型带有泛型，必须写具体的泛型。
        _MyCollection<String> c2 = new _MyCollection<>();
        c2.add("admin");
        c2.add("user");
        for(Iterator<String> iter = c2.iterator(); iter.hasNext();){
            String s = iter.next();
            System.out.println(s);
        }
        //当创建MyCollection对象没有使用泛型时，默认是Object类型
        MyCollection c1 = new MyCollection();
        c1.add(5);
        c1.add(4.0);
        c1.add("3");
        c1.add(2.0f);
        c1.add(new Object());
        for(Iterator iter = c1.iterator(); iter.hasNext();){
            Object i = iter.next();
            System.out.println(i);
        }
    }
}

public class HXG {
    public static void main(String[] args) {
        _MyCollectionTest.text();
    }
}
```

## 泛型通配符
当使用泛型类或者接口时，如果泛型类型不能确定，可以通过通配符`?`表示。例如`Collection`接口中的约定:

```java
boolean containsAll(Collection<?> c); // 判断集合是否包含给定集合中的所有元素
```

- 泛型使用通配符的集合不能存储数据，只能读取数据
```java
// 泛型使用通配符的集合不能存储数据，只能读取数据
MyCollection<?> c3 = new MyCollection<>();
// c3.add(1);
// c3.add("");
```

## 泛型上限
语法:

```java
<? extend 数据类型>
```

在使用泛型时，可以设置泛型的上限，表示只能接受该类型或其子类。当集合使用泛型上限时，因为编译器只知道存储类型的上限，但不知道具体存的是什么类型，因此，该集合不能存储元素，只能读取元素

示例:

```java
public class GenericUpperLimit {
    public static void main(String[] args) {
        // 定义了一个泛型上限为Number的集合 Integer Double Short Byte Float Long
        MyCollection<? extends Number> c = new MyCollection<>();
        // 添加元素时，使用的是占位符? extends Number来对泛型变量进行替换，当传入参数时，
        // 无法确定该参数应该怎么来匹配? extends Number，因此不能存入数据
        // c.add((Integer)1);
        // c.add(1.0);
    }
}
```

## 泛型下限
语法:

```java
<? super 数据类型>
```

在使用泛型时，可以设置泛型的下限，表示只能接受该类型及其子类或该类型的父类。当集合使用泛型下限时，因为编译器知道存储类型的下限，至少可以将该类型对象存入，但不知道有存储的数据有多少种父类，因此，该集合只能存储元素，不能读取元素。

示例:
```java
public class GenericLowerLimit {
    public static void main(String[] args) {
        // 集合中存储元素的类型可以是Number的子类、Number类、Number的父类
        MyCollection<? super Number> c = new MyCollection<>();
        // 虽然存储元素的类型可以是Number的父类，但是由于父类类型无法确定具体多少种，
        // 因此在使用添加功能时，编译器会报错
        // c.add(new Object());
        // 但是集合中可以存储Number类
        c.add(1.0);
    }
}
```

### 示例

```java
class GTxt<T> {
    public <T extends Number> void put(T t) { // 可以传入 T 类型的父类, 最父可以父到Number类
        System.out.println(t);
    }
    
//    报错, super不能读取!, 只能存放
//    public  <T super Number>  void show(T t) {
//        System.out.println(t);
//    }
}

public class TextG {
    public static void main(String[] args) {
        GTxt<Integer> g = new GTxt<>();
        g.put(1.1);
//      g.put(new Object()); // 不对, 类已经超过Number类了
    }
}
```
