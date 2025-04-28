# java Object 类常用方法
Object类中定义的方法大多数都是属于`native`方法，`native`表示的是本地方法，实现方式是在C++中。

## getClass()

```java
public class Object {
    public final native Class<?> getClass();
}

// getClass() 方法返回一个Class对象，该对象具有可用于获取有关该类的信息的方法，
// 例如其名称(getSimpleName())，其超类(getSuperclass())及其实现的接口(getInterfaces())。
```

示例:
```java
package demo2;

interface JK {

}

class A implements JK {
    public void show() {
        System.out.print("The A");
    }
}

/* 注意:
在Java中，只有在一个类显式地实现了某个接口时，该类才被认为是实现了该接口。
如果一个类的父类实现了接口，但子类没有显式地实现该接口，那么子类并不被视为实现了该接口。
因此，在我的代码中，类B并没有实现接口JK，所以在for循环中不会输出接口信息。
*/
class B extends A /* implements JK */ {
    @Override
    public void show() {
        System.out.print("The B");
    }
}

public class Text {
    public static void main(String[] args) {
        A b = new B();
        Class clazz = b.getClass();
        System.out.println(clazz.getSimpleName());   // 获取类名
        System.out.println(clazz.getClass());        // 获取类的全限定名
        Class clazz_f = clazz.getSuperclass();       // 获取父类的定义信息
        System.out.println(clazz_f.getSimpleName()); // 获取父类名
        System.out.println(clazz_f.getClass());      // 获取父类的全限定名
        // --- 输出接口信息 --- (实现的接口不一定为一个, 所以需要使用数组)
        Class[] clazz_i = clazz.getInterfaces();
        for (int i = 0; i < clazz_i.length; ++i) {
            System.out.println(clazz_i[i].getSimpleName());
            System.out.println(clazz_i[i].getName());
        }
    }
}
```

## hashCode()
```java
public int hashCode()
// hashCode() 返回值是对象的哈希码，即对象的内存地址（十六进制）。

// 根据定义，如果两个对象相等，则它们的哈希码也必须相等。
// 如果重写equals() 方法，则会更改两个对象的相等方式，并且Object的hashCode() 实现不再有效。
// 因此，如果重写equals() 方法，则还必须重写hashCode() 方法。
```
`Object`类中的`hashCode()`方法返回的就是对象的内存地址。一旦重写`hashCode()`方法，那么`Object`类中的`hashCode()`方法就是失效，此时的`hashCode()`方法返回的值不再是内存地址。

示例:

```java
class Aa {
    // hashCode()方法被重写之后，返回的值就不在是对象的内存地址
    @Override
    public int hashCode() {
        // return super.hashCode();
        return 1;
    }
}
```

## equals(Object obj)
```java
public boolean equals(Object obj) {
    return (this == obj);
}
// equals()方法比较两个对象是否相等，如果相等则返回true。
// Object类中提供的equals()方法使用身份运算符（==）来确定两个对象是否相等。
// 对于原始数据类型，这将给出正确的结果。
// 但是，对于对象，则不是。 Object提供的equals()方法测试对象引用是否相等，即所比较的对象是否完全相同。

// 要测试两个对象在等效性上是否相等（包含相同的信息），必须重写equals()方法。
```

示例:
```java
package demo2;

import java.util.Objects;

class Aa {
    private String name;
    private int id;

    Aa(String name, int id) {
        this.name = name;
        this.id = id;
    }

    @Override
    public int hashCode() {
        // return super.hashCode();
        return name.hashCode() + id;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) // 为自己的引用(显然字段相等)
            return true;
        if (o == null || this.getClass() != o.getClass()) // 这个o类的类型不是this类
            return false;
        Aa aa = (Aa) o; // 如果类型一致, 那么就可以强转
        return id == aa.id && Objects.equals(name, aa.name); // 判断所有的字段是否相等
    }
}

public class Text2 {
    public static void main(String[] args) {
        Aa a = new Aa("张三", 1);
        Aa b = new Aa("张三", 1);

        // 即 == 引用类型, 只能判断内存地址是否相同: a, b指针是否指向同一个堆内存
        System.out.println(a == b); // 只是判断 a b 是不是 同一个引用
        /*
        即: Aa a = new Aa("张三", 1);
            Aa b = a;
            System.out.println(a == b); // 为 true
         */
        System.out.println(a.equals(b));

        // 依照Java的定义, 重写后的方法需要保证:
        // 如果equals为真, 那么hashCode需要相同
        System.out.println(a.hashCode());
        System.out.println(b.hashCode());
    }
}
```

根据定义，如果两个对象相等，则它们的哈希码也必须相等，反之则不然。<br>
重写了equals方法，就需要重写hashCode方法，才能满足上面的结论

- 面试题：请描述 == 和 equals 方法的区别

    基本数据类型使用 == 比较的就是两个数据的字面量是否相等。引用数据类型使用 == 比较的是内存地址。`equals`方法来自Object类，本身实现使用的就是 ==，此时它们之间没有区别。但是Object类中的`equals`方法可能被重写，此时比较就需要看重写逻辑来进行。

## toString()
```java
public String toString() {
    return getClass().getName() + "@" + Integer.toHexString(hashCode());
}
// 你应该始终考虑在类中重写toString() 方法。

// Object的toString() 方法返回该对象的String表示形式，这对于调试非常有用。 
// 对象的String表示形式完全取决于对象，这就是为什么你需要在类中重写toString() 的原因。
```

示例:

```java
class Aa {
    private String name;
    private int id;

    Aa(String name, int id) {
        this.name = name;
        this.id = id;
    }

    @Override
    public String toString() {
        return "Aa{" +
                "name='" + name + '\'' +
                ", id=" + id +
                '}';
    }
}

public class Text2 {
    public static void main(String[] args) {
        Aa a = new Aa("张三", 1);
        System.out.println(a.toString());
    }
}
```

## finalize()

```java
protected void finalize() throws Throwable { }

// Object类提供了一个回调方法finalize()，当该对象变为垃圾时可以在该对象上调用该方法。
// Object类的finalize()实现不执行任何操作, 你可以覆盖finalize()进行清理，例如释放资源。
```

示例:

```java
package demo2;

import java.util.Objects;

class Aa {
    private String name;
    private int id;

    Aa(String name, int id) {
        this.name = name;
        this.id = id;
    }

    @Override
    protected void finalize() throws Throwable {
        this.name = null; // 如果重写, 推荐这样 (如果不理解, 没关系, gpt说根本就不建议这样...)
        System.out.println("资源已释放");
    }
}

public class Text2 {
    public static void main(String[] args) {
        Text2.text();
        System.gc(); // 建议java进行垃圾回收, 不是马上
        System.out.println("已经出作用域了~");
    }

    public static void text() {
        Aa a = new Aa("张三", 1);
    }
}
```

### 浅谈java的垃圾回收
在Java中，判断对象是否可被垃圾回收主要使用的是**垃圾回收器（Garbage Collector）** 进行的垃圾回收算法，而不是引用计数方法。

`引用计数`方法是一种垃圾回收的方法，它通过记录每个对象被引用的次数来判断对象是否可被回收。当对象被引用时，引用计数加1；当引用失效时，引用计数减1。当引用计数为0时，表示该对象没有被任何引用指向，可以被回收。

然而，在Java中并没有使用引用计数方法来进行垃圾回收。相反，Java使用的是基于**可达性分析（Reachability Analysis）** 的垃圾回收算法。该算法通过从一组称为"`GC Roots`"的根对象开始，追踪对象之间的引用链路，如果一个对象与`GC Roots`之间没有任何引用链路相连，则认为该对象是不可达的，即无法通过程序访问到该对象，可以被垃圾回收器回收。

Java的垃圾回收器会**定期**扫描内存，找出所有不可达的对象，并回收它们所占用的内存空间。具体的垃圾回收策略和算法可能因JVM实现的不同而有所差异，但都是基于可达性分析的原理。

总结起来，Java中判断对象是否可被垃圾回收是通过可达性分析算法来实现的，而不是使用引用计数方法。<sup>By GPT-3.5</sup>