# 函数式接口
## 什么是函数式接口
> 函数式接口是仅包含一种抽象方法的任何接口。(一个函数式接口可能包含一个或多个默认方法或静态方法。) 由于一个函数式接口仅包含一个抽象方法，因此在实现该方法时可以省略该方法的名称。

例如, 就是一个函数式接口(只有一个接口需要被实现):

```java
interface Fun {
    void fun();

    default void xxx() {}
    static void yyy() {}
}
```

JDK8 专门为函数式接口提供了一个注解标识`@FunctionalInterface`，该注解只能使用在接口类型的定义上，表明这是一个函数式接口，<span style="color:red">编译器在编译时就是会对该接口进行检测：接口中是否只有一个抽象接口方法。如果有多个抽象接口方法或者一个抽象接口方法也没有，则将报编译错误</span>

即

```java
@FunctionalInterface
interface Fun {
    void fun();

    default void xxx() {}
    static void yyy() {}
    // 也可以加上私有的
}
```

## 函数式编程
函数式编程是一种编程方式，在 Java 中，简单来说就是一个变量能够存储一个函数。而能够实现这种赋值操作的只有 Lambda 表达式

例如

```java
@FunctionalInterface
interface Fun {
    void fun();
}

public class FunText {
    public static void main(String[] args) {
        Fun c = () -> System.out.println("函数式接口, C++的std::function?!");
        c.fun();
    }
}
```

## Lambda 表达式延迟执行
**应用场景**

在某种条件下才会处理数据

示例

```java
@FunctionalInterface
interface Fun {
    void fun(String...strArr);
}

class PrintStrArr {
    private static void printf(String...strArr) {
        StringBuilder sb = new StringBuilder();
        for (String it : strArr)
            sb.append(it);
        System.out.println(sb);
    }

    public static void print(boolean doKa, String...strArr) {
        if (!doKa)
            return;
        Fun f = PrintStrArr::printf;
        f.fun(strArr);
    }
}

public class FunText {
    public static void main(String[] args) {
        PrintStrArr.print(false, "张三" + "是" + "法外狂徒"); // 不输出, 但是组装了字符串, 有资源的浪费!
        PrintStrArr.print(false, "张三", "是", "法外狂徒");   // 不输出, 也不组装, 没有资源浪费
        PrintStrArr.print(true, "张三", "是", "法外狂徒");    // 内部组装后输出
    }
}
```

<div style="margin-top: 80px;">

---
</div>

## Consumer 接口

```java
void accept(T t);//接收一个被消费的数据
```

**解释说明**

`Consumer`顾名思义就是消费者的意思。可以消费一个被接收到的数据，至于如何消费，就需要看这个接口被如何实现。

示例

```java
public class FunText {
    public static void main(String[] args) {
        Consumer<String> c = System.err::println;
        c.accept("Error awa -> qwq");

        Consumer<String> cc = System.out::println;
        cc.accept("啊");

        Consumer<String> ccc = cc.andThen(c);
        ccc.accept("の?"); // 会执行 cc 后 再 执行 c, 并且都是相同的传参
    }
}
```

<div style="margin-top: 80px;">

---
</div>

## BiConsumer 接口

```java
void accept(T t, U u);//接收两个被消费的数据
```

**解释说明**

`BiConsumer`也是一个消费者，只是这个消费者可以一次性消费两个数据（一般是键值对）。至于如何消费，就需要看这个接口被如何实现。

示例

```java
public class FunText {
    public static void main(String[] args) {
        BiConsumer<String, String> c = (s1, s2) -> System.out.println(s1 + " " + s2);
        c.accept("key", "val");
//        c.andThen() 同样也有这个
    }
}
```

<div style="margin-top: 80px;">

---
</div>

## Predicate 接口

```java
boolean test(T t);                                    // 检测是否满足条件
default Predicate<T> and(Predicate<? super T> other); // 条件之间的逻辑与衔接
default Predicate<T> negate();                        // 条件取反
default Predicate<T> or(Predicate<? super T> other);  // 条件之间的逻辑或衔接
```

**解释说明**

`Predicate`是条件的意思，可以检测给定数据是否满足条件，也可以与其他条件进行衔接。至于如何检测，就需要看这个接口被如何实现。

示例+练习

```java
import java.util.ArrayList;
import java.util.function.Predicate;

class Stu {
    private String name;
    private String sex;
    private int aeg;

    Stu(String name, String sex, int aeg) {
        this.name = name;
        this.sex = sex;
        this.aeg = aeg;
    }

    public String getName() {
        return name;
    }

    public String getSex() {
        return sex;
    }

    public int getAeg() {
        return aeg;
    }
}

public class LenSuText {
    public static void main(String[] args) {
        // 学生有姓名、性别和年龄。现有一个集合内存储有10名学生信息，请找出其中性别为男，年龄在20岁以上的学生，并在控制台进行输出
        ArrayList<Stu> arr = new ArrayList<>();
        arr.add(new Stu("张三", "男", 21));
        arr.add(new Stu("老八", "男", 31));
        arr.add(new Stu("老六", "男", 11));
        arr.add(new Stu("mm", "女", 14));
        arr.add(new Stu("ll", "女", 12));

        Predicate<Stu> tj_1 = stu -> stu.getAeg() >= 20;
        Predicate<Stu> tj_2 = stu -> stu.getSex().equals("男");
        Predicate<Stu> tj_3 = tj_1.and(tj_2);
        
        arr.forEach(stu -> {if (tj_3.test(stu)) System.out.println(stu.getName());});
        // 等价下面
//        for (Stu it : arr) {
//            if (tj_3.test(it))
//                System.out.println(it.getName());
//        }
    }
}
```

<div style="margin-top: 80px;">

---
</div>

## Function 接口

```java
R apply(T t); // 将一个对象转换为另一种数据类型的对象
default <V> Function<T, V> andThen(Function<? super R, ? extends V> after); // 复合转换
```

**解释说明**

`Function`是功能的意思，可以将一种数据类型的对象转换为另一种数据类型的对象，至于如何转换，就需要看这个接口被如何实现。

示例
```java
public class FunText {
    public static void main(String[] args) {
        Function<String, Integer> f = Integer::parseInt; // 将类型String 转为 Integer
        Integer i = f.apply("114514");
        System.out.println(i);
    }
}
```
