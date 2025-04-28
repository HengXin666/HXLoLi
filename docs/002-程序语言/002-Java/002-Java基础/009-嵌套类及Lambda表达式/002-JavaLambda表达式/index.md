# Lambda表达式
## 为什么要使用Lambda表达式
> 匿名类的一个问题是，如果匿名类的实现非常简单，比如仅包含一个方法的接口，则匿名类的语法可能看起来笨拙且不清楚。在这些情况下，你通常试图将功能作为参数传递给另一种方法，例如，当某人单击按钮时应采取什么措施。Lambda表达式使你能够执行此操作，将功能视为方法参数，或将代码视为数据。

示例:
```java
public interface Actor{
    void performance();//表演节目
}

public class Test {
    public static void main(String[] args) {
        Actor actor = new Actor() {
            @Override
            public void performance() {
                System.out.println("演员表演节目");
            }
        };
        actor.performance();//执行方法
    }
}
```

从上面的代码中可以看出：匿名内部类只是对`Actor`接口的实现，重点是强调其有表演节目的行为。如果能够直接将表演节目的行为赋值给`actor`对象的引用，使得`actor`对象的引用在调用接口方法时直接执行该行为，那么将大大节省代码的编写量。而Lambda表达式就能够实现这样的功能。

## Lambda表达式标准语法

```java
(参数类型1 变量名1,参数类型2 变量名2,...参数类型n 变量名n) -> {
    //代码块
    [return 返回值;]
};
```

示例:

```java
interface AddFun {
    int add(int a, int b);
}

public class lbdClass {
    public static void main(String[] args) {
        AddFun fun = (int a, int b) -> {
            return a + b;
        };

        System.out.println(fun.add(114, 514));
    }
}
```

<span style="color:red">Lambda表达式只能使用在只有一个接口方法的接口上。只有一个接口方法的接口称之为**函数式接口(Functional Interface)** </span>

## Lambda表达式省略规则

1. ()中的所有**参数类型可以省略** (因为接口处已经定义了类型, 可以自动推导)

2. 如果()中**有且仅有一个参数**，那么()可以省略

3. 如果{}中**有且仅有一条语句，那么{}可以省略**，这条语句后的**分号也可以省略**。如果这条语句是`return`语句，那么**`return`关键字也可以省略**

示例:
```java
interface AddFun {
    int add(int a, int b);
}

public class lbdClass {
    public static void main(String[] args) {
        AddFun fun = (a, b) -> a + b;

        System.out.println(fun.add(114, 514));
    }
}
```
