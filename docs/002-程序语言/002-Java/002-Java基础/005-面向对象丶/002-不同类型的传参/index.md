# java不同类型的传参
## 基本类型
传递的是该类型的拷贝

## 引用类型
传递的是该类型的引用

<b style="color:red">引用数据类型传值时传递的是对象在堆内存上的空间地址</b>

- 对于引用类型（类、接口、数组等）的参数传递，在Java中并不会进行拷贝构造，实际上仍然是传递的是引用的副本。这意味着在方法内部对于对象的修改会影响原始对象，因为它们引用的是同一个对象实例。所以，与C++中的拷贝构造不同，Java中对于引用类型的参数传递仍然是“按引用传递”。

- 但值得注意的是 `String` 是不可变类型(Immutable)

例如:
```java
class text_1 {
    public static void setStr(String str) {
        str = new String("张三");
    }

    public static void setCat(Cat cat) {
        cat.setName("???");
    }
}

public class demo_5 {
    public static void main(String[] arg) {
        Cat cat = new Cat(3);
//        cat.catId = 112;
        cat.setName("猫子");
        cat.printName();

        String str = new String("李四");
        System.out.print("\n" + str);
        text_1.setStr(str); // 仍然是输出 李四
        System.out.println("\n" + str);

        text_1.setCat(cat); // 成功修改名字为 ???
        cat.printName();
    }
}
// 对于String类型的参数传递，在方法setStr中尝试修改参数str的值并不会影响原始的String对象。
// 这是因为在Java中，String是不可变的（Immutable）对象，一旦创建后就不能被修改。
// 当你调用new String("张三")时，实际上是创建了一个新的String对象，而原始的str仍然指向之前的对象"李四"，并没有改变。
```

实际上不止`String`是`Immutable`哦~