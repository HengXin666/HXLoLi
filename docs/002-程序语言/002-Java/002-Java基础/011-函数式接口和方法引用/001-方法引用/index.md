# 方法引用
## 应用场景
> 你使用lambda表达式创建匿名方法。但是，有时lambda表达式除了调用现有方法外什么也不做。在这种情况下，通常更容易按名称引用现有方法。方法引用使你可以执行此操作； 它们是紧凑的，对于已经具有名称的方法lambda表达式更易于阅读。

例如:

```java
interface Actor {
    void perform(String item);
}

public class Main {
    public static void main(String[] args) {
        Actor c = item -> System.out.println(item);
        c.perform("表演");
    }
}
```

**分析**

上面的示例中，Lambda表达式的作用就是调用 System.out 中的 println(String msg) 方法，这个方法已经有具体的实现，如果能够直接引用这个方法，那么代码将变得更为简洁。

<div style="margin-top: 80px;">

---
</div>

## 方法引用符
双冒号`::`为方法引用符，而它所在的表达式被称为方法引用。如果`Lambda`表达式赋值的方法已经在某个类中有具体的实现，那么则可以通过双冒号来引用该方法作为`Lambda`表达式的替代者。

示例

```java
public class Main {
    public static void main(String[] args) {
        Actor c = System.out::println; // 参数被自动推导了
        c.perform("表演");
    }
}
```

<span style="color:red">方法引用与`Lambda`表达式一样，只能应用于**函数式接口**。方法有静态方法、成员方法和构造方法之分，方法引用因此也分为静态方法引用、成员方法引用和构造方法引用</span>

<div style="margin-top: 80px;">

---
</div>

## 静态方法引用
语法:

```java
类名::方法名 // 实际上就是把 类名.方法名 的"."改为"::"
```

示例

```java
interface Actor {
    void perform(String item);
}

class Text{
    public static void fun (String str) {
        System.out.println(str);
    }
}

public class Main {
    public static void main(String[] args) {
        Actor c = Text::fun;
        c.perform("表演");
    }
}
```

<div style="margin-top: 80px;">

---
</div>

## 成员方法引用
语法:

```java
对象名::方法名
```

示例

```java
interface Actor {
    void perform(String item);
}

class Text{
    public void fun (String str) {
        System.out.println(str);
    }
}

public class Main {
    public static void main(String[] args) {
        Actor c = (new Text())::fun;
        c.perform("表演");
    }
}
```

<span style="color:red">**注意**: 如果函数式接口的抽象方法中只有一个引用数据类型的参数，且实现过程只需要调用该类型中定义的成员方法，那么可以使用静态引用的方式直接引用该成员方法</span>

例如: (注意fun为非static方法, 并且fun不用传参!)
```java
interface Actor {
    void perform(Text t);
}

class Text{
    public void fun () {
        System.out.println("纳尼?");
    }
}

public class Main {
    public static void main(String[] args) {
        Actor c = Text::fun;
        c.perform(new Text()); // 语法糖?! 相当于自动调用传参的对象的方法
    }
}
```

这样也可以: (多参数), 不过 Text t 要放在参数列表的第一个位置, 不然不行

```java
interface Actor {
    void perform(Text t, String s);
}

class Text{
    public void fun (String s) {
        System.out.println(s);
    }
}

public class Main {
    public static void main(String[] args) {
        Actor c = Text::fun;
        c.perform(new Text(), "awa!!!");
    }
}
```

### this 引用成员方法
语法

```java
this::方法名
```

示例

```java
interface ToRu {
    void toRu(String mono); // 撮る
}

/**
 * 摄像师
 * */
class KaMeRaSyaIn {
    private void kaCa(String mono) {
        System.out.println("摄像师给" + mono + "拍照");
    }

    public void playKaMeRa(String mono) {
        ToRu c = this::kaCa;
        c.toRu(mono);
    }
}

public class KaMeRaText {
    public static void main(String[] args) {
        KaMeRaSyaIn kk = new KaMeRaSyaIn();
        kk.playKaMeRa("猫猫");
    }
}
```

### super 引用父类成员方法
语法
```java
super::方法名
```

示例

```java
interface PlayAlgorithm {
    void playITHomo();
}

class CodeMen {
    void doAlgorithm() {
        System.out.println("刷算法题");
    }

    void doCode() {
        System.out.println("写代码");
    }
}

class CppCodeMen extends CodeMen {
    @Override
    void doCode() {
        System.out.println("写C++代码");
    }

    @Override
    void doAlgorithm() {
        PlayAlgorithm c = super::doAlgorithm;
        c.playITHomo();
    }
}

public class CodeMenText {
    public static void main(String[] args) {
        CppCodeMen cMen = new CppCodeMen();
        cMen.doAlgorithm();
    }
}
```

## 构造方法引用
语法

```java
类名::new
```

示例

```java
interface BuildCat {
    Cat Build(String name);
}

class Cat {
    private String name;
    Cat(String name) {
        this.name = name;
    }

    void nyanNyan() {
        System.out.println(this.name + "在にゃーにゃgiao~"); // 喵喵叫
    }
}

public class BuildText {
    public static void main(String[] args) {
        BuildCat c = Cat::new;
        Cat cat = c.Build("听泉");
        cat.nyanNyan();
    }
}
```
