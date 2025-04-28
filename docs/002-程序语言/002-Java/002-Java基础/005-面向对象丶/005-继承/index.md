# 继承
<b style="color:yellow">Java中所有类都隐式的继承了`Object`类</b><sup>[0]</sup>

继承的概念很简单但是很强大：当你要创建一个新类并且已经有一个包含所需代码的类时，可以从现有类中派生新类。这样，你可以重用现有类的字段和方法，而不必自己编写（和调试！）它们。

子类从其父类继承所有成员（字段，方法和嵌套类）。构造方法不是成员，因此它们不会被子类继承，但是可以从子类中调用父类的构造方法。

从一个类派生的类称为子类（也可以是派生类，扩展类或子类）。派生子类的类称为超类（也称为基类或父类）。

## 语法
```java
class 子类名 extends 父类名 {

}
```

## 继承的属性和方法
不论子类在什么包中，子类会继承父类中所有的公开的和受保护的成员（包括字段和方法）。如果子类和父类在同一个包中，子类也会继承父类中受包保护的成员。

子类不会继承父类中定义的私有成员。尽管如此，如果父类有提供公开或者受保护的访问该字段的方法，这些方法也能在子类中被使用。<sup>[1]</sup>


由[1]可以推出: 如果 A 被 B 继承, 那么 **sizeof(B) >= sizeof(A) 恒成立**, 即不论A中的变量被什么修饰符修饰(比如私有的`private`), 但实际上是存在于子类的, 只是子类不能访问, 只能通过父类访问.

## 子类与父类
如果一个对象赋值给其父类的引用，此时想要调用该对象的特有的方法，必须要进行强制类型转换

示例:

父类
```java
package ord.HX.text;

public class Doputsu {
    private String name;

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    protected int id;
}
```


子类
```java
package ord.HX.text;

public class Cat extends Doputsu {
    private String gosujin;

    public String getGosujin() {
        return gosujin;
    }

    public void setGosujin(String gosujin) {
        this.gosujin = gosujin;
    }

    public void text() {
        System.out.print("Zz...");
    }
}
```


main
```java
package ord.HX.text;

public class Text {
    public static void main(String[] args) {
        Doputsu dongWu = new Cat();
        dongWu.setName("猫");
        System.out.println(dongWu.getName());

        ((Cat)dongWu).text(); // 强转才可以使用
    }
}
```

## [0] 万物皆对象
除了没有父类的Object之外，每个类都有一个且只有一个直接父类（单继承）。在没有其他任何显式超类的情况下，每个类都隐式为Object的子类。

类可以派生自另一个类，另一个类又可以派生自另一个类，依此类推，并最终派生自最顶层的类Object。据说这样的类是继承链中所有类的后代，并延伸到Object。

所有类都是Object的子类，因此，创建对象时都需要调用Object类中的无参构造方法，而Object本身就表示对象，因此创建出来的都是对象。