# java多态
多态性的字典定义是指在生物学原理，其中的生物体或物质可具有许多不同的形式或阶段。 该原理也可以应用于面向对象的编程和Java语言之类的语言。 一个类的子类可以定义自己的独特行为，但可以共享父类的某些相同功能。

从上面的描述中我们可以得出：继承、接口就是多态的具体体现方式。多态主要体现在类别、做事的方式上面。多态是面向对象的三大特征之一，多态分为编译时多态和运行时多态两大类。

## 编译时多态
方法重载在编译时就已经确定如何调用，因此`方法重载`属于编译时多态。

## 运行时多态
Java虚拟机（JVM）为每个变量中引用的对象调用适当的方法。 它不会调用由变量类型定义的方法。

这种行为称为虚拟方法调用，它说明了Java语言中重要的多态性特征的一个方面。


*在这段代码中，虽然通过`A b = new B();`创建了一个B类的实例，但是变量b的类型是A，所以在调用`b.show()`方法时会根据变量类型A去找方法。由于Java是动态绑定（动态分派）的，所以实际上会调用对象的实际类型B中的`show()`方法，而不是变量的类型A中的`show()`方法。因此无论是`((A)(b)).show();`还是`b.show();`都会输出"The B"。*
```java
package demo2;

class A {
    public void show() {
        System.out.print("The A");
    }
}

class B extends A {
    @Override
    public void show() {
        System.out.print("The B");
    }
}

public class Text {
    public static void main(String[] args) {
        A b = new B();
        ((A)(b)).show(); // The B
        b.show();        // The B
    }
}
```

区别: *在 C++ 中，对于非虚函数，函数的调用是根据变量的静态类型确定的，而不是根据实际对象的类型。因此，在 C++ 中，无论如何进行类型转换，都会调用变量的类型的函数。*

## 多态的用处
场景 有一个抽象类`动物`, 其子类有`吗喽`,`猩猩`, `猴子`, 有一个类`ZooOP`(动物园管理员)

管理员可以喂动物, 比如:

```java
public class ZooOP {
    public void 喂吗喽(吗喽Class 吗喽) {
        吗喽.eat();
    }
}
```

如果有很多个动物, 那么`ZooOP`类就要写很多`喂xx`的方法, 显然这不好.

故利用多态, 可以写一个`喂动物`的方法:

```java
public class ZooOP {
    public void 喂动物(动物Class 动物) {
        动物.eat();
    }
}
```

这样不就更好了吗?

# instanceof 运算符
instanceof 本身意思表示的是什么什么的一个实例。主要应用在类型的强制转换上面。在使用强制类型转换时，如果使用不正确，在运行时会报错。而 instanceof 运算符对转换的目标类型进行检测，如果是，则进行强制转换。这样可以保证程序的正常运行。

语法
```java
对象名 instanceof 类名; //表示检测对象是否是指定类型的一个实例。返回值类型为boolean类型
```

示例:
```java
package demo2;

class A {

}

class B extends A {

}

public class Text {
    public static void main(String[] args) {
        A b = new B();
        if (b instanceof B)
            System.out.print(1); // 1
    }
}
```