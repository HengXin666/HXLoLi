# Java类与对象
Java作为一种面向对象语言。支持以下基本概念：

- 多态
- 继承
- 封装
- 抽象
- 类
- 对象
- 实例
- 方法
- 重载

本节我们重点研究对象和类的概念。

- 对象：对象是类的一个实例（对象不是找个女朋友），有状态和行为。例如，一条狗是一个对象，它的状态有：颜色、名字、品种；行为有：摇尾巴、叫、吃等。
- 类：类是一个模板，它描述一类对象的行为和状态。

## 创建类

值得注意的是, 同一个文件.java只能用一个`public`修饰符的类, 并且类名就是文件名

```Java
修饰符 class 类名 {
  // 内容
}
```

## 构造函数/成员方法/变量
基本上同C++, 只不过需要在变量/方法的前面写上`修饰符`, 而不是`修饰符:`

### 示例

```java
// Cat.java中:
public class Cat {
    // 显式写构造函数, 那么默认的构造函数就不会被生成
    public Cat(int id) {
        this.catId = id;
        this.catName = null;
    }

    public String catName;

    int catId; // 注意没有修饰符的 默认的访问修饰符是包级别
               // 而在C++中默认的访问修饰符是私有

    public void setName(String name) {
        this.catName = name;
    }

    public void printName() {
        System.out.print(this.catId + " " + this.catName);
    }
}
```

```java
public class demo_5 {
    public static void main(String[] arg) {
        // java中实例化类就是 直接new一个
        Cat cat = new Cat(3); // 默认构造被删除, 所以 new Cat(); 是报错的
        cat.catId = 112;      // 可以修改
        cat.setName("猫子");
        cat.printName();
    }
}
```

## 方法重载
同C++ (函数重载([函数名] 一样, [参数列表, 返回值] 不同的))

## 源文件声明规则
当在一个源文件中定义多个类，并且还有import语句和package语句时，要特别注意这些规则。

- 一个源文件中<span style="color:red">只能有一个 public 类</span>
- 一个源文件可以有多个非 public 类

源文件的名称应该和 public 类的类名保持一致。

例如：源文件中 public 类的类名是 Employee，那么源文件应该命名为Employee.java。

如果一个类定义在某个包中，那么 package 语句应该在源文件的首行。

如果源文件包含 import 语句，那么应该放在 package 语句和类定义之间。如果没有 package 语句，那么 import 语句应该在源文件中最前面。

import 语句和 package 语句对源文件中定义的所有类都有效。在同一源文件中，不能给不同的类不同的包声明。

类有若干种访问级别，并且类也分不同的类型：抽象类和 final 类等。这些将在访问控制章节介绍。

除了上面提到的几种类型，Java 还有一些特殊的类，如：内部类、匿名类。