# 嵌套类
## 概念
> Java编程语言允许你在一个类中定义一个类。 这样的类称为嵌套类
>
> 嵌套类分为两类：静态和非静态。 声明为静态的嵌套类称为静态嵌套类。 非静态嵌套类称为内部类。
>
> 嵌套类是其外部类的成员。 非静态嵌套类（内部类）可以访问外部类的其他成员，即使它们被声明为私有的也可以访问。 静态嵌套类无权访问外部类的其他成员。 作为OuterClass的成员，可以将嵌套类声明为私有，公共，受保护或包私有。 （回想一下，外部类只能声明为公共或包私有。）

## 为什么要使用内部类
当一个事物内部还有其他事物时，使用内部类来描述就显得更加合理。比如计算机包含显卡、主板、CPU，此时就可以使用内部类来描述计算机。

```java
class Computer { // 电脑
    class Mainboarf { // 主板

    }

    class GPU {

    }

    class CPU {

    }
}
```

## 静态嵌套类
> 与类方法和变量一样，静态嵌套类与其外部类相关联。 与静态类方法一样，静态嵌套类不能直接引用其外部类中定义的实例变量或方法：它只能通过对象引用来使用它们。
>
> 静态嵌套类与它的外部类（和其他类）的实例成员进行交互，就像其他任何顶级类一样。 实际上，静态嵌套类在行为上是顶级类，为了包装方便，该顶级类已嵌套在另一个顶级类中。


```java
//静态嵌套类构建对象的语法:
外部类类名.内部类类名 变量名 = new 外部类类名.内部类类名()
```

```java
public class QtClass {
    public static void main(String[] args) {
        int n = 5;
        Student[] sArr = new Student[n];

        for (int i = 0; i < n; ++i) {
            String s = new String("我是");
            s += i;
            sArr[i] = new Student(s, i);
        }

        // 使用静态嵌套类 (实际上和普通的类没有区别 (只是需要.父嵌套类名而已))
        Student.Sort sss = new Student.Sort(sArr);
    }
}

class Student {
    private String name;
    private int id;

    Student(String name, int id) {
        this.name = name;
        this.id = id;
    }

    static class Sort {
        Sort(Student[] s) {
            // 无法使用Student的成员变量
            for (Student it : s) {
                System.out.printf("%s %d\n", it.name, it.id);
            }
        }
    }
}
```

## 内部类
> 与实例方法和变量一样，内部类与其所在类的实例相关联，并且**可以直接访问该对象的方法和字段**。 另外，由于内部类与实例相关联，因此它本身**不能定义任何静态成员**。

内部类对象创建语法

```java
外部类类名.内部类类名 对象名 = new 外部类类名().new 内部类类名();
```

示例:

```java
public class QtClass {
    public static void main(String[] args) {
        int n = 5;
        Student[] sArr = new Student[n];

        for (int i = 0; i < n; ++i) {
            String s = new String("我是");
            s += i;
            sArr[i] = new Student(s, i);
            sArr[i].getCat_2().setName("开" + i + "门");
//            sArr[i].getCat().show();
        }

        // 单独new成员 不可行
//        Student.Cat cat = new Student.Cat("听泉喵");
//        sArr[0].setCat(new Student.Cat("听泉喵")); // 传参也不行
        Student.Cat cat = new Student().new Cat("听泉喵");
//        cat.show();

        Student.Cat ccat = sArr[1].new Cat("大开门"); // 这个不是 cat 也不是 cat_2, 应该是个隐藏字段, 但关联顶级类是 sArr[1]

        System.out.println(sArr[1].hashCode());
        System.out.println(sArr[1].getCat_2().getStudent().hashCode());
        System.out.println(ccat.getStudent().hashCode());

        ccat.show();
        sArr[1].setCat(cat);     // 这个 cat 关联的 是 new Student().new Cat("听泉喵"); 的 Student()
        sArr[1].getCat().show(); // 可以证明!!!
        System.out.println(sArr[1].getCat().getStudent().hashCode());
        sArr[1].getCat_2().show();


//        Student.Sort sss = new Student.Sort(sArr);
    }
}

class Student {
    private String name;
    private int id;
    private Cat cat;
    private Cat cat_2;

    Student() {}
    Student(String name, int id) {
        this.name = name;
        this.id = id;
        this.cat_2 = new Cat("大开门");
    }

    public Cat getCat() {
        return cat;
    }

    public Cat getCat_2() {
        return cat_2;
    }

    public void setCat(Cat cat) {
        this.cat = cat;
    }

    // --- 嵌套内部类 ---
    public class Cat {
        private String name;

        public Cat(String name) {
            this.name = name;
        }

        public void setName(String name) {
            this.name = name;
        }

        public void show() {
            // 如果需要调用上层类的成员, 需要指定 上层类的类名.this.变量名
            // 而在本类, 默认可以省略: this.变量名 == 本类名.this.变量名
            System.out.printf("id 为 %d 的 %s 的猫猫是 %s\n", Student.this.id, Student.this.name, /*Cat.*/this.name);
        }

        public Student getStudent() {
            return Student.this;
        }
    }
}
```

## 局部内部类
> 局部类是在一个块中定义的类，该块是一组在平衡括号之间的零个或多个语句。通常，你会在方法的主体中找到定义的局部类。

同cpp的

```java
public class JBClass {
    public static void main(String[] args) {
        class JuBuClass {
            private int a;
            private int b;
            JuBuClass(int a, int b) {
                this.a = a;
                this.b = b;
            }

            int add() {
                return this.a + this.b;
            }
        }

        System.out.println((new JuBuClass(114, 514)).add());
    }
}
```

## 匿名内部类
> 匿名类可以使你的代码更简洁。它们使你在声明一个类的同时实例化它。除了没有名称外，它们类似于局部类。如果只需要使用一次局部类，则使用它们。
>
> 匿名类表达式的语法类似于构造方法的调用，不同之处在于代码块中包含类定义。

**相当于写了个局部类对new的类/接口继承, 并且可以重写方法, 但是这个基本类是匿名的.**

```java
public class NiMinClass {
    public static void main(String[] args) {
        Show s = new Show("张三", 233) {
            @Override
            void show() {
                System.out.println("我套你猴子的: " + name + " " + id);
            }
        };

        s.show();

        Print p = new Print() {
            @Override
            public void print() {
                System.out.println("114514");
            }
        };

        p.print();
    }
}

interface Print {
    void print();
}

class Show {
    protected String name;
    protected int id;

    Show(String name, int id) {
        this.name = name;
        this.id = id;
    }

    void show() {
        System.out.println(name + " " + id);
    }
}
```

当然, 也是可以作为返回值的:

```java
interface Print {
    void print();
}

public class NiMinClass {
    public static final Print fun() {
        return new Print() {
            @Override
            public void print() {
                System.out.println("114514");
            }
        };
    }
    
    public static void main(String[] args) {
        fun().print();
    }
}
```

值得注意的是, 局部类不能作为返回值: (不同于C++)

```java
// 下面代码是无法运行的, 报错的!!!
public class JBClass {
    public static final Object fun() { // 甚至返回值写 JuBuClass 类 是报错的
        class JuBuClass {
            private int a;
            private int b;
            JuBuClass(int a, int b) {
                this.a = a;
                this.b = b;
            }

            int add() {
                return this.a + this.b;
            }
        }
        
        return new JuBuClass(114, 514);
    }
    
    public static void main(String[] args) {
        System.out.println(((JuBuClass)fun()).add()); // 实际上无法访问JuBuClass
    }
}
```
