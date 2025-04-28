# 抽象类
抽象类是被声明为`abstract`的类, 它可能包含也可能不包含抽象方法。 抽象类不能被实例化，但是可以被子类化（也就是可以被继承）。

## 语法
```java
// 抽象类
访问修饰符 abstract class 类名 {

    // 抽象方法
    访问修饰符 abstract 返回值类型 方法名([参数列表]);
}
```


示例
```java
abstract class Aa {
    //抽象方法是没有方法体"{}"的。因为方法体就是表示知道具体怎么去做这件事情。
    public abstract void aToA();
}

class Bb extends Aa {

    // 子类必需实现抽象类的"所有"抽象方法
    @Override
    public void aToA() {
        
    }

    // public abstract void bTob(); // 非抽象类不能声明抽象方法
}

abstract class Cc extends Aa {
    // 不实现 父类抽象方法, 那么只能是抽象子类 abstract
}
```

<span style="color:red">如果一个类继承于一个抽象类，那么该类必须实现这个抽象类中的所有抽象方法。否则，该类必须定义抽象类<br><br>
抽象类不一定有抽象方法，但有抽象方法的类一定是抽象类</span>
