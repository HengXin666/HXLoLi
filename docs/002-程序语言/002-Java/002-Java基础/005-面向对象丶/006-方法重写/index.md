# 方法重写
子类中的一个成员方法与父类中的成员方法有相同的签名（方法名加上参数数量和参数类型）和返回值类型的实例方法重写了父类的方法

## 如何使用方法重写
子类重写方法的能力使类可以从行为“足够近”的父类继承，然后根据需要修改行为。重写方法与被重写的方法具有相同的名称，数量和参数类型，并且返回类型相同。重写方法还可以返回重写方法返回的类型的子类型。 此子类型称为`协变`返回类型。

重写方法时，您可能需要使用`@Override`注解，该注释指示编译器您打算重写父类中的方法。 如果由于某种原因，编译器检测到该方法在父类中不存在，则它将生成错误。

|##container##|||||||
|-|-|-|-|-|-|-|
|基本类型|byte|short|int|long|float|double|
|类 => 都是`Number`的子类|Byte|Short|Integer|Long|Float|Double|

<span style="color:red">重写方法时访问修饰符的级别不能降低。</span>

示例:

```java
package ord.HX.text;

public class Geometry {
    /**
     * 计算面积
     * @return 面积
     */
    protected Number area() {
        return 0;
    }
}

class Round extends Geometry {
    private int radius;

    public Round(int radius) {
        this.radius = radius;
    }

    @Override // 重写, 写这个会让编译器检查是否重写成功: 如果父类没有这个函数就会报错
    /* 可以重写访问修饰符, 级别: private(不能重写私有因为不继承) < 默认(受包保护) < protected < public */ public Double area() {
        return this.radius * this.radius * Math.PI;
    }
}
```