# 包装类
## 什么是包装类
> 不论怎样，总有理由使用对象代替原始数据类型，并且Java平台为每种原始数据类型提供了包装类。这些类将“原始数据类型”包装在对象中。

|基本数据类型|包装类|基本数据类型|包装类|
|-|-|-|-|
|int|<b style="color:red">Integer</b>|char|<b style="color:red">Character</b>|
|byte|Byte|boolean|Boolean|
|short|Short|float|Float|
|long|Long|double|Double|

## 自动装箱和拆箱
> 通常，包装是由编译器完成的, 如果你在期望一个对象的地方使用原始数据类型，则编译器会为你将原始数据类型放入其包装类中。

自动装箱的方法

```java
包装类名.valueOf(原始数据类型的值);
```

示例
```java
// 变量num期望获取一个整数对象，但赋值时给定的是一个基本数据类型int值，此时编译器将会将int值5进行包装
// 调用的是Integer.valueOf(5)
Integer num = 5;
```

## 自动拆箱
> 类似地，如果在期望使用基本数据类型的情况下使用包装类型，则编译器会为你解包该对象。

自动拆箱方法

```java
包装类对象.xxxValue();
```

示例
```java
Integer num = new Integer(10);
// 变量a期望获取一个基本数据类型的值，但赋值时给定的是一个引用数据类型的对象，此时编译器会将这个引用数据类型
// 的对象中存储的数值取出来，然后赋值给变量a。调用的是num.intValue();
int a = num;
```

## 字符串转数字的方法

```java
Integer.parseInt("123");      // 将字符串类型的数字转换为整数
Long.parseLong("123");        // 将字符串类型的数字转换为长整数
Byte.parseByte("13");         // 将字符串类型的数字转换为字节
Short.parseShort("13");       // 将字符串类型的数字转换为短整数
Float.parseFloat("12.0f");    // 将字符串类型的数字转换为单精度浮点数
Double.parseDouble("123");    // 将字符串类型的数字转换为双精度浮点数
Boolean.parseBoolean("true"); // 将字符串类型的布尔值转换为布尔值
```

<b style="color:red">注意:如果字符串参数的内容无法正确转换为对应的基本类型，则会抛出`java.lang.NumberFormatException`异常。</b>