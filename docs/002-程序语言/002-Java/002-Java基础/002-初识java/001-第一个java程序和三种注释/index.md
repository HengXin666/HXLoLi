# 第一个java程序
## 代码

创建一个 `MyCat.java` 的文件, 编写以下内容:

<b style="color:red">使用public修饰的class，该class的名字必须与文件名保持一致</b>

```java
public class MyCat {
    public static void main(String[] args) {
        System.out.println("我是cat");
    }
}
```

编译: (指定编译器版本, 编码 等等)
```cmd
javac -encoding utf-8 -source 1.8 -target 1.8 MyCat.java
```

运行:
```cmd
java MyCat
```

## 注释

### 单行注释

主要应用于单行代码上，对该行代码进行解释说明

```java
// 单行
```

### 多行注释

主要应用于方法或者类上面，对方法的作用或者类的作用进行解释说明

```java
/*
 *
 */
```

### 文档注释

主要应用于方法或者类上面，对方法的作用或者类的作用进行解释说明，方便生成帮助文档

```java
/**
 * 多了一颗星星
 */
```
