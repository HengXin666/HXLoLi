#  StringBuilder 和 StringBuffer
## 特性介绍
- StringBuilder 类位于 java.lang 包中，无需引入，直接使用即可。
- StringBuilder 类是由 final 修饰的，表示 StringBuilder 是一个最终类，不可能被继承
- StringBuilder 类构建的对象，可以实现字符序列的追加，但不会产生新的对象，只是将这个字符序列保存在字符数组中。


StringBuilder 和 StringBuffer 几乎是一样(构造方法/方法)的,

但是 `StringBuffer` 专门写了`synchronized`, 即 操作的时候加上了`锁`, 用于多线程环境, 单线程使用下会比`StringBuilder`慢, 因为要上锁/解锁

## 构造方法


```java
public StringBuilder(); // 构建一个StringBuilder对象，默认容量为16
public StringBuilder(int capacity);// 构建一个StringBuilder对象并指定初始化容量
public StringBuilder(String str); // 构建一个StringBuilder对象，并将指定的字符串存储在其中
```

示例:
```java
public class Example1 {
    public static void main(String[] args) {
        StringBuilder sb1 = new StringBuilder(); // 构建了一个初始化容量为16的字符串构建器
        StringBuilder sb2 = new StringBuilder(1024);// 构建了一个初始化容量为1024的字符串构建器
        StringBuilder sb3 = new StringBuilder("超用心在线教育");
    }
}
```

## 常用方法
### 追加
```java
public StringBuilder append(String str);      // 将一个字符串添加到StringBuilder存储区
public StringBuilder append(StringBuffer sb); // 将StringBuffer存储的内容添加StringBuilder存储区

/* 返回的是this 可以链式调用 */
@Override
public StringBuilder append(String str) {
    super.append(str);
    return this;
}
```

StringBuilder [建造者模式](../../../../../001-计佬常識/002-设计模式/007-创建型模式/005-建造者模式/index.md) 类似于(`Builder`)

示例:
```java
public class Example2 {
    public static void main(String[] args) {
        StringBuilder sb = new StringBuilder(1024);
        sb.append("超用心在线教育");
        sb.append(1);
        sb.append(1.0);
        sb.append(true);
        sb.append('a');
        System.out.println(sb);
        StringBuffer buffer = new StringBuffer(1024);
        buffer.append("超用心在线教育");//synchronized
        buffer.append(1);
        buffer.append(1.0);
        buffer.append(true);
        buffer.append('a');
        System.out.println(buffer);
        sb.append(buffer);
        System.out.println(sb);
        StringBuilder sb1 = new StringBuilder(1024);
        sb1.append("超用心在线教育").append(1).append(1.0).append(true).append('a');
        System.out.println(sb1);
    }
}
```

### 删除指定区间存储的内容
```java
public StringBuilder delete(int start, int end); // 将StringBuilder存储区指定的开始位置到指定的结束位置之间的内容删除掉
```

### 删除存储区指定下标位置存储的字符
```java
public StringBuilder deleteCharAt(int index)
```

示例:
```java
public class Example3 {
    public static void main(String[] args) {
        StringBuilder builder = new StringBuilder("abcdefg");
        builder.delete(1, 5); // [1, 5)
        System.out.println(builder);
        builder.deleteCharAt(0);
        System.out.println(builder);
    }
}
```

### 在StringBuilder存储区指定偏移位置处插入指定的字符串
```java
public StringBuilder insert(int offset, String str);
```

### 将存储区的内容倒序
```java
public StringBuilder reverse();
```

### 获取指定字符串在存储区中的位置
```java
public int indexOf(String str);     // 获取指定字符串在存储区中第一次出现的位置
public int lastIndexOf(String str); // 获取指定字符串在存储区中最后一次出现的位置
```

示例:

```java
public class Example4 {
    public static void main(String[] args) {
        StringBuilder builder = new StringBuilder("admin");
        builder.reverse(); // 倒序
        System.out.println(builder);
        builder.insert(2,","); // 在偏移量后面位置插入一个字符串
        System.out.println(builder);
        // 需要注意的是：length()方法返回的是char[]中使用的数量
        System.out.println(builder.length());
        StringBuilder sb = new StringBuilder("abababa");
        int index1 = sb.indexOf("ab"); // 获取字符串第一次出现的位置
        int index2 = sb.lastIndexOf("ab"); // 获取字符串最后一次出现的位置
        System.out.println(index1);
        System.out.println(index2);
    }
}
```

## 练习

```java
import java.util.Scanner;

public class Text5 {
    public static Scanner sc = new Scanner(System.in);

    public static void main(String[] args) {
        // 现有字符串 ababababababababa ，求其中子字符串 aba 出现的次数（使用String类完成）
        {
            String str = "ababababababababa";
            int c = 0;
            for (int i = 0; i <= str.length() - 3; ++i) {
                if ("aba".equals(str.substring(i, i + 3))) {
                    ++c;
                }
            }
            System.out.println(c);
        }

        // 将从控制台输入的数字转换为财务数字(10005.25 -->)(10,005.25)(使用 StringBuilder 完成)
        // 1. 找到 . 的位置, 然后偏移往前, 插入 .
        {
            StringBuilder str = new StringBuilder(sc.next());
            int dian_wz = str.length();
            for (int i = str.length() - 1; i >= 0; --i) {
                if (str.charAt(i) == '.') {
                    dian_wz = i;
                    break;
                }
            }

            if (dian_wz > 3) {
                for (int i = dian_wz - 3; i > 0; i -= 3) {
                    str.insert(i, ',');
                }
            }

            System.out.println(str);
        }
    }
}
```

## 对比 String
String 、 StringBuilder 和 StringBuffer 都是用来处理字符串的。在处理少量字符串的时候，它们之间的处理效率几乎没有任何区别。但在处理大量字符串的时候，由于 String 类的对象不可再更改，因此在处理字符串时会产生新的对象，对于内存的消耗来说较大，导致效率低下。而 StringBuilder和 StringBuffer 使用的是对字符串的字符数组内容进行拷贝，不会产生新的对象，因此效率较高。而StringBuffer 为了保证在多线程情况下字符数组中内容的正确使用，在每一个成员方法上面加了锁，有锁就会增加消耗，因此 StringBuffer 在处理效率上要略低于 StringBuilder 。