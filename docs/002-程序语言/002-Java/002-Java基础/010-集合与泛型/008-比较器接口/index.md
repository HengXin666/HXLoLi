# 比较器接口
## 比较器接口的作用
在使用数组或者集合时，我们经常都会遇到排序问题，比如将学生信息按照学生的成绩从高到低依次排列。数字能够直接比较大小，对象不能够直接比较大小，为了解决这个问题，Java 平台提供了`Comparable`和`Comparator`两个接口来解决。

## Comparable 接口
> 接口对实现该接口的每个类的对象强加了总体排序。 此排序称为类的自然排序，而该类的`compareTo`方法被称为其自然比较方法

示例:

```java
import java.util.Arrays;

class User implements Comparable<User> {
    private String name;
    private int level;  // 用户等级

    User(String name, int level) {
        this.name = name;
        this.level = level;
    }

    @Override
    public String toString() {
        return "User{" +
                "name='" + name + '\'' +
                ", level=" + level +
                '}';
    }

    @Override
    public int compareTo(User o) { // 类似于 C++20 的 <=>
//        if (o.level == this.level)
//            return 0;
//        else if (o.level > this.level)
//            return -1;
//        return 1;
        return this.level - o.level; // 等价
        // 不用记忆, 反正顺序乱了就把符号变一下
    }
}

public class ComparableText {
    public static void main(String[] args) {
        User[] arr = new User[]{
                new User("张三", 4),
                new User("老六", 2),
                new User("老八", 3),
                new User("木子", 6),
        };

        // Collections.sort (如果是List则使用这个排序)
        Arrays.sort(arr);

        for (User it : arr)
            System.out.println(it.toString());
    }
}
```

## Comparator 接口
> 比较功能，对某些对象集合施加总排序。 可以将比较器传递给排序方法（例如`Collections.sort`或`Arrays.sort`），以实现对排序顺序的精确控制。

示例:

```java
import java.util.Arrays;
import java.util.Comparator;

class User {
    private String name;
    private int level;  // 用户等级

    User(String name, int level) {
        this.name = name;
        this.level = level;
    }

    public int getLevel() {
        return level;
    }

    @Override
    public String toString() {
        return "User{" +
                "name='" + name + '\'' +
                ", level=" + level +
                '}';
    }

    // 删除了 Comparable 接口
}

public class ComparableText {
    public static void main(String[] args) {
        User[] arr = new User[]{
                new User("张三", 4),
                new User("老六", 2),
                new User("老八", 3),
                new User("木子", 6),
        };

        // 匿名类实现接口
        Comparator<User> c = (o1, o2) -> {
            return o1.getLevel() - o2.getLevel();
        }; // 值得注意的是, 需要提供get方法, 因为已经不是在类内部了

        // Collections.sort (如果是List则使用这个排序)
        Arrays.sort(arr, c); // 指定c这个比较的方法

        for (User it : arr)
            System.out.println(it.toString());
    }
}
```

<span style="color:red">`Comparable`接口是有数组或者集合中的对象的类所实现，实现后对象就拥有比较的方法，因此称为**内排序**或者**自然排序**。`Comparator`接口是**外部**提供的对两个对象的比较方式的实现，对象本身并没有比较的方式，因此被称为**外排序器**</span>