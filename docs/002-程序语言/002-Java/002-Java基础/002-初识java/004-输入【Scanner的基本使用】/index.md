# Scanner的基本使用
Scanner提供一种与用户交互的方式，用户可以在控制台输入一些数据，程序来获取这些数据
## 常用方法

| 方法名|解释说明|
|-|-|
| nextDouble() | 获取用户从控制台输入的浮点数，如果输入的不是数字，就会出错   |
| nextFloat()  | 获取用户从控制台输入的浮点数，如果输入的不是数字，就会出错   |
| nextInt()    | 获取用户从控制台输入的整数，如果输入的不是整数，就会出错     |
| nextByte()   | 获取用户从控制台输入的整数，如果输入的不是整数，就会出错     |
| nextShort()  | 获取用户从控制台输入的整数，如果输入的不是整数，就会出错     |
| nextLong()   | 获取用户从控制台输入的整数，如果输入的不是整数，就会出错     |
| nextBoolean()| 获取用户从控制台输入的boolean值，只能输入true或者false，否则就会出错 |
| next()       | 获取用户从控制台输入的字符串                                 |



## 示例

```java
import java.util.Scanner;

public class Main {
    public static void main(String[] args) {
        System.out.println("请输入name:");
        Scanner cin = new Scanner(System.in); // 固定写法不用纠结
        System.out.println(cin.next());
    }
}
```

使用 `空格/换行` 都是截断输入 同 scanf("%s %s");

```java
import java.util.Scanner;

public class Main {
    public static void main(String[] args) {
        Scanner cin = new Scanner(System.in);
        System.out.println("请输入name:");
        System.out.println(cin.next());
        System.out.println(cin.next());
    }
}
```

对应C++代码: *也就是在cin.类方法的时候才进行一次输入, `Scanner cin = new Scanner(System.in);`只是声明(类似于输入的数据暂时存放到了`Scanner`类的`cin`对象)*
```C++
int main() {
    char str[1024];
    cout << "请输入name" << endl;
    scanf("%s", str); 
    printf("%s\n", str);
    scanf("%s", str);
    printf("%s\n", str);
    return 0;
}
```
