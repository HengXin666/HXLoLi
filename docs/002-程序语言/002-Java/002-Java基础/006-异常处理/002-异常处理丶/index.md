# 异常处理
## 如何处理异常
在Java中，异常的种类很多，如果每一种异常类型我们都需要去记住，这无疑是一件很困难的事。如果能够有一套机制来处理异常，那么将减少程序员在开发时的耗时。Java就提供了一套异常处理机制来出来异常。Java处理异常使用了5个关键字：throw 、throws、try、catch、finally

## throw 抛出异常
throw关键字只能在方法内部使用，throw关键字抛出异常表示自身并未对异常进行处理。

语法:

```java
throw 异常对象; //通常与if选择结构配合使用
```

示例:
```java
package demo2;

import java.util.Scanner;

public class MyError {
    public static void main(String[] args) {
        MyError.text();
    }

    public static final void text() {
        Scanner sc = new Scanner(System.in);
        int num_1 = sc.nextInt();
        int num_2 = sc.nextInt();
        if (num_2 == 0) {
            // ArithmeticException e = new ArithmeticException("不能tm的除个0!");
            // throw e
            throw new ArithmeticException("不能tm的除个0!");
        }
    }
}
```

## throws 声明可能抛出的异常类型
throws关键字只能应用在<b style="color:red">方法或者构造方法</b>的定义上对可能抛出的异常类型进行声明，自身不会对异常做出处理，由方法的调用者来处理。如果方法的调用者未处理，则异常将持续向上一级调用者抛出，直至main()方法为止，如果main()方法也未处理，那么程序可能因此终止.

语法:
```java
访问修饰符 返回值类型 方法名(参数列表) throws 异常类型1, 异常类型2, ..., 异常类型n {

}
```

示例:
```java
package demo2;

import java.util.InputMismatchException;
import java.util.Scanner;

public class MyError {
    public static void main(String[] args) {
        MyError.text();
    }

    /**
     * 除法测试
     * @throws ArithmeticException 不能除以0
     * @throws InputMismatchException 不能输入非数字
     */
    public static final void text() throws ArithmeticException, InputMismatchException {
        Scanner sc = new Scanner(System.in);
        int num_1 = sc.nextInt();
        int num_2 = sc.nextInt();
        if (num_2 == 0) {
            throw new ArithmeticException("不能tm的除个0!");
        }
    }
}
```

throws 可以声明方法执行时可能抛出的异常类型，但需要注意的是：**方法执行过程中只能抛出声明的异常类型的其中一个异常**。

## try-catch 捕获异常
`throw`和`throws`关键字均没有对异常进行处理，这可能会导致程序终止。在这种情况下，可以使用`try-catch`结构来对抛出异常进行捕获处理，从而保证程序能够正常运行。

语法:

```java
try {
    // 代码块
} catch (异常类型 异常对象名) {
    
}
```

其中`try`表示尝试的意思，尝试执行`try`结构中的代码块，如果执行过程中抛出了异常，则交给`catch`语句块进行捕获操作

示例:

```java
package demo2;

import java.util.InputMismatchException;
import java.util.Scanner;

public class Text3 {
    public static Scanner sc = new Scanner(System.in);

    public static void main(String[] args) {
        try {
            System.out.println(sc.nextInt());
        } catch (InputMismatchException e) {
            System.out.println("不能输入非int: " + e.getClass().getName());
            e.printStackTrace(); // 打印异常轨迹
            System.out.println("异常信息："+ e.getMessage());
        }
    }
}
```

### 捕获多异常
我们可以使用 `try-catch-...-catch` 来分别对不同的异常进行处理

可以把`catch`理解为`switch`的`case`

示例:
```java
package demo2;

import java.util.InputMismatchException;
import java.util.Scanner;

public class Text3 {
    public static Scanner sc = new Scanner(System.in);

    public static void main(String[] args) {
        try {
            System.out.println(sc.nextInt() / sc.nextInt());
        } catch (InputMismatchException e) {
            System.out.println("不能输入非int: " + e.getClass().getName());
            e.printStackTrace(); // 打印异常轨迹
            System.out.println("异常信息："+ e.getMessage());
        } catch (ArithmeticException e) {
            System.out.println("不能除以ZeRo: " + e.getClass().getName());
            e.printStackTrace(); // 打印异常轨迹
            System.out.println("异常信息："+ e.getMessage());
        }
        System.out.println("不会终止, 程序进行执行...");
    }
}
```

值得注意的是: <span style="color:red">当使用多个catch子句捕获异常时，如果捕获的多个异常对象的数据类型具有继承关系，那么父类异常不能放在前面。</span>

为什么?, 请考虑以下代码:

```java
int x = 101;
if (x > 100) {
    // ...
} else if (x > 10) {
    // ...
}
```

*else if 的代码永远都不可能执行*

一样的, 如果有父类, 那么就不会轮到后面的子类执行了!

## finally 语句
`finally`语句不能单独使用，必须与`try`语句或者`try-catch`结构配合使用，表示无论程序是否发生异常都会执行，主要用于释放资源。但如果在try语句或者catch语句中存在系统退出的代码，则 finally 语句将得不到执行。

系统退出:
```java
System.exit(0); // 系统正常退出 0-正常退出 非0-异常退出
System.exit(1); // 系统异常退出
```

语法:

```java
try {

}/*[ catch (... ...) {

} ]*/finally {

}
```

示例:
```java
package demo2;

import java.util.InputMismatchException;
import java.util.Scanner;

public class Text3 {
    public static Scanner sc = new Scanner(System.in);

    public static void main(String[] args) {
        try {
            // System.exit(0); // 不会执行 finally
            System.out.println(sc.nextInt() / sc.nextInt());
        } catch (InputMismatchException e) {
            System.out.println("不能输入非int: " + e.getClass().getName());
            e.printStackTrace(); // 打印异常轨迹
            System.out.println("异常信息："+ e.getMessage());
        } catch (ArithmeticException e) {
            System.out.println("不能除以ZeRo: " + e.getClass().getName());
            e.printStackTrace(); // 打印异常轨迹
            System.out.println("异常信息："+ e.getMessage());
        } finally {
            System.out.println("========");
        }
    }
}
```

- 面试题

    - 分析如下代码的执行结果:


```java
public class Text3 {
    public static Scanner sc = new Scanner(System.in);

    public static final int getNumbr() {
        int n = 100;
        try {
            return n;
        } finally {
            ++n;
            System.out.println(n);
        }
    }

    public static void main(String[] args) {
        System.out.println(Text3.getNumbr());
    }
}
```

显然`System.out.println(Text3.getNumbr());`会输出 100, 但`finally`却仍会执行!!!

此时`return n`的`n`算是一个临时值!
```java
// 尝试执行
// 返回值 => 尝试返回一个结果，但发现后面还有finally模块，而finally模块一定会得到执行。
// 于是在这里只能将返回值使用一个临时变量(例如变量a)存储起来。
// 然后再执行finally模块，finally模块执行完之后，再将这个临时变量(a)存储的值返回

// return 不是 系统退出
```

## 自定义异常
### 为什么要使用自定义异常
在Java中，异常的类型非常的多，要想使用这些异常，首先必须要熟悉它们。这无疑是一个巨大的工作量，很耗费时间。如果我们可以自定异常，则只需要熟悉`RuntimeException`、`Exception`和`Throwable`即可。这大大缩小了熟悉范围。自定义异常还可以帮助我们快速的定位问题。

自定义运行时异常语法
```java
class 类名 extends RuntimeException {

}
```

自定义检查异常语法
```java
class 类名 extends Exception {
    
}
```

示例:
```java
package demo2;

/**
 * 用户名不存在异常
 *
 * 异常命名规范：场景描述 + Exception
 */
class UsernameNotFoundException extends Exception {
    public UsernameNotFoundException() {
        super();
    }
    public UsernameNotFoundException(String msg){
        super(msg);
    }
}

/**
 * 账号或密码错误异常
 */
class BadCredentialsException extends Exception {
    BadCredentialsException () {
        super();
    }

    BadCredentialsException (String msg) {
        super(msg);
    }
}

class Login {
    public static final void login(String username, String password) throws BadCredentialsException, UsernameNotFoundException {
        if ("Heng_Xin".equals(username)) {
            if ("114514".equals(password)) {
                System.out.println("登录成功!");
            } else {
                throw new BadCredentialsException("账号或密码错误!");
            }
        } else {
            throw new UsernameNotFoundException("账号不存在!");
        }
    }
}

public class Text4 {
    public static void login() {
        try {
            Login.login("张三", "123");
        } catch (UsernameNotFoundException e) {
            System.out.println(e.getMessage());
        } catch (BadCredentialsException e) {
            System.out.println(e.getMessage());
        }
    }

    public static void main(String[] args) {
        Text4.login();
    }
}
```

对于 检查异常 我们必需要进行以下处理(选一):
- 使用 try-catch 块：调用一个声明了 throws 子句的方法时，可以使用 try-catch 块来捕获这些异常，进行相应的处理。
- 使用 throws 子句传递异常：如果调用一个声明了 throws 子句的方法时，当前方法也可以选择将这些异常继续向上抛出，直到最终有一个方法捕获这些异常或者继续往上层传递。

对于 运行时 异常, 编译器不强制我们进行处理 (但是检查异常不处理, 编译都不通过!)

### 异常使用注意事项
a. 运行时异常可以不处理。

b. 如果父类抛出了多个异常,子类覆盖父类方法时,只能抛出相同的异常或者是该异常的子集。(与协变返回类型原理一致)

c. 父类方法没有抛出异常，子类覆盖父类该方法时也不可抛出`检查异常`。此时子类产生该异常，只能捕获处理，不能声明抛出`检查异常`, 但子类中方法可以声明抛出`运行时异常`

## try-with-resources
在Java中，try-with-resources 是一种在处理资源（例如文件流、数据库连接等）时非常方便的语法结构。try-with-resources 语句包含**一个或多个资源**的声明，这些资源在括号中被初始化，并在 try 块结束后自动关闭。这样可以确保资源在使用完毕后被正确关闭，无需手动编写关闭资源的代码，提高了代码的可读性和可维护性。

语法:

```java
try (ResourceType resource = ResourceInitialization;
     ResourceType2 resource2 = ResourceInitialization2;
     /* ... */) {
    // 对资源进行操作的代码
} catch (ExceptionType e) {
    // 异常处理代码
} finally {
    // 可选的 finally 代码块，用于在 try 块执行后执行清理代码（无论是否抛出异常）
}
```
其中`ResourceType`是需要关闭的资源类型，比如`InputStream`或`Connection`等。在`try`后的括号中，可以声明并初始化需要使用的资源。在`try`块执行结束之后，会自动调用资源的`close()`方法进行关闭。

如果`try`块中抛出异常，会先执行`catch`块的异常处理代码，然后再执行`finally`块的清理代码。如果没有发生异常，也会执行`finally`块的清理代码。

- 作用域: 在`try-with-resources`结构中，括号中声明的资源的作用域仅限于`try`块内部。也就是说，只有在`try`块内部才可以直接访问这些资源。

### 自定义资源类型

如果你想使用自定义资源类型，可以编写实现`AutoCloseable`或`Closeable`接口的类，并在`try-with-resources`中使用。

```java
class MyResource implements AutoCloseable {
    public void close() throws Exception {
        // 在这里进行资源的清理操作
    }
}

// 在 try-with-resources 中使用 MyResource
try (MyResource resource = new MyResource()) {
    // 对资源进行操作的代码
} catch (Exception e) {
    // 异常处理代码
}
```
