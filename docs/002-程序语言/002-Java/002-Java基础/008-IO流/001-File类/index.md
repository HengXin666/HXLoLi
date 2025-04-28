# File类
## File类的作用
`java.io.File`类是对存储在磁盘上的文件信息的一个抽象表示。主要用于文件的创建、查找和删除。

## 常用构造函数

```java
public File(String pathname);             // 通过将给定的字符串路径名转换为抽象路径名来创建File实例
public File(String parent, String child); // 通过给定的字符父级串路径和字符串子级路径来创建File实例
public File(File parent, String child);   // 通过父级抽象路径名和字符串子路径创建File实例。
```

示例:
```java
import java.io.File;

public class Main {
    public static final void demo_1() {
        File file1 = new File("F:\\text\\awa\\qwq.txt");
        File file2 = new File("F:\\text\\awa", "qwq.txt");

        File path = new File("F:\\text\\awa");
        File file3 = new File(path, "qwq.txt");
    }

    public static void main(String[] args) {
        demo_1();
    }
}
```

## 常用方法
### 获取文件相关信息

```java
// 绝对路径: 带有盘符的路径称之为绝对路径
// 相对路径: 不带盘符的路径称之为相对路径, 相对路径相对于当前工程来定位的
public String getAbsolutePath(); // 获取文件的绝对路径
public String getName();         // 获取文件的名字
public String getPath();         // 获取文件的路径
public File getParentFile();     // 获取文件的父目录
public String getParent();       // 获取文件的父目录路径
public long length();            // 获取文件的大小
public long lastModified();      // 获取文件最后修改时间
```

示例:
```java
import java.io.File;

public class Main {
    public static final void demo_2() {
        File file = new File("F:\\text\\awa\\qwq.txt");
        System.out.println(file.getAbsoluteFile()); // 获取绝对路径
        System.out.println(file.getName());         // 获取文件名字
        System.out.println(file.getPath());         // 获取文件路径
        System.out.println(file.getParentFile());   // 获取文件父目录
        System.out.println(file.getParent());       // 获取父目录路径
        System.out.println(file.length());          // 获取文件大小: 单位字节
        System.out.println(file.lastModified());    // 获取文件最后修改时间
        System.out.println(System.currentTimeMillis()); // 获取当前系统时间 (毫秒)
    }

    public static void main(String[] args) {
        demo_2();
    }
}
```

### 文件相关的判断
```java
public boolean canRead();                          // 是否可读
public boolean canWrite();                         // 是否可写
public boolean exists();                           // 是否存在
public boolean isDirectory();                      // 是否是目录
public boolean isFile();                           // 是否是一个正常的文件(可能网络传输没有传完什么的)
public boolean isHidden();                         // 是否隐藏
public boolean canExecute();                       // 是否可执行
public boolean createNewFile() throws IOException; // 创建新的文件
public boolean delete();                           // 删除文件
public boolean mkdir();                            // 创建目录，一级
public boolean mkdirs();                           // 创建目录，多级
public boolean renameTo(File dest);                // 文件重命名
```

示例:
```java
import java.io.File;

public class Main {
    public static final void demo_3() {
        File file = new File("F:\\text\\awa\\stu.txt");
        // 判断文件是否可读
        boolean readable = file.canRead();
        System.out.println("文件是否可读：" + readable);
        // 判断文件是否可写
        boolean writable = file.canWrite();
        System.out.println("文件是否可写：" + writable);
        // 判断文件是否存在
        boolean exists = file.exists();
        System.out.println("文件是否存在：" + exists);
        // 判断文件是否是目录
        boolean isDirectory = file.isDirectory();
        System.out.println("文件是否是目录：" + isDirectory);
        File parent = file.getParentFile();
        System.out.println("父级文件是否是目录：" + parent.isDirectory());
        // 判断文件是否是隐藏文件
        boolean hidden = file.isHidden();
        System.out.println("文件是否是隐藏文件：" + hidden);
        // 判断文件是否可执行
        boolean executable = file.canExecute();
        // 所谓的可执行文件，是指双击之后有反应的文件都称之为可执行文件
        System.out.println("文件是否可执行：" + executable);
        File newFile = new File("F:\\test\\stu\\new.txt");
        File parentFile = newFile.getParentFile();
        if (!parentFile.exists()) {// 通常会与创建目录的方法配合使用
            // 创建父级目录，但只能创建一级
            // parentFile.mkdir();
            // 创建多级父级目录
            parentFile.mkdirs();
        }
        if (!newFile.exists()) {
            try {
                // 创建文件时，必须保证改文件的父级目录存在，否则，创建将报IO异常
                boolean success = newFile.createNewFile();
                System.out.println("文件创建是否成功：" + success);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        boolean deleteSuccess = file.delete();
        System.out.println("文件删除是否成功：" + deleteSuccess);
        // 删除文件夹时，必须保证文件夹中没有任何文件，也就是保证文件夹是空的
        boolean deleteFolderSuccess = parentFile.delete();
        System.out.println("文件夹删除是否成功：" + deleteFolderSuccess);
        File renameDest = new File("F:\\test\\a.txt");
        // 文件重命名至目标文件夹时，必须保证目标文件夹存在。重命名操作成功后，原来的文件就移动过去了。
        boolean renameSuccess = newFile.renameTo(renameDest);
        System.out.println("文件重命名是否成功：" + renameSuccess);
    }

    public static void main(String[] args) {
        demo_3();
    }
}
```

- <span style="color:red">删除文件夹时必须保证文件夹为空，否则将删除失败</span>

- 重命名文件后, 文件会被移动到目标路径

### 文件列表相关
```java
public File[] listFiles();                  // 列出文件夹下所有文件
public File[] listFiles(FileFilter filter); // 列出文件夹下所有满足条件的文件
```

示例:
```java
import java.io.File;

public class Main {
    public static void main(String[] args) {
        demo_4();
    }

    public static final void demo_4() {
        {
            File file = new File("F:\\HX_html");
            File[] files = file.listFiles();

            if (files != null) {
                for (File it : files) {
                    System.out.println(it.getPath());
                }
            }
        }

        // 指定需要满足条件
        {
            File file = new File("F:\\HX_html");
            
            // 匿名类
            FileFilter re_f = new FileFilter() {
                @Override
                public boolean accept(File pathname) {
                    return pathname.getName().endsWith(".html"); // 判断是否以 ".html" 结尾
                }
            };
            File[] files = file.listFiles(re_f);

            if (files != null) {
                for (File it : files) {
                    System.out.println(it.getPath());
                }
            }
        }
    }
}
```

递归遍历指定文件路径的所有文件夹和文件并输出:

```java
public class Main {
    public static void main(String[] args) {
        demo_5();
    }

    // 递归输出
    public static final void demo_5_r(File file) {
        if (file.isFile()) {
            System.out.println(file.getPath());
            return;
        }
        else
            System.out.println(file.getPath());

        File[] files = file.listFiles();
        if (files == null) // 这里可能是多余的(实际上有用的只有一次, 可以优化)
            return;

        for (File it : files) {
            demo_5_r(it);
        }
    }

    public static final void demo_5() {
        demo_5_r(new File("F:\\HX_html")); // 指定路径
    }
}
```

注: 对于上面的 `if (files != null)` 的解释如下: 因为可能路径是错误的, 导致连文件夹都打不开, 故会返回`null`

```java
public File[] listFiles() {
    String[] ss = list();
    if (ss == null) return null;
    int n = ss.length;
    File[] fs = new File[n];
    for (int i = 0; i < n; i++) {
        fs[i] = new File(ss[i], this);
    }
    return fs;
}
```
