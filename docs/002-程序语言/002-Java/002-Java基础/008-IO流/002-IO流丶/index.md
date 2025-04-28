# I/O 流
<style>
h2 {
    color: rgb(110,236,255);
    margin-top: 80px;
}
</style>
## 什么是I/O流
在使用计算机时，你可能遇到过如下场景：
> 当你在编写一个文档时，突然断电了或者计算机蓝屏了，而文档又没有保存。当你重启计算机后，发现文档中修改的那部分内容丢失了，但保存过的内容依然还在。

这是为什么呢？因为我们编写文档的时候，编写的内容是存储在计算机的内存中，这些内容属于临时数据，当我们保存文档后，这些临时数据就写进了磁盘，得以保存。

**过程分析**

编写文档内容存储在内存，换言之，就是向内存写数据

保存文档内容至磁盘，换言之，就是将内存中的数据取出来存储到磁盘

**I / O的来源**

I / O是Input和Ouput两个单词的首字母，表示输入输出。其参照物就是内存，写入内存，就是输入，从内存读取数据出来，就是输出。

**Java 中的 I / O流**

磁盘和内存是两个不同的设备，它们之间要实现数据的交互，就必须要建立一条通道，在Java中实现建立这样的通道的是I / O流。Java 中的 I / O 流是按照数据类型来划分的。分别是字节流（缓冲流、二进制数据流和对象流）、字符流

## 使用流程

1. 创建文件对象, 以告知文件的具体路径, 注意要判断文件父目录是否存在(写入的时候)

```java
File file = new File("./path/to/file");

// 如果目录不存在，则创建目录
File parentDir = file.getParentFile();
if (!parentDir.exists()) {
    parentDir.mkdirs();
}
```

2. 打开输入流或输出流

```java
// 使用 字节流 (二进制/图片)
InputStream inputStream = new FileInputStream(file);
OutputStream outputStream = new FileOutputStream(file);

// 如果要使用其他流而非普通字节流, 则需要使用上面作为构造的参数(当然你可以一行写)
BufferedInputStream bis = new BufferedInputStream(inputStream);
DataOutputStream dos = new DataOutputStream(outputStream);

// 使用字符流 (文档/日志)
Writer fw = new FileWriter(file);
BufferedWriter bw = new BufferedWriter(fw); // 同理, 如果要使用其他流而非普通字符流, 则需要使用上面作为构造的参数(当然你可以一行写)
```

3. 读/写
4. 释放文件指针

## 字节流
> 程序使用字节流执行8位字节的输入和输出。
>
> 所有字节流类均来自`InputStream`和`OutputStream`

### OutputStream 常用方法

```java
public abstract void write(int b) throws IOException;; // 写一个字节
public void write(byte b[]) throws IOException;        // 将给定的字节数组内容全部写入文件中

// 将给定的字节数组中指定的偏移量和长度之间的内容写入文件中
public void write(byte b[], int off, int len) throws IOException;
public void flush() throws IOException; // 强制将通道中数据全部写出
public void close() throws IOException; // 关闭通道
```

### 文件输出流 FileOutputStream 构造方法

```java
public FileOutputStream(String name) throws FileNotFoundException; // 根据提供的文件路径构建一条文件输出通道

// 根据提供的文件路径构建一条文件输出通道，并根据append的值决定是将内容追加到末尾还是直接覆盖
public FileOutputStream(String name, boolean append) throws FileNotFoundException;
public FileOutputStream(File file) throws FileNotFoundException; // 根据提供的文件信息构建一条文件输出通道

// 根据提供的文件信息构建一条文件输出通道，并根据append的值决定是将内容追加到末尾还是直接覆盖
public FileOutputStream(File file, boolean append) throws FileNotFoundException;
```

### InputStream 常用方法

```java
public abstract int read() throws IOException; // 读取一个字节
public int read(byte b[]) throws IOException;  // 读取多个字节存储至给定的字节数组中

// 读取多个字节按照给定的偏移量及长度存储在给定的字节数组中
public int read(byte b[], int off, int len) throws IOException;
public void close() throws IOException;    // 关闭流，也就是关闭磁盘和内存之间的通道
public int available() throws IOException; // 获取通道中数据的长度
```

### 文件输入流 FileInputStream 构造方法

```java
public FileInputStream(String name) throws FileNotFoundException; // 根据提供的文件路径构建一条文件输入通道
public FileInputStream(File file) throws FileNotFoundException;   // 根据提供的文件信息构建一条文件输入通道
```

如果通道中数据长度过长，那么根据通道中数据的长度来构建字节数组，则可能导致内存不够，比如使用流读取一个大小为10G的文件，那么通道中就应该存在10G长的数据，此时应该怎么办？

- 答案是分段读取 (实际上这个意义不大, 建议数据库吧)

### 字节流应用场景
> 字节流仅仅适用于读取原始数据（基本数据类型）

## 字符流
> Java平台使用Unicode约定存储字符值。 字符流I / O会自动将此内部格式与本地字符集转换。 在西方语言环境中，本地字符集通常是ASCII的8位超集。
>
> 所有字符流类均来自`Reader`和`Writer`。 与字节流一样，也有专门用于文件I / O的字符流类: `FileReader`和`FileWriter`。

### Writer 常用方法

```java
public void write(int c) throws IOException;       // 写一个字符
public void write(char cbuf[]) throws IOException; // 将给定的字符数组内容写入到文件中

// 将给定的字符数组中给定偏移量和长度的内容写入到文件中
abstract public void write(char cbuf[], int off, int len) throws IOException;
public void write(String str) throws IOException; // 将字符串写入到文件中
abstract public void flush() throws IOException;  // 强制将通道中的数据全部写出
abstract public void close() throws IOException;  // 关闭通道
```

### FileWriter 构造方法

```java
// 根据提供的文件路径构建一条文件输出通道
public FileWriter(String fileName) throws IOException;
public FileWriter(File file) throws IOException;

// 根据提供的文件信息构建一条文件输出通道，并根据append的值决定是将内容追加到末尾还是直接覆盖
public FileWriter(String fileName, boolean append) throws IOException;
public FileWriter(File file, boolean append) throws IOException;
```

### Reader 常用方法

```java
public int read() throws IOException;            // 读取一个字符
public int read(char cbuf[]) throws IOException; // 读取字符到给定的字符数组中

// 将读取的字符按照给定的偏移量和长度存储在字符数组中
abstract public int read(char cbuf[], int off, int len) throws IOException;
abstract public void close() throws IOException; // 关闭通道
```

### FileReader 构造方法

```java
// 根据提供的文件路径构建一条文件输入通道
public FileReader(String fileName) throws FileNotFoundException;
public FileReader(File file) throws FileNotFoundException;
```

OutputStream 和 Writer 都是 Java 中用于处理输出流的抽象类，它们在功能上有一些相似，但也存在一些重要的区别：

- 处理的数据类型不同：
    - OutputStream 用于处理字节流，主要用于处理二进制数据。
    - Writer 用于处理字符流，主要用于处理文本数据。
- 字符编码支持：
    - OutputStream 处理的是字节流，不涉及字符编码的问题。
    - Writer 包含了对字符编码的支持，可以指定字符编码，方便进行字符输出的操作。
- 使用场景不同：
    - OutputStream 主要适用于处理二进制数据，如图片、音频等。
    - Writer 主要适用于处理字符数据，如文本文件、日志文件等。

同理: `Reader`和`InputStream`也一样
- InputStream 可以通过 read() 方法来从输入流中读取数据，返回一个字节或字节数组。
- Reader 可以通过 read() 方法来从输入流中读取数据，返回一个字符或字符数组。

## 缓冲流
> 到目前为止，我们看到的大多数示例都使用无缓冲的I / O。 这意味着每个读取或写入请求均由基础操作系统直接处理。 由于每个这样的请求通常会触发磁盘访问，网络活动或某些其他相对昂贵的操作，因此这可能会使程序的效率大大降低。
>
> 为了减少这种开销，Java平台实现了缓冲的I / O流。 缓冲的输入流从称为缓冲区的存储区中读取数据； 仅当缓冲区为空时才调用本机输入API。 同样，缓冲的输出流将数据写入缓冲区，并且仅在缓冲区已满时才调用本机输出API。
>
> 有四种用于包装非缓冲流的缓冲流类：BufferedInputStream和BufferedOutputStream创建缓冲的字节流，而BufferedReader和BufferedWriter创建缓冲的字符流。

### BufferedOutputStream 构造方法

```java
public BufferedOutputStream(OutputStream out);           // 根据给定的字节输出流创建一个缓冲输出流，缓冲区大小使用默认大小
public BufferedOutputStream(OutputStream out, int size); // 根据给定的字节输出流创建一个缓冲输出流，并指定缓冲区大小
```

### BufferedInputStream 构造方法

```java
public BufferedInputStream(InputStream in);           // 根据给定的字节输入流创建一个缓冲输入流，缓冲区大小使用默认大小
public BufferedInputStream(InputStream in, int size); // 根据给定的字节输入流创建一个缓冲输入流，并指定缓冲区大小
```

### BufferedWriter 构造方法
> 其常用方法同上`Writer`
```java
public BufferedWriter(Writer out);         // 根据给定的字符输出流创建一个缓冲字符输出流，缓冲区大小使用默认大小
public BufferedWriter(Writer out, int sz); // 根据给定的字符输出流创建一个缓冲字符输出流，并指定缓冲区大小
```

### BufferedReader 构造方法
> 其常用方法同上`Reader`
```java
public BufferedReader(Reader in);         // 根据给定的字符输入流创建一个缓冲字符输入流，缓冲区大小使用默认大小
public BufferedReader(Reader in, int sz); // 根据给定的字符输入流创建一个缓冲字符输入流，并指定缓冲区大小
```

## 数据流
> 数据流支持原始数据类型值（布尔值，char，字节，short，int，long，float和double）以及String值的二进制I / O。 所有数据流都实现DataInput接口或DataOutput接口。 本节重点介绍这些接口的最广泛使用的实现，即DataInputStream和DataOutputStream。

### DataOutput 接口常用方法

```java
void writeBoolean(boolean v) throws IOException; // 将布尔值作为1个字节写入底层输出通道
void writeByte(int v) throws IOException;        // 将字节写入底层输出通道
void writeShort(int v) throws IOException;       // 将短整数作为2个字节(高位在前)写入底层输出通道
void writeChar(int v) throws IOException;        // 将字符作为2个字节写(高位在前)入底层输出通道
void writeInt(int v) throws IOException;         // 将整数作为4个字节写(高位在前)入底层输出通道
void writeLong(long v) throws IOException;       // 将长整数作为8个字节写(高位在前)入底层输出通道
void writeFloat(float v) throws IOException;     // 将单精度浮点数作为4个字节写(高位在前)入底层输出通道
void writeDouble(double v) throws IOException;   // 将双精度浮点数作为8个字节写(高位在前)入底层输出通道
void writeUTF(String s) throws IOException;      // 将UTF-8编码格式的字符串以与机器无关的方式写入底层输出通道。
```

### DataOutputStream 构造方法

```java
public DataOutputStream(OutputStream out); // 根据给定的字节输出流创建一个二进制输出流
```

### DataInput 接口常用方法

```java
boolean readBoolean() throws IOException;   // 读取一个字节，如果为0，则返回false；否则返回true。
byte readByte() throws IOException;         // 读取一个字节
int readUnsignedByte() throws IOException;  // 读取一个字节，返回0~255之间的整数
short readShort() throws IOException;       // 读取2个字节，然后返一个短整数
int readUnsignedShort() throws IOException; // 读取2个字节，返回一个0~65535之间的整数
char readChar() throws IOException;         // 读取2个字节，然后返回一个字符
int readInt() throws IOException;           // 读取4个字节，然后返一个整数
long readLong() throws IOException;         // 读取8个字节，然后返一个长整数
float readFloat() throws IOException;       // 读取4个字节，然后返一个单精度浮点数
double readDouble() throws IOException;     // 读取8个字节，然后返一个双精度浮点数
String readUTF() throws IOException;        // 读取一个使用UTF-8编码格式的字符串
```

### DataInputStream 构造方法

```java
public DataInputStream(InputStream in); // 根据给定的字节输入流创建一个二进制输入流
```

### 注意
> [!TIP]
> 注意: `DataStreams`通过捕获`EOFException`来检测文件结束条件,而不是测试无效的返回值。`DataInput`方法的所有实现都使用`EOFException`而不是返回值。

## 对象流
> 正如二进制数据流支持原始数据类型的I / O一样，对象流也支持对象的I / O。大多数（但不是全部）标准类支持其对象的序列化。那些类实现了序列化标记接口Serializable才能够序列化。

### ObjectOutput 接口常用方法

```java
public void writeObject(Object obj) throws IOException; // 将对象写入底层输出通道
```

### ObjectOutputSteam 构造方法

```java
public ObjectOutputStream(OutputStream out) throws IOException; // 根据给定的字节输出流创建一个对象输出流
```

#### 需要实现 Serializable 接口

<span style="color:red">将一个对象从内存中写入磁盘文件中的过程称之为序列化。序列化必须要求该对象所有类型实现序列化的接口`Serializable`</span>

- 它的成员类变量也需要实现 `Serializable` 接口,

- 不希望被序列化的则需要使用 `transient` 关键字标记!

---

一般来说，以下类型的成员变量是不可序列化的：

1. **非 `Serializable` 类型的对象：** 如果一个类没有实现 `Serializable` 接口，它的对象就不能被序列化。这通常包括自定义的类，除非明确地实现了 `Serializable` 接口。
2. **静态变量：** 静态变量属于类的状态，而不是对象的状态，因此它们不会被序列化。
3. **Transient 关键字标记的变量：** 使用 `transient` 关键字标记的成员变量不会被序列化。这在需要排除某些不需要序列化的对象或敏感信息时很有用。
4. **匿名内部类和局部内部类：** 匿名内部类和局部内部类无法被序列化，因为它们没有名称，无法被正确地重建。

要确保对象可以被序列化，需要仔细检查其成员变量的类型，并确保它们是可序列化的。如果有不可序列化的成员变量，可以使用 `transient` 关键字标记它们，或者在需要时实现 `Serializable` 接口。

---

<span style="color:yellow">如果已经写入文件但是报错了/什么的, 记得删除它, 不然读取的时候也会报错! (没有实现`Serializable`接口就写入文件的情况下)</span>

### ObjectInput 接口常用方法

```java
public Object readObject() throws ClassNotFoundException, IOException; // 读取一个对象
```

### ObjectInputSteam 构造方法

```java
public ObjectInputStream(InputStream in) throws IOException; // 根据给定的字节输入流创建一个对象输入流
```

<span style="color:red">将磁盘中存储的对象信息读入内存中的过程称之为反序列化，需要注意的是，反序列化必须保证与序列化时使用的**JDK版本一致**</span>

## 思考: 如何将一个字节流转成字符流?

```java
import java.io.*;

public class IOConverter {
    public static void main(String[] args) {
        write();
        read();
    }

    private static void write() {
        try (OutputStream os = new FileOutputStream("F:\\lines.txt");
             OutputStreamWriter osw = new OutputStreamWriter(os); // 字节流转为字符流
             BufferedWriter bw = new BufferedWriter(osw)) {
            String[] lines = {
                    "这是写入的第1行",
                    "这是写入的第2行",
                    "这是写入的第3行",
                    "这是写入的第4行"
            };
            for (String line : lines) {
                bw.write(line);
                bw.newLine();
            }
            bw.flush();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void read() {
        try (InputStream is = new FileInputStream("F:\\lines.txt");
             InputStreamReader isr = new InputStreamReader(is);
             BufferedReader reader = new BufferedReader(isr);) {
            while (true) {
                String line = reader.readLine();
                if (line == null) break;
                System.out.println(line);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
```

## 流的继承关系图

```C++
- Stream // 抽象基类或接口，表示流的概念  
   |  
   -+ ByteStream // 字节流基类或接口  
   |  |  
   |  -+ InputStream // 字节输入流基类或接口  
   |  |  |  
   |  |  -+ FileInputStream      // 文件字节输入流  
   |  |  -+ BufferedInputStream  // 缓冲字节输入流  
   |  |  -+ DataInputStream      // 数据字节输入流  
   |  |  -+ ObjectInputStream    // 对象输入流，继承自InputStream
   |  |  ...  
   |  |  
   |  -+ OutputStream // 字节输出流基类或接口  
   |     |  
   |     -+ FileOutputStream      // 文件字节输出流  
   |     -+ BufferedOutputStream  // 缓冲字节输出流  
   |     -+ DataOutputStream      // 数据字节输出流  
   |     -+ ObjectOutputStream    // 对象输出流，继承自OutputStream
   |     ...  
   |  
   -+ CharStream // 字符流基类或接口  
   |  |  
   |  -+ Reader // 字符输入流基类或接口  
   |  |  |  
   |  |  -+ FileReader        // 文件字符输入流  
   |  |  -+ BufferedReader    // 缓冲字符输入流  
   |  |  -+ CharArrayReader   // 字符数组输入流  
   |  |  ...  
   |  |  
   |  -+ Writer // 字符输出流基类或接口  
   |     |  
   |     -+ FileWriter        // 文件字符输出流  
   |     -+ BufferedWriter    // 缓冲字符输出流  
   |     -+ CharArrayWriter   // 字符数组输出流  
   |     ...  
   |  
   ...
```
