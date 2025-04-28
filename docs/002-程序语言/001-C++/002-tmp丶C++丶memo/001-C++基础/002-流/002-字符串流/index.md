# 字符串流
## 介绍
`<sstream>` 定义了三个类：istringstream、ostringstream 和 stringstream，分别用来进行流的输入、输出和输入输出操作。

和 cin / cout / 文件流 一样

## 不同类的重要区别
- `istringstream` 主要用于从字符串中读取数据，因此只支持输入操作，例如使用 `>>` 运算符来从流中提取数据。(不能使用 `<<` 运算符)

- `ostringstream` 主要用于向字符串中写入数据，因此只支持输出操作，例如使用 `<<` 运算符向流中写入数据。(不能使用 `>>` 运算符)

- `stringstream` 则同时支持输入和输出操作，可以用来既从字符串中读取数据，又向字符串中写入数据。

虽然这三种类型的流对象**都**可以使用 `str()` 方法来获取其中的字符串，但是它们的主要用途和支持的操作是有所不同的。

## 应用场景

`<sstream>` 主要用来进行**数据类型转换**，由于 `<sstream>` 使用 string 对象来代替字符数组（snprintf 方式）, $ 避免了缓冲区溢出的危险 $ ；而且，因为传入参数和目标对象的类型会被`自动推导`出来，所以不存在错误的格式化符号的问题。简单说，相比 C 编程语言库的数据类型转换，`<sstream>` 更加安全、自动和直接。

例如: (如果不知道下面代码的具体语法, 请进行往下看)

```C++
#include <iostream>
#include <sstram>

using namespace std;

int main(void)
{
    stringstream str;
    str << "123 0.721 qwq";

    int num;
    double f;
    string s;
    // 类型转换
    str >> num >> f >> s;
  
    cout << num << " " << f << " " << s << endl;
    return 0;
}
```

字符串拼接也可以(string容器也行)

## 使用方法


### 构造函数

```C++
stringstream s1("字符串"); // 直接传入字符串或者string对象

istringstream s2("字符串");

ostringstream s3("字符串");

s1.str("新的字符串"); // 直接覆写字符串, 不论原本是什么, 直接变成 str(...) 的内容, 
                    // 如果为空是另一个重载函数--返回字符串的拷贝
```

### 输出
> 输入输出是以 该对象本身 为参考系的.
```C++
// ======= 打印输出 =======
// .str() 方法 得到 其string对象的一份拷贝
cout << s4.str() << endl;

//string 的 c_str() 方法, c风格的字符串
printf("%s\n", s4.str().c_str());

// ======= 赋值输出 =======
int a;
s4 >> a; // 需要保证其内容在字符串前面 (类似于 cin >> a 时候接收输入)
```

### 输入
> 输入输出是以 该对象本身 为参考系的.

使用构造函数, 其输入的指针还是在0的位置, 所以不会因为 `<<`运算符, `put()`方法 而在字符串后边补充, 而是需要从 输入指针 位置自增到目标处

当然, 所以 `str(内容)` 方法, 也会清空输入指针的位置
```C++
stringstream s_str_obj("hello");
cout << "0: " << s_str_obj.str() <<endl;    // hello

s_str_obj << 'a';
cout << "1: " << s_str_obj.str() << endl;    // aello

s_str_obj.put('b');
cout << "2: " << s_str_obj.str() << endl;    // abllo

s_str_obj << "str";
cout << "3: " << s_str_obj.str() << endl;    // abstr

s_str_obj.str("c");
cout << "4: " << s_str_obj.str() << endl;    // c

s_str_obj << "gg";
cout << "5: " << s_str_obj.str() << endl;    // gg
```

### 清空

1. 使用 `str()` 方法将对象重新设置为空字符串, 并且重置输入指针的位置, 但是会保留内存.
```C++
s_str_obj.str("");
```
2. 使用 `clear()` 方法清空状态标志，并将读写指针都重置为初始位置。需要注意的是，这个方法并不会清空流中的内容，只是将对象恢复到初始状态


```C++
// 清空状态标志，并将读写指针都重置为初始位置
s_str_obj.clear();

```

3. 如果你想要确保释放 stringstream 对象占用的内存空间，可以使用 swap() 方法将其与一个空的 stringstream 对象交换。这样做会导致原来的对象释放其内存，并将其重置为空对象。

```C++
stringstream ss;
// 使用 swap() 方法将对象与一个空的 stringstream 对象交换
ss.swap(stringstream());

// 或者
stringstream().swap(ss);
```

### 拓展

其他可能常用的方法:

- tellp()：返回当前写入位置在流中的位置（以字节为单位）。
- tellg()：返回当前读取位置在流中的位置（以字节为单位）。
- seekp(pos) 和 seekp(pos, mode)：设置写入位置到指定的位置。pos 表示要设置的位置，mode 控制偏移方式，默认值为 ios::beg，表示相对于流的开头；还可以设置为 ios::cur，表示相对于当前位置；或者 ios::end，表示相对于流的结尾。
- seekg(pos) 和 seekg(pos, mode)：设置读取位置到指定的位置。pos 表示要设置的位置，mode 控制偏移方式，默认值为 ios::beg，表示相对于流的开头；还可以设置为 ios::cur，表示相对于当前位置；或者 ios::end，表示相对于流的结尾。

除此之外，还有一些其他的方法，例如 ignore()、get()、putback() 等等，它们也经常被使用。具体使用哪些方法，需要根据具体的需求来决定。By GPT-3.5