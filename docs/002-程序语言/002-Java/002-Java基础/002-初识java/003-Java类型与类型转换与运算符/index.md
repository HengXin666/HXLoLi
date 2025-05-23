# java类型
同 C++

```java
// 基本数据类型
byte    1B,   表示范围为 -128 到 127 的整数。
char    2B    字符 表示 Unicode 字符。不是整数。因此，char 类型的取值范围是 0 到 65535，而不是 -32768 到 32767。
short    2字节  表示范围为 -32768 到 32767 的整数。
int     4B    表示范围为 -2147483648 到 2147483647 的整数。(2 * 10^9)
long    8B    -9223372036854775808 到 9223372036854775807 的整数。(9 * 10^18)
float   4B
double  8B
boolean 1B 布尔

// 引用类型
string  字符串
```

## 类型转化

`+=` 等有其他功能, 强转语法同`C语言`: `(类型)变量`

```java
int a = 1;
long b = 2 + a; // 自动类型转换
// int c = b;   // 报错: b 是 long, 注意C/C++只是警告
int c = (int)b; // 合法的
c += b;         // 合法的, b被强制转换了

// 同理, 要求需要强转
long g = 1;
float gg = 2.0f;

// g = gg;      // 报错!
g = (long)gg;   // 合法
```

## 运算符
完全同C++

int / int == int