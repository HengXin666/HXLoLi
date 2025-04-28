# 四、QVariant
QVariant这个类很神奇，或者说方便。很多时候，需要几种不同的数据类型需要传递，如果用结构体，又不大方便，容器保存的也只是一种数据类型，而QVariant则可以统统搞定。

QVariant 这个类型充当着最常见的数据类型的联合。QVariant 可以保存很多Qt的数据类型，包括QBrush、QColor、QCursor、QDateTime、QFont、QKeySequence、 QPalette、QPen、QPixmap、QPoint、QRect、QRegion、QSize和QString，并且还有C++基本类型，如 int、float等。

## 4.1 标准类型
- 将标准类型转换为QVariant类型

```C++
// 这类转换需要使用QVariant类的构造函数, 由于比较多, 大家可自行查阅Qt帮助文档, 在这里简单写几个
QVariant::QVariant(int val);
QVariant::QVariant(bool val);
QVariant::QVariant(double val);
QVariant::QVariant(const char *val);
QVariant::QVariant(const QByteArray &val);
QVariant::QVariant(const QString &val);
// ......
    
// 使用设置函数也可以将支持的类型的数据设置到QVariant对象中
// 这里的 T 类型, 就是QVariant支持的类型
void QVariant::setValue(const T &value);
// 该函数行为和 setValue() 函数完全相同
[static] QVariant QVariant::fromValue(const T &value);
// 例子:
#if 1 // 方式1
QVariant v;
v.setValue(5);
#else // 方式2
QVariant v = QVariant::fromValue(5);
#endif

int i = v.toInt();          // i is now 5
QString s = v.toString();   // s is now "5"
```

- 判断 QVariant中封装的实际数据类型

```C++
// 该函数的返回值是一个枚举类型, 可通过这个枚举判断出实际是什么类型的数据
Type QVariant::type() const;
```
返回值`Type`的部分枚举定义, 全部信息可以自行查阅Qt帮助文档


- 将QVariant对象转换为实际的数据类型

```C++
// 如果要实现该操作, 可以使用QVariant类提供的 toXxx() 方法, 全部转换可以参考Qt帮助文档
// 在此举列举几个常用函数:
bool QVariant::toBool() const;
QByteArray QVariant::toByteArray() const;
double QVariant::toDouble(bool *ok = Q_NULLPTR) const;
float QVariant::toFloat(bool *ok = Q_NULLPTR) const;
int QVariant::toInt(bool *ok = Q_NULLPTR) const;
QString QVariant::toString() const;
// ......
```

## 4.2 自定义类型
除了标准类型, 我们自定义的类型也可以使用QVariant类进行封装, 被QVariant存储的数据类型需要有一个默认的构造函数和一个拷贝构造函数。为了实现这个功能，首先必须使用`Q_DECLARE_METATYPE()`宏。通常会将这个宏放在类的声明所在头文件的下面， 原型为:

```C++
Q_DECLARE_METATYPE(Type)
```
使用的具体步骤如下

1. 第一步: 在头文件中声明

```C++
// *.h
struct MyTest {
    int id;
    QString name;
};
// 自定义类型注册
Q_DECLARE_METATYPE(MyTest)
```

2. 第二步: 在源文件中定义(使用)

```C++
MyTest t;
t.name = "张三丰";
t.num = 666;
// 值的封装
QVariant vt = QVariant::fromValue(t);

// 值的读取
if(vt.canConvert<MyTest>()) {
    MyTest t = vt.value<MyTest>();
    qDebug() << "name: " << t.name << ", num: " << t.num;
}
```

以上操作用到的QVariant类的API如下:

```C++
// 如果当前QVariant对象可用转换为对应的模板类型 T, 返回true, 否则返回false
bool QVariant::canConvert() const;
// 将当前QVariant对象转换为实际的 T 类型
T QVariant::value() const;
```
