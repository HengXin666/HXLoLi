# 三、字符串类型
## 3.1 QByteArray
在Qt中QByteArray可以看做是c语言中 char*的升级版本。我们在使用这种类型的时候可通过这个类的构造函数申请一块动态内存，用于存储我们需要处理的字符串数据。

- 构造函数
```C++
// 构造空对象, 里边没有数据
QByteArray::QByteArray();
// 将data中的size个字符进行构造, 得到一个字节数组对象
// 如果 size==-1 函数内部自动计算字符串长度, 计算方式为: strlen(data) O(n) 的
QByteArray::QByteArray(const char *data, int size = -1);
// 构造一个长度为size个字节, 并且每个字节值都为ch的字节数组
QByteArray::QByteArray(int size, char ch);
```

- 数据操作

```C++
// 在尾部追加数据
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QByteArray &QByteArray::append(const QByteArray &ba);
void QByteArray::push_back(const QByteArray &other);

// 头部添加数据
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QByteArray &QByteArray::prepend(const QByteArray &ba);
void QByteArray::push_front(const QByteArray &other);

// 插入数据, 将ba插入到数组第 i 个字节的位置(从0开始)
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QByteArray &QByteArray::insert(int i, const QByteArray &ba);

// 删除数据
// 从大字符串中删除len个字符, 从第pos个字符的位置开始删除
QByteArray &QByteArray::remove(int pos, int len);
// 从字符数组的尾部删除 n 个字节
void QByteArray::chop(int n);
// 从字节数组的 pos 位置将数组截断 (前边部分留下, 后边部分被删除)
void QByteArray::truncate(int pos);
// 将对象中的数据清空, 使其为null
void QByteArray::clear();

// 字符串替换
// 将字节数组中的 子字符串 before 替换为 after
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QByteArray &QByteArray::replace(const QByteArray &before, const QByteArray &after);
```

- 子字符串查找和判断

```C++
// 判断字节数组中是否包含子字符串 ba, 包含返回true, 否则返回false
bool QByteArray::contains(const QByteArray &ba) const;
bool QByteArray::contains(const char *ba) const;
// 判断字节数组中是否包含子字符 ch, 包含返回true, 否则返回false
bool QByteArray::contains(char ch) const;

// 判断字节数组是否以字符串 ba 开始, 是返回true, 不是返回false
bool QByteArray::startsWith(const QByteArray &ba) const;
bool QByteArray::startsWith(const char *ba) const;
// 判断字节数组是否以字符 ch 开始, 是返回true, 不是返回false
bool QByteArray::startsWith(char ch) const;

// 判断字节数组是否以字符串 ba 结尾, 是返回true, 不是返回false
bool QByteArray::endsWith(const QByteArray &ba) const;
bool QByteArray::endsWith(const char *ba) const;
// 判断字节数组是否以字符 ch 结尾, 是返回true, 不是返回false
bool QByteArray::endsWith(char ch) const;
```

- 遍历

```C++
// 使用迭代器
iterator QByteArray::begin();
iterator QByteArray::end();

// 使用数组的方式进行遍历
// i的取值范围 0 <= i < size()
char QByteArray::at(int i) const;
char QByteArray::operator[](int i) const;
```

- 查看字节数

```C++
// 返回字节数组对象中字符的个数
int QByteArray::length() const;
int QByteArray::size() const;
int QByteArray::count() const;

// 返回字节数组对象中 子字符串ba 出现的次数
int QByteArray::count(const QByteArray &ba) const;
int QByteArray::count(const char *ba) const;
// 返回字节数组对象中 字符串ch 出现的次数
int QByteArray::count(char ch) const;
```

- 类型转换

```C++
// 将QByteArray类型的字符串 转换为 char* 类型
char *QByteArray::data();
const char *QByteArray::data() const;

// int, short, long, float, double -> QByteArray
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QByteArray &QByteArray::setNum(int n, int base = 10); // base 进制
QByteArray &QByteArray::setNum(short n, int base = 10);
QByteArray &QByteArray::setNum(qlonglong n, int base = 10);
QByteArray &QByteArray::setNum(float n, char f = 'g', int prec = 6); // f = 'g', 'g' 是科学计数法, prec是精度
QByteArray &QByteArray::setNum(double n, char f = 'g', int prec = 6);
[static] QByteArray QByteArray::number(int n, int base = 10);
[static] QByteArray QByteArray::number(qlonglong n, int base = 10);
[static] QByteArray QByteArray::number(double n, char f = 'g', int prec = 6);

// QByteArray -> int, short, long, float, double
int QByteArray::toInt(bool *ok = Q_NULLPTR, int base = 10) const;
short QByteArray::toShort(bool *ok = Q_NULLPTR, int base = 10) const;
long QByteArray::toLong(bool *ok = Q_NULLPTR, int base = 10) const;
float QByteArray::toFloat(bool *ok = Q_NULLPTR) const;
double QByteArray::toDouble(bool *ok = Q_NULLPTR) const;

// std::string -> QByteArray
[static] QByteArray QByteArray::fromStdString(const std::string &str);
// QByteArray -> std::string
std::string QByteArray::toStdString() const;

// 所有字符转换为大写
QByteArray QByteArray::toUpper() const;
// 所有字符转换为小写
QByteArray QByteArray::toLower() const;
```


## 3.2 QString
QString也是封装了字符串, 但是`内部的编码为utf8`, UTF-8属于Unicode字符集, 它固定使用多个字节（window为2字节, linux为3字节）来表示一个字符，这样可以将世界上几乎所有语言的常用字符收录其中。

- 构造函数

```C++
// 构造一个空字符串对象
QString::QString();
// 将 char* 字符串 转换为 QString 类型
QString::QString(const char *str);
// 将 QByteArray 转换为 QString 类型
QString::QString(const QByteArray &ba);
// 其他重载的同名构造函数可参考Qt帮助文档, 此处略
```

- 数据操作

```C++
// 尾部追加数据
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QString &QString::append(const QString &str);
QString &QString::append(const char *str);
QString &QString::append(const QByteArray &ba);
void QString::push_back(const QString &other);

// 头部添加数据
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QString &QString::prepend(const QString &str);
QString &QString::prepend(const char *str);
QString &QString::prepend(const QByteArray &ba);
void QString::push_front(const QString &other);

// 插入数据, 将 str 插入到字符串第 position 个字符的位置(从0开始)
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QString &QString::insert(int position, const QString &str);
QString &QString::insert(int position, const char *str);
QString &QString::insert(int position, const QByteArray &str);

// 删除数据
// 从大字符串中删除len个字符, 从第pos个字符的位置开始删除
QString &QString::remove(int position, int n);

// 从字符串的尾部删除 n 个字符
void QString::chop(int n);
// 从字节串的 position 位置将字符串截断 (前边部分留下, 后边部分被删除)
void QString::truncate(int position);
// 将对象中的数据清空, 使其为null
void QString::clear();

// 字符串替换
// 将字节数组中的 子字符串 before 替换为 after
// 参数 cs 为是否区分大小写, 默认区分大小写
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QString &QString::replace(const QString &before, const QString &after, Qt::CaseSensitivity cs = Qt::CaseSensitive);
```

- 子字符串查找和判断

```C++
// 参数 cs 为是否区分大小写, 默认区分大小写
// 其他重载的同名函数可参考Qt帮助文档, 此处略

// 判断字符串中是否包含子字符串 str, 包含返回true, 否则返回false
bool QString::contains(const QString &str, Qt::CaseSensitivity cs = Qt::CaseSensitive) const;

// 判断字符串是否以字符串 ba 开始, 是返回true, 不是返回false
bool QString::startsWith(const QString &s, Qt::CaseSensitivity cs = Qt::CaseSensitive) const;

// 判断字符串是否以字符串 ba 结尾, 是返回true, 不是返回false
bool QString::endsWith(const QString &s, Qt::CaseSensitivity cs = Qt::CaseSensitive) const;
```

- 遍历

```C++
// 使用迭代器
iterator QString::begin();
iterator QString::end();

// 使用数组的方式进行遍历
// i的取值范围 0 <= position < size()
const QChar QString::at(int position) const
const QChar QString::operator[](int position) const;
```

- 查看字节数

```C++
// 返回字节数组对象中字符的个数 (字符个数和字节个数是不同的概念)
int QString::length() const;
int QString::size() const;
int QString::count() const;

// 返回字节串对象中 子字符串 str 出现的次数
// 参数 cs 为是否区分大小写, 默认区分大小写
int QString::count(const QStringRef &str, Qt::CaseSensitivity cs = Qt::CaseSensitive) const;
```

- 类型转换

```C++
// 将int, short, long, float, double 转换为 QString 类型
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QString &QString::setNum(int n, int base = 10); // base 是进制
QString &QString::setNum(short n, int base = 10);
QString &QString::setNum(long n, int base = 10);
QString &QString::setNum(float n, char format = 'g', int precision = 6); // g 是科学计数法, precision 是精度((科学计数法的?)小数点几位)
QString &QString::setNum(double n, char format = 'g', int precision = 6);
[static] QString QString::number(long n, int base = 10);
[static] QString QString::number(int n, int base = 10);
[static] QString QString::number(double n, char format = 'g', int precision = 6);

// 将 QString 转换为 int, short, long, float, double 类型
int QString::toInt(bool *ok = Q_NULLPTR, int base = 10) const;
short QString::toShort(bool *ok = Q_NULLPTR, int base = 10) const;
long QString::toLong(bool *ok = Q_NULLPTR, int base = 10) const
float QString::toFloat(bool *ok = Q_NULLPTR) const;
double QString::toDouble(bool *ok = Q_NULLPTR) const;

// 将标准C++中的 std::string 类型 转换为 QString 类型
[static] QString QString::fromStdString(const std::string &str);
// 将 QString 转换为 标准C++中的 std::string 类型
std::string QString::toStdString() const;

// QString -> QByteArray
// 转换为本地编码, 跟随操作系统
QByteArray QString::toLocal8Bit() const;
// 转换为 Latin-1 编码的字符串 不支持中文
QByteArray QString::toLatin1() const;
// 转换为 utf8 编码格式的字符串 (常用)
QByteArray QString::toUtf8() const;

// 所有字符转换为大写
QString QString::toUpper() const;
// 所有字符转换为小写
QString QString::toLower() const;
```

- 字符串格式

```C++
// 其他重载的同名函数可参考Qt帮助文档, 此处略
QString QString::arg(const QString &a, 
          int fieldWidth = 0, 
          QChar fillChar = QLatin1Char( ' ' )) const;
QString QString::arg(int a, int fieldWidth = 0, 
          int base = 10, 
          QChar fillChar = QLatin1Char( ' ' )) const;

// 示例程序
int i;                // 假设该变量表示当前文件的编号
int total;            // 假设该变量表示文件的总个数
QString fileName;     // 假设该变量表示当前文件的名字
// 使用以上三个变量拼接一个动态字符串
QString status = QString("Processing file %1 of %2: %3")
                  .arg(i).arg(total).arg(fileName);
```

## 3.3 这两个类的区别
除了部分函数签名不一样以外, 还有对于相同内容 QString.size() 不一定 == QByteArray.size() 的

因为 QString.size() 得到的是 **字符** 个数.

而 QByteArray.size() 得到的只是 **字节** 个数