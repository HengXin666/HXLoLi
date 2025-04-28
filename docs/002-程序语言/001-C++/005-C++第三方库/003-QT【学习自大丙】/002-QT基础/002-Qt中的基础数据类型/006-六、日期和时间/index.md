# 六、日期和时间
## 6.1 QDate
QDate类可以封装日期信息也可以通过这个类得到日期相关的信息, 包括: 年, 月, 日

```C++
// 构造函数
QDate::QDate();
QDate::QDate(int y, int m, int d);

// 公共成员函数
// 重新设置日期对象中的日期
bool QDate::setDate(int year, int month, int day);
// 给日期对象添加 ndays 天
QDate QDate::addDays(qint64 ndays) const;
// 给日期对象添加 nmonths 月
QDate QDate::addMonths(int nmonths) const;
// 给日期对象添加 nyears 月
QDate QDate::addYears(int nyears) const;

// 得到日期对象中的年/月/日
int QDate::year() const;
int QDate::month() const;
int QDate::day() const;
void QDate::getDate(int *year, int *month, int *day) const;

// 日期对象格式化
/*
    d    - The day as a number without a leading zero (1 to 31)
    dd   - The day as a number with a leading zero (01 to 31)
    ddd     - The abbreviated localized day name (e.g. 'Mon' to 'Sun'). Uses the system locale to localize the name, i.e. QLocale::system().
    dddd - The long localized day name (e.g. 'Monday' to 'Sunday'). Uses the system locale to localize the name, i.e. QLocale::system().
    M    - The month as a number without a leading zero (1 to 12)
    MM   - The month as a number with a leading zero (01 to 12)
    MMM     - The abbreviated localized month name (e.g. 'Jan' to 'Dec'). Uses the system locale to localize the name, i.e. QLocale::system().
    MMMM - The long localized month name (e.g. 'January' to 'December'). Uses the system locale to localize the name, i.e. QLocale::system().
    yy   - The year as a two digit number (00 to 99)
    yyyy - The year as a four digit number. If the year is negative, a minus sign is prepended, making five characters.
*/
QString QDate::toString(const QString &format) const;

// 操作符重载 ==> 日期比较
bool QDate::operator!=(const QDate &d) const;
bool QDate::operator<(const QDate &d) const;
bool QDate::operator<=(const QDate &d) const;
bool QDate::operator==(const QDate &d) const;
bool QDate::operator>(const QDate &d) const;
bool QDate::operator>=(const QDate &d) const;

// 静态函数 -> 得到本地的当前日期
[static] QDate QDate::currentDate();
```

## 6.2 QTime
QTime类可以封装时间信息也可以通过这个类得到时间相关的信息, 包括: 时, 分, 秒, 毫秒

```C++
// 构造函数
QTime::QTime();
/*
    h         ==> 取值范围： 0 ~ 23
    m and s     ==> 取值范围： 0 ~ 59
    ms         ==> 取值范围： 0 ~ 999
*/ 
QTime::QTime(int h, int m, int s = 0, int ms = 0);

// 公共成员函数
// Returns true if the set time is valid; otherwise returns false.
bool QTime::setHMS(int h, int m, int s, int ms = 0);
QTime QTime::addSecs(int s) const;
QTime QTime::addMSecs(int ms) const;

// 示例代码
  QTime n(14, 0, 0);                // n == 14:00:00
  QTime t;
  t = n.addSecs(70);                // t == 14:01:10
  t = n.addSecs(-70);               // t == 13:58:50
  t = n.addSecs(10 * 60 * 60 + 5);  // t == 00:00:05
  t = n.addSecs(-15 * 60 * 60);     // t == 23:00:00

// 从时间对象中取出 时/分/秒/毫秒
// Returns the hour part (0 to 23) of the time. Returns -1 if the time is invalid.
int QTime::hour() const;
// Returns the minute part (0 to 59) of the time. Returns -1 if the time is invalid.
int QTime::minute() const;
// Returns the second part (0 to 59) of the time. Returns -1 if the time is invalid.
int QTime::second() const;
// Returns the millisecond part (0 to 999) of the time. Returns -1 if the time is invalid.
int QTime::msec() const;


// 时间格式化
/*
    -- 时 --
    h    ==>    The hour without a leading zero (0 to 23 or 1 to 12 if AM/PM display)
    hh    ==>    The hour with a leading zero (00 to 23 or 01 to 12 if AM/PM display)
    H    ==>    The hour without a leading zero (0 to 23, even with AM/PM display)
    HH    ==>    The hour with a leading zero (00 to 23, even with AM/PM display)
    -- 分 --
    m    ==>    The minute without a leading zero (0 to 59)
    mm    ==>    The minute with a leading zero (00 to 59)
    -- 秒 --
    s    ==>    The whole second, without any leading zero (0 to 59)
    ss    ==>    The whole second, with a leading zero where applicable (00 to 59)
    -- 毫秒 --
    zzz    ==>    The fractional part of the second, to millisecond precision, 
            including trailing zeroes where applicable (000 to 999).
    -- 上午或者下午 (用这个的话, 小时为12小时制 啦哇~)
    AP or A    ==>    使用AM/PM(大写) 描述上下午, 中文系统显示汉字
    ap or a    ==>    使用am/pm(小写) 描述上下午, 中文系统显示汉字
*/
QString QTime::toString(const QString &format) const;

// 阶段性计时
// [过时]的API函数
// 开始计时
void QTime::start();
// 计时结束
int QTime::elapsed() const;
// 重新计时
int QTime::restart();

// 推荐使用的API函数 (~~不如STL的 嘿嘿~~)
// QElapsedTimer 类
void QElapsedTimer::start();
qint64 QElapsedTimer::restart();
qint64 QElapsedTimer::elapsed() const;


// 操作符重载 ==> 时间比较
bool QTime::operator!=(const QTime &t) const;
bool QTime::operator<(const QTime &t) const;
bool QTime::operator<=(const QTime &t) const;
bool QTime::operator==(const QTime &t) const;
bool QTime::operator>(const QTime &t) const;
bool QTime::operator>=(const QTime &t) const;

// 静态函数 -> 得到当前时间
[static] QTime QTime::currentTime();
```

## 6.3 QDateTime
QDateTime类可以封装日期和时间信息也可以通过这个类得到日期和时间相关的信息, 包括:年, 月, 日, 时, 分, 秒, 毫秒。其实这个类就是QDate 和 QTime 这两个类的结合体。

```C++
// 构造函数
QDateTime::QDateTime();
QDateTime::QDateTime(const QDate &date, const QTime &time, Qt::TimeSpec spec = Qt::LocalTime);

// 公共成员函数
// 设置日期
void QDateTime::setDate(const QDate &date);
// 设置时间
void QDateTime::setTime(const QTime &time);
// 给当前日期对象追加 年/月/日/秒/毫秒, 参数可以是负数
QDateTime QDateTime::addYears(int nyears) const;
QDateTime QDateTime::addMonths(int nmonths) const;
QDateTime QDateTime::addDays(qint64 ndays) const;
QDateTime QDateTime::addSecs(qint64 s) const;
QDateTime QDateTime::addMSecs(qint64 msecs) const;

// 得到对象中的日期
QDate QDateTime::date() const;
// 得到对象中的时间
QTime QDateTime::time() const;

// 日期和时间格式, 格式字符参考QDate 和 QTime 类的 toString() 函数
QString QDateTime::toString(const QString &format) const;


// 操作符重载 ==> 日期时间对象的比较
bool QDateTime::operator!=(const QDateTime &other) const;
bool QDateTime::operator<(const QDateTime &other) const;
bool QDateTime::operator<=(const QDateTime &other) const;
bool QDateTime::operator==(const QDateTime &other) const;
bool QDateTime::operator>(const QDateTime &other) const;
bool QDateTime::operator>=(const QDateTime &other) const;

// 静态函数
// 得到当前时区的日期和时间(本地设置的时区对应的日期和时间)
[static] QDateTime QDateTime::currentDateTime();
```

示例:

```C++
auto now = QDateTime::currentDateTime();
qDebug() << now.toString("yyyy-MM-dd hh:mm:ss.ms") << '\n';
```
