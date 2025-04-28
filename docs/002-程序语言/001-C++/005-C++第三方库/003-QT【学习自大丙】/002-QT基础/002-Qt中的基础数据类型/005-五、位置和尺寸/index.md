# 五、位置和尺寸
在QT中我们常见的 点, 线, 尺寸, 矩形 都被进行了封装, 下边依次为大家介绍相关的类。

## 5.1 QPoint (坐标类)
> QPoint类封装了我们常用用到的坐标点 (x, y), 常用的 API如下:

```C++
// 构造函数
// 构造一个坐标原点, 即(0, 0)
QPoint::QPoint();
// 参数为 x轴坐标, y轴坐标
QPoint::QPoint(int xpos, int ypos);

// 设置x轴坐标
void QPoint::setX(int x);
// 设置y轴坐标
void QPoint::setY(int y);

// 得到x轴坐标
int QPoint::x() const;
// 得到y轴坐标
int QPoint::y() const;

// 添加r, 可以获得坐标的`引用` 
// 得到x轴坐标的引用
int &QPoint::rx();
// 得到y轴坐标的引用
int &QPoint::ry();

// 直接通过坐标对象进行算术运算: 加减乘除
QPoint &QPoint::operator*=(float factor);
QPoint &QPoint::operator*=(double factor);
QPoint &QPoint::operator*=(int factor);
QPoint &QPoint::operator+=(const QPoint &point);
QPoint &QPoint::operator-=(const QPoint &point);
QPoint &QPoint::operator/=(qreal divisor);

// 其他API请自行查询Qt帮助文档, 不要犯懒哦哦哦哦哦......
```

## 5.2 QLine (线段类)
QLine是一个直线类, 封装了两个坐标点 (`两点确定一条直线`)

常用API如下:

```C++
// 构造函数
// 构造一个空对象
QLine::QLine();
// 构造一条直线, 通过两个坐标点
QLine::QLine(const QPoint &p1, const QPoint &p2);
// 从点 (x1, y1) 到 (x2, y2)
QLine::QLine(int x1, int y1, int x2, int y2);

// 给直线对象设置坐标点
void QLine::setPoints(const QPoint &p1, const QPoint &p2);
// 起始点(x1, y1), 终点(x2, y2)
void QLine::setLine(int x1, int y1, int x2, int y2);
// 设置直线的起点坐标
void QLine::setP1(const QPoint &p1);
// 设置直线的终点坐标
void QLine::setP2(const QPoint &p2);

// 返回直线的起始点坐标
QPoint QLine::p1() const;
// 返回直线的终点坐标
QPoint QLine::p2() const;
// 返回值直线的中心点坐标, (p1() + p2()) / 2
QPoint QLine::center() const;

// 返回值直线起点的 x 坐标
int QLine::x1() const;
// 返回值直线终点的 x 坐标
int QLine::x2() const;
// 返回值直线起点的 y 坐标
int QLine::y1() const;
// 返回值直线终点的 y 坐标
int QLine::y2() const;

// 用给定的坐标点平移这条直线
void QLine::translate(const QPoint &offset);
void QLine::translate(int dx, int dy);
// 用给定的坐标点平移这条直线, 返回平移之后的坐标点 (this 不变, 而是创建一个新的QLine (毕竟是const!))
QLine QLine::translated(const QPoint &offset) const;
QLine QLine::translated(int dx, int dy) const;

// 直线对象进行比较
bool QLine::operator!=(const QLine &line) const;
bool QLine::operator==(const QLine &line) const;

// 其他API请自行查询Qt帮助文档, 不要犯懒哦哦哦哦哦......
```

## 5.3 QSize (尺寸类)
在QT中QSize类用来形容长度和宽度, 常用的API如下:

```C++
// 构造函数
// 构造空对象, 对象中的宽和高都是无效的
QSize::QSize();
// 使用宽和高构造一个有效对象
QSize::QSize(int width, int height);

// 设置宽度
void QSize::setWidth(int width)
// 设置高度
void QSize::setHeight(int height);

// 得到宽度
int QSize::width() const;
// 得到高度
int QSize::height() const;

// 照旧: + `r`在前, 得`引用`
// 得到宽度的引用
int &QSize::rwidth();
// 得到高度的引用
int &QSize::rheight();

// 交换高度和宽度的值
void QSize::transpose();
// 交换高度和宽度的值, 返回交换之后的尺寸信息 (this 不变, 而是创建一个新的QSize)
QSize QSize::transposed() const;

// 进行算法运算: 加减乘除
QSize &QSize::operator*=(qreal factor);
QSize &QSize::operator+=(const QSize &size);
QSize &QSize::operator-=(const QSize &size);
QSize &QSize::operator/=(qreal divisor);

// 其他API请自行查询Qt帮助文档, 不要犯懒哦哦哦哦哦......
```

## 5.4 QRect (矩形类)
在Qt中使用 QRect类来描述一个矩形, 常用的API如下:

```C++
// 构造函数
// 构造一个空对象
QRect::QRect();
// 基于左上角坐标, 和右下角坐标构造一个矩形对象
QRect::QRect(const QPoint &topLeft, const QPoint &bottomRight);
// 基于左上角坐标, 和 宽度, 高度构造一个矩形对象
QRect::QRect(const QPoint &topLeft, const QSize &size);
// 通过 左上角坐标(x, y), 和 矩形尺寸(width, height) 构造一个矩形对象
QRect::QRect(int x, int y, int width, int height);

// 设置矩形的尺寸信息, 左上角坐标不变
void QRect::setSize(const QSize &size);
// 设置矩形左上角坐标为(x,y), 大小为(width, height)
void QRect::setRect(int x, int y, int width, int height);
// 设置矩形宽度
void QRect::setWidth(int width);
// 设置矩形高度
void QRect::setHeight(int height);

// 返回值矩形左上角坐标
QPoint QRect::topLeft() const;
// 返回矩形右上角坐标
// 该坐标点值为: QPoint(left() + width() -1, top())
QPoint QRect::topRight() const;
// 返回矩形左下角坐标
// 该坐标点值为: QPoint(left(), top() + height() - 1)
QPoint QRect::bottomLeft() const;
// 返回矩形右下角坐标
// 该坐标点值为: QPoint(left() + width() -1, top() + height() - 1)
QPoint QRect::bottomRight() const;
// 返回矩形中心点坐标
QPoint QRect::center() const;

// 返回矩形上边缘y轴坐标
int QRect::top() const;
int QRect::y() const;

// 返回值矩形下边缘y轴坐标
int QRect::bottom() const;

// 返回矩形左边缘 x轴坐标
int QRect::x() const;
int QRect::left() const;

// 返回矩形右边缘x轴坐标
int QRect::right() const;

// 返回矩形的高度
int QRect::width() const;
// 返回矩形的宽度
int QRect::height() const;
// 返回矩形的尺寸信息
QSize QRect::size() const;
```
