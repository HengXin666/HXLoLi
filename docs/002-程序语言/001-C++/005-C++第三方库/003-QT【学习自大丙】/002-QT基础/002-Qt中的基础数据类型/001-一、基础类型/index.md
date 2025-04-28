# 一、基础类型
> [!TIP]
> 认识即可, 日常项目中使用c++的原生类型就OK(如 int 等)

QT基本数据类型定义在`#include <QtGlobal>`中，QT基本数据类型有:

| 类型名称 | 注释 | 备注 |
| --- | --- | --- |
| qint8 | signed char | 有符号8位数据 |
| qint16 | signed short | 16位数据类型 |
| qint32 | signed short | 32位有符号数据类型 |
| qint64 | long long int 或(\_\_int64) | 64位有符号数据类型，Windows中定义为\_\_int64 |
| qintptr | qint32 或 qint64 | 指针类型 根据系统类型不同而不同，32位系统为qint32、64位系统为qint64 |
| qlonglong | long long int 或(\_\_int64) | Windows中定义为\_\_int64 |
| qptrdiff | qint32 或 qint64 | 根据系统类型不同而不同，32位系统为qint32、64位系统为qint64 |
| qreal | double 或 float | 除非配置了-qreal float选项，否则默认为double |
| quint8 | unsigned char | 无符号8位数据类型 |
| quint16 | unsigned short | 无符号16位数据类型 |
| quint32 | unsigned int | 无符号32位数据类型 |
| quint64 | unsigned long long int 或 (unsigned \_\_int64) | 无符号64比特数据类型，Windows中定义为unsigned \_\_int64 |
| quintptr | quint32 或 quint64 | 根据系统类型不同而不同，32位系统为quint32、64位系统为quint64 |
| qulonglong | unsigned long long int 或 (unsigned \_\_int64) | Windows中定义为\_\_int64 |
| uchar | unsigned char | 无符号字符类型 |
| uint | unsigned int | 无符号整型 |
| ulong | unsigned long | 无符号长整型 |
| ushort | unsigned short | 无符号短整型 |

虽然在Qt中有属于自己的整形或者浮点型, 但是在变成过程中这些一般不用, 常用的类型关键字还是 C/C++中的 int, float, double 等。