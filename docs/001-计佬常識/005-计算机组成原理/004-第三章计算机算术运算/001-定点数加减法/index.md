# 定点数加减法
## 定点数
**定点数**: 是一种表示固定小数位数的数值类型，通常表示为一个有符号的整数部分和一个小数部分。

**定点小数**: 如果是有符号数，约定小数点在符号位的后面，如果是无符号数，约定小数点在最前边。定点小数的是类似: `0.XXXXXX`这个形式

**定点整数**: 约定小数点在最后边。定点整数是类似: `XXXXXX.0`这种形式

**浮点数**: 相当于生活中的科学计数法，小数点的位置是可以不断变化的；且小数点位置的变化，不会影响表达式整体数值的大小。`IEEE 754`是由`IEEE`制定的有关浮点数的工业标准，被广泛采用。

在计算机中,通常是用定点数来表示整数和纯小数,分别称为定点整数和定点小数。对于既有整数部分、又有小数部分的数，一般用浮点数表示。

<div style="margin-top: 80px;">

---
</div>

## 定点数加减法
在计算机中，`有符号数`一般采用**补码**表示。

**加法**: 一种基本且频繁使用的操作。现代计算机在硬件层面上实现加法操作，通常是通过<span style="color:yellow">加法器电路</span>来完成的。加法器会接收两个二进制数作为输入，然后按照二进制加法规则进行逐位相加，同时处理可能出现的进位。

**减法**: 虽然其基本原理与手动计算中的减法相似，但在计算机内部实现时，通常会转换为加法来执行。这是因为计算机内部的电路更擅长执行加法操作，而减法操作可以通过取反和加法的组合来实现。

虽然从原理上看，补码减法是通过取反和加法实现的，但在实际的计算机硬件中，这个过程通常是自动且高效地完成的，无需程序员显式地进行取反和加法的操作。程序员在编写代码时，只需要使用减法运算符(如`-`)，然后计算机硬件和操作系统会自动处理底层的细节。

<div style="margin-top: 80px;">

---
</div>

## 算术逻辑单元

一般情况下,用一个专门的算术逻辑部件(ALU)来完成基本逻辑运算和定点数加减运算,各类定点乘除运算和浮点数运算则可利用加法器或 ALU 和移位器来实现。ALU 的核心部件是加法器。

**算术逻辑单元(Arithmetic Logic Unit，简称ALU)** 是计算机处理器内部的一个核心组件，负责执行基本的算术和逻辑运算。它是一个能够处理二进制数据并进行快速计算的硬件电路块。在功能上，ALU主要包括以下几个方面:
1. 算术运算
2. 逻辑运算
3. 比较运算
4. 移位运算
    - 注意区分:
    - 算术移位: 左移补0, 右移补符号位 (`>>`)
    - 逻辑移位: 左移/右移都补0, (可以看成是无符号数的移位(比如Javaの`<<<`))
5. 其他复合运算

<div style="margin-top: 80px;">

---
</div>

## 加法器

**加法器** 是计算机中的一种数位电路，用于执行数字的加法计算。在电子学中，加法器常用于各种数值的表示和计算，特别是在二进制运算中。在计算机和一些处理器中，加法器被运用于算术逻辑单元ALU中，或者处理器的其他部分，如计算地址、执行加减操作等类似功能。因此，加法器是算术逻辑单元中的一个重要部件。

> 注: 具体电路会在后面详细讲解

## 原码 & 补码 の 运算

原码(符号位不参与运算):
- 加法运算: 同号相加，符号位不变，数值位相加
- 减法运算: <span style="color:red">先判断两个数大小确定结果的符号，绝对值大的减去绝对值小的</span>

补码(符号位参与运算):
- 加法运算: 补码直接相加
- 减法运算: <span style="color:red">被减数的补码 + 减数相反数的补码</span>

在计算机中，有符号数一般采用补码表示。重点关注补码运算。

## 补码定点数加减溢出: 正溢出 & 负溢出

正溢出 => 正数 + 正数 = 负数

负溢出 => 负数 + 负数 = 正数

溢出检测方法:

1. 和的符号位与两个加数的符号位不同

> 以 8 位补码数来举例 (就是一个char嘛~)
>
> 124 + 8 = ? (-124)
> 
> ```C++
>   0111 1100
> + 0000 1000
> ------------
>   1000 0100 (补)
>   1000 0011 (反)
>   1111 1100 (源: -(111 1100)_2 = -124)
> ```

2. 最高位和次高位的进位不同

> 同样是这个例子: 124 + 8 = ?
> 
> ```C++
>   0111 1100
> + 0000 1000
> ------------
>   1000 0100
> ```
> 最高位进位: 0 + 0 = 0
> 
> 次高位进位: 0 + 1 = 1
> 
> 最高位进位 != 次高位进位 => 溢出
>
> 怎么实现呢?
> - `if ((最高位进位 ^ 次高位进位) == true) printf("溢出!\n")`

3. 双符号位判别

> 同上[2]的原理