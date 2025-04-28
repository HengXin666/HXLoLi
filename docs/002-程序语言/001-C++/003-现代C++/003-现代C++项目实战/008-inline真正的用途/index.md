# 【C++辟谣】inline关键字的作用

- 视频: [【C++辟谣】inline关键字的作用是“内联优化”？你可能是谭浩强的受害者！inline真正的用途是...](https://www.bilibili.com/video/BV1Mg4y1d79w/)

- [inline 说明符](https://zh.cppreference.com/w/cpp/language/inline)

> 在现代编译器中, `inline`不是使其被`"内联优化"`, 而是用于声明`链接属性`!
>
> *现代编译器很智能, 它自己会决定是否内联、是否优化* (-> [四、从汇编角度看编译器优化](https://blog.HXLoLi.com/blog/#/articles?articleId=20914 "##20914##"))

## 一、链接属性

C++中, 有以下三种链接属性:

```C++
static // 文件内可见, 限定作用域为声明所在的源文件

inline // 允许多次定义(跨翻译单元一致), 修改链接属性以支持内联声明

extern // 用于声明外部变量或函数, 提供跨翻译单元的共享访问
```

小例子:

```C++
// head.h
#pragma once
#include <cstdio>
#include <iostream>

// 注意此处的`inline`
inline void look() {}

void fun();
```

```C++
// head.cpp
#include "head.h"

void fun() {
    printf("fun看到的: %p\n", &look);
}
```

```C++
// main.cpp
#include "head.h"

int main() {
    fun();
    printf("main看到的 %p\n", &look);
    return 0;
}
```

输出:

```C++
fun看到的: 0x5b251a3cd1dc
main看到的 0x5b251a3cd1dc
```

> - 如果我们把上面的`inline`改为`static`:
> 
> 输出:
> 
> ```C++
> fun看到的: 0x5e6dacd4f179
> main看到的 0x5e6dacd4f1e3
> ```
>
> *值得注意的是, 如果换为`inline static`/`static inline`输出同上(两个函数地址不同)*

函数的地址不同, 则说明: <span style="color:gold">编译器生成了多份二进制实体</span>

因此, 如果在头文件中实现某些函数, 需要使用`inline`声明, 让编译器去重, 以减小最终的二进制文件的体积.

<details>
<summary>更加明显的示例: (使用静态变量来验证~)</summary>

```C++
// head.h
#pragma once
#include <cstdio>
#include <iostream>
#include <string>

static void look(const std::string& str) {
    static struct _ {
        explicit _(const std::string& str) {
            std::cout << str << '\n';
        }
    } _{str};
}

void fun();
```

```C++
// head.cpp
#include "head.h"

void fun() {
    printf("fun看到的: %p\n", &look);
    look("fun");
    look("yyy");
}
```

```C++
// main.cpp
#include "head.h"

int main() {
    fun();
    printf("main看到的 %p\n", &look);
    look("main");
    look("xxx");
    return 0;
}
```

输出:

```C++
fun看到的: 0x59a491cfa320
fun
main看到的 0x59a491cfb9c6
main
```

---

替换为`inline`:

```C++
inline void look(const std::string& str) {
    static struct _ {
        explicit _(const std::string& str) {
            std::cout << str << '\n';
        }
    } _{str};
}
```

输出:

```C++
fun看到的: 0x5851a486a7be
fun
main看到的 0x5851a486a7be
```

</details>

> [!TIP]
> 顺带一提, 上面的情况, 不能将`inline`/`static`去掉 (或者换成`extern`), 这都是会报错的.
>
> 而如果使用模版、constexpr、consteval、在结构体、类、共用体内实现的函数, 都是默认为`inline`的, 除非你显式书写了. (具体请看c++文档)

> [!NOTE]
> 
> ```C++
> // 头文件
> inline int i;
> ```
>
> 上面的操作对于`全局变量`也同理(`static`在头文件, 则互不影响; `inline`则共用; `extern`就报错)

## 二、一些相关の危険的な行為
### 2.1 结构体函数声明实现の分离

- 我们知道: 整个定义都在`class/struct/union`的定义内的函数都是隐式的内联函数, 无论它是成员函数还是非成员 friend 函数。

> [!TIP]
> 我在下面代码的注释中, 标注了此时函数的链接属性.

```C++
struct Dog {
    /*inline*/ void eat() {}
};
```

---

可是, 以下方式使用, 则是`extern`链接属性:

```C++
// .h
struct Dog {
    /*extern*/ void eat();
};

// .cpp
Dog::eat() {}
```

这看起来也没问题, 这很正常.

---

但是:

```C++
// .h
struct Dog {
    /*extern*/ void eat();
};

// 仍然在 .h
/*extern*/ void Dog::eat() {}
```

如果有两个`.cpp`都`#include`这个头文件; 那么它就会报错(重定义):

```C++
[build] /usr/bin/ld: CMakeFiles/02-link-property.dir/main.cpp.o: in function `std::allocator<char>::~allocator()':
[build] /usr/include/c++/14.2.1/bits/basic_string.tcc:221: multiple definition of `Dog::eat()'; CMakeFiles/02-link-property.dir/head.cpp.o:/root/HXcode/HXTest/src/02-link-property/head.h:44: first defined here
```

因此我们需要把任意一处修改为`inline`, 这样就没问题了.

### 2.2 ub: `inline`声明, 但多份不同实现

```C++
// 头文件
inline void myShow();

// A.cpp文件
inline void myShow() {
    printf("在.cpp文件\n");
}

// main.cpp文件
inline void myShow() {
    printf("在main文件\n");
}
```

> ODR(One Definition Rule, 唯一定义规则):
> - C++ 标准要求, 同一个函数在所有翻译单元中的定义必须完全一致。对于 inline 函数, 编译器会在每个包含该函数定义的翻译单元中生成一份相同的实现, 并且在链接时合并为一个定义。如果实现不同, 会违反 ODR, 从而导致未定义行为。

上面的后果是, 一个实现被另一个实现覆盖, 导致`运行时行为不可预测`.

因此, 最好我们把`inline`都在头文件中实现, 而不要分离!

### 2.3 ub: 多个.cpp同一个结构体, 但成员不同

```C++
// A.cpp
struct C {
    int v;
    std::string s;
};

// B.cpp
struct C {
    int x, y;
};
```

这两个文件, 一起编译, 会触发ut.

因为`struct C`的默认构造函数是`inline C() = default`的, 可能会导致只使用其中一个作为构造函数, 导致某些成员没有被初始化/初始化错误.

解决方案:

1. 统一结构定义: 放头文件
    - ps: 因为头文件不能`static struct`(静态链接属性结构体), 因此只能放头文件.

2. 用名字区分
    - ps: 如果我不知道另外一个的存在, 不就丸辣?
  
3. 使用命名空间 (*jo级* 解决方案)

```C++
// A.cpp
namespace { // 匿名命名空间即可
    struct C {
        int v;
        std::string s;
    };
}

// B.cpp
namespace {
    struct C {
        int x, y;
    };
}
```
