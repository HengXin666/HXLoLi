# 无宏反射

[Heng_Xin](https://github.com/HengXin666) 的个人笔记! **禁止转载, 尤其禁止转载 CSDN**!

本文章是 个人学习雅兰亭库的无宏反射机制的学习笔记.

---

## 一、知识准备
### 1.1 C++11 聚合初始化

- [cppreference-聚合初始化](https://cppreference.cn/w/cpp/language/aggregate_initialization)

想要使用聚合初始化, 那么它就必须是一个聚合类 (简单理解, 就是 **没有**`构造函数`并且 **全部** 成员为公开的类)

如:
```C++
// 这个是一个聚合类
struct Man {
    int id;
    std::string name;
};
```

这时候, 就可以使用`聚合初始化`:

```C++
Man{};              // Man{id = 0, name = ""}
Man{2233};          // Man{id = 2233, name = ""}
Man{66, "从不吃素"}; // Man{id = 66, name = "从不吃素"}
```

这个有什么用呢? 我们可以看看:

```C++
Man{0, "", 7}; // 编译报错
```

我们可以发现, 当构造参数个数为 0, 1, 2 时候是不会报错的, 但是当 它个数 > 成员个数的时候, 就会报错. (这不是显然的吗?!)

那这样, 阁下又当如何应对:

> 通过模版, 递归判断是否匹配, 最终得到参数的个数;
>
> (TIP: 甚至你还可以写一个二分算法来计算参数个数..., 虽然性能提升比较小)

```C++
struct Any {
    template <class T>
    operator T(); // 重载了类型转化运算符
};

template <class T, class... Args>
consteval auto membersCount() { // consteval C++20 关键字, 要求该函数必须在编译期
    if constexpr (requires { // C++20 的 约束
        T{ {Args{}}..., Any{} }; // 如果可以 以 Args + 1 个参数构造 T 则为 true
    }/* == true*/) {
        return membersCount<T, Args..., Any>();
    } else {
        return sizeof...(Args);
    }
}

int main() {
    std::cout << membersCount<Man>() << '\n'; // 2
}
```

> $总结$: 现在我们有了一种获取类的成员变量 **个数** 的方法.

> [!TIP]
> 如果不使用C++20的`requires`, 理论上应该可以使用`decltype(T{Any{}})`的思路, 但是我不会...

### 1.2 C++17 结构化绑定

- [cppreference-结构化绑定](https://cppreference.cn/w/cpp/language/structured_binding)

想必各位都知道, 它很方便, 可以直接绑定tuple/pair, 并且类型自动:

```C++
auto [a, b, c] = std::tuple<int, std::string, double>{1, "2", 3.0};
```

但是不仅如此, 它还可以绑定类的成员:

```C++
auto [id, name] = Man{114, "514"};
```

可是, 这样不行呀~

那我问你, 那, 那我问你, 我多个成员怎么适配呢?

通过`C++17的编译期if`可以在编译时候决定, 到时候只需要 $O(1)$ 的调用即可.

通过`访问者模式`进行封装, 方便支持任何操作.

```C++
constexpr decltype(auto) visit_members(
    auto &&obj,
    auto &&visitor
) {
    // 去除&&、&、const, 以获取实际类型
    using ObjType = std::remove_cv_t<std::remove_reference_t<decltype(obj)>>;
    constexpr auto Cnt = membersCount<ObjType>();

    if constexpr (Cnt == 0) {
        return visitor();
    } else if constexpr (Cnt == 1) {
        auto&& [a1] = obj;
        return visitor(a1);
    } else if constexpr (Cnt == 2) {
        auto&& [a1, a2] = obj;
        return visitor(a1, a2);
    } else if constexpr (Cnt == 3) {
        auto&& [a1, a2, a3] = obj;
        return visitor(a1, a2, a3);
    } // ... 写他个255个 ...
    throw;
}
```

> $总结$: 现在我们有了一种获取(使用)类的成员变量 **值** 的方法.

### 1.3 编译器方言 || C++20 source_location

- 编译器方言: [__PRETTY_FUNCTION__(gcc拓展) || __FUNCSIG__(msvc拓展)](https://stackoverflow.org.cn/questions/4384765)

- C++20: [cppreference-std::source_location](https://zh.cppreference.com/w/cpp/utility/source_location)

这里以`__PRETTY_FUNCTION__`为例子 (GCC环境):

```C++
int loli(bool _) {
    std::cout << __PRETTY_FUNCTION__ << '\n'; // int loli(bool)
    return -1;
}
```

这没有什么用, 好吧; 这是在编译的时候, 会被替换为字符串, 内容是`函数签名`.

但是, 如果是模版呢?

```C++
template <class T>
int loli(bool _) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
    return -1;
}

loli<double>(0); // int loli(bool) [with T = double]
loli<Man>(0); // int loli(bool) [with T = Man]
```

似乎也没有什么用, 有类型, 没有 成员名称 啊...

此时就需要: ↓

### 1.4 C++17 auto占位非类型模版形参

> `template<auto val>`可以推导为任意类型，只要该值满足编译期常量的要求。

```C++
template <auto ptr>
inline constexpr std::string getPtrName() {
    return __PRETTY_FUNCTION__;
}

// constexpr std::string getPtrName() [with auto ptr = 'a'; std::string = std::__cxx11::basic_string<char>]
std::cout << getPtrName<'a'>() << '\n';

// constexpr std::string getPtrName() [with auto ptr = 0; std::string = std::__cxx11::basic_string<char>]
int __main__ = [] {
    static const constexpr Man man {0, "0"};
    std::cout << getPtrName<man.id>() << '\n';
    return 0;
}();
```

似乎都无法搞到 成员变量名 啊, 那我问你, 那... 那我问你

但...但是, 还有高手!

### 1.5 符号指针 (编译期指针/常量表达式指针)

编译器在处理指针和普通值时的行为不同, 主要是因为指针的语义和实现需要特别考虑内存地址和指针本身的特性。

- **指针表达式的语义复杂性**: 指针的来源可能是更复杂的表达式, 直接记录内存地址可能丢失了语义信息。

- **符号可重用性**: 通过名称表示, 编译器可以追踪到源代码中指针的定义, 便于优化和分析。

```C++
int __main__ = [] {
    static const /*constexpr*/ Man man {0, "0"};
    // constexpr std::string getPtrName() [with auto ptr = (& man.Man::id); std::string = std::__cxx11::basic_string<char>]
    std::cout << getPtrName<&man.id>() << '\n';

    static int staticPtr = 114514;
    // constexpr std::string getPtrName() [with auto ptr = (& staticPtr); std::string = std::__cxx11::basic_string<char>]
    std::cout << getPtrName<&staticPtr>() << '\n';
    return 0;
}();
```

至此, 我们已经获得了`成员名称`的字符串.

## 二、无宏反射の实现
### 2.1 实现思路
反射得到类成员的变量名, 思路:

1. 通过`1.1`得到`参数个数`
2. 通过`参数个数`选择`结构化绑定`分支
3. 获取到各个成员, 把他们变为`指针`
4. 分别把`各个成员`传入到`auto占位非类型模版形参`模版函数
5. 模版函数通过`__PRETTY_FUNCTION__`得到对应字符串, 在编译期解析到对应的`成员名称`
6. 返回, 剩下的就任君处置~

### 2.2 雅兰亭库

- 见视频 ...

```bilibili ##BV19s6fYwECB##w100%##h320

```

## 三、局限性

- 如你所见, 它只适用于`聚合类`, 对于私有的、有自定义构造函数的类, 就不能这样了...

- 但是大差不差吧, 一般我们都会分一个 DTO、JsonVO来专门存储数据的吧...

## 四、各种反射的对比
> [!TIP]
> 纯个人见解. 我太菜了, 可能说的有不对, 希望原谅qwq..

### 4.1 Portobuf

咕噜咕噜的Portobuf, 太难用啦, 直接让你学它的 .proto 文件语法 (虽然是为了跨语言), 但是你想把它编译出来, 要用 protoc 命令将 .proto 文件转化为相应的 C++ 文件(.cc + .pb.h) (编译出来都不是人看的那种...); 而使用 protoc 命令, 我还要编译那个库, 又是静态库、动态库的, 烦也烦死了...

而做这么多, 仅仅只是为了一行代码的反射...

使用就已经很烦了, 你如果要实现的话, 你要设计这个"独特的语法", 还有对应的语法解析器, 然后再生成出对应的cpp代码...好难

### 4.2 Qt
qt的反射比较无感, 因为配置环境比较"简单", 编译的时候是通过`Q_OBJECT`等宏标记需要支持反射的类、信号、槽等, 在编译时由 Qt 的 MOC 工具解析这些标记, 生成一份额外的代码文件(如元数据表、虚函数重载等)。这些生成的代码与用户代码一起编译, 从而实现了反射功能

你使用的话, 是比较简单的

但是如果你要手撕这种反射, 就需要你自己编写类似于 MOC 工具的编译器, 来生成对应的代码, 还是很难的qwq...

### 4.3 宏
比较简单的, 只需要让用户自己写名称即可:

```C++
#define _REFLECT_TO_JSON(name) \
        res += "\""#name"\":"; \
        res += HX::STL::utils::toString(name); \
        res += ',';
```

然后需要设计一个计算 宏的可变参数个数的宏, 以方便展开 ...

-> 我之前学习时候写的代码: [include/HXJson/ReflectJson.hpp](https://github.com/HengXin666/HXNet/blob/main/include/HXJson/ReflectJson.hpp)

-> 使用示例: [examples/JsonTest.cpp](https://github.com/HengXin666/HXNet/blob/main/examples/JsonTest.cpp)

```C++
#include <HXJson/ReflectJson.hpp> // <-- 反射 宏的头文件: 对外提供的是 以`REFLECT`开头, 以`_`开头的是内部使用的宏

struct Student {
    std::string name;
    int age;
    struct Loli {
        std::string name;
        int age;
        int kawaiiCnt;

        REFLECT_CONSTRUCTOR_ALL(Loli, name, age, kawaiiCnt) // 可以嵌套, 但是也需要进行静态反射(需要实现`toString`方法)
    };
    std::vector<Loli> lolis;
    std::unordered_map<std::string, std::string> woc;
    std::unordered_map<std::string, Loli> awa;

    // 静态反射, 到时候提供`toString`方法以序列化为JSON
    // 提供 构造函数(从json字符串和json构造, 以及所有成员的默认构造函数)
    // 注: 如果不希望生成 [所有成员的默认构造函数], 可以使用 REFLECT_CONSTRUCTOR 宏
    REFLECT_CONSTRUCTOR_ALL(Student, name, age, lolis, woc, awa)
};

/// @brief 一个只读的 Json 反射
struct StudentConst {
    const Student stuConts;
    const int& abc;

    // 这个只反射到toString函数(即序列化为json), 而不能从`jsonStr/jsonObj`构造
    // 你 const auto& 还怎么想从一个临时的jsonObj引用过来? 它本身就不安全, jsonStr就更不用说了!
    REFLECT(stuConts, abc)
};

#include <HXJson/UnReflectJson.hpp> // <-- undef 相关的所有宏的头文件, 因为宏会污染全局命名空间

// JSON 序列化(结构体 toJsonString)示例
void test_02() {
    Student stu { // 此处使用了 宏生成的 [所有成员的默认构造函数] (方便我调试awa)
        "Heng_Xin",
        20,
        {{
            "ラストオーダー",
            13,
            100
        }, {
            "みりむ",
            14,
            100
        }},
        {
            {"hello", "word"},
            {"op", "ed"}
        },
        {
            {"hello", {
                "みりむ",
                14,
                100
            }}
        }
    };
    // 示例: 转化为json字符串(紧凑的)
    HX::print::print(stu.toString());
    auto json = HX::json::parse(stu.toString()).first;
    json.print();
    printf("\n\n");

    // 示例: 从json对象 / json字符串转为 结构体

    json["age"] = HX::json::JsonObject {}; // 如果我们修改了它的类型 / 解析不到对应类型

    Student x(json);
    HX::json::parse(x.toString()).first.print();

    printf("\n\n");
    // 即便是空的也无所谓, 不是json也无所谓, 只是解析到的是空josn对象
    HX::json::parse(Student("Heng_Xin is nb!").toString()).first.print();
}
```

### 4.4 无宏反射

- 即本项目的方案: [src/01-reflection/hansya.cpp](https://github.com/HengXin666/HXTest/blob/main/src/01-reflection/hansya.cpp)

- 学习自: [雅兰亭库 - yaLanTingLibs](https://github.com/alibaba/yalantinglibs)

- 示例: [src/reflection/tests/test_reflection.cpp](https://github.com/alibaba/yalantinglibs/blob/main/src/reflection/tests/test_reflection.cpp)

---

禁止转载:

- 尤其禁止转载到 CSDN!

- 尤其禁止转载到 CSDN!

- 尤其禁止转载到 CSDN!