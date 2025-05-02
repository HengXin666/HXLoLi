# decltype
## 说明
`decltype`，在C++中，作为**操作符**，用于**查询表达式的数据类型**。

decltype在C++11标准制定时引入，主要是为泛型编程而设计，以解决泛型编程中，由于有些类型由模板参数决定，而难以（甚至不可能）表示的问题。

decltype 关键字是为了解决 auto 关键字只能对`变量`进行类型推导的缺陷而出现的。它的用法和 sizeof 很相似。

在此过程中，编译器分析表达式并得到它的类型，却不实际计算表达式的值。有时候，我们可能需要计算某个表达式的类型

lambda表达式如果我们想要使用它的类型我们就需要使用decltype

## 用法
### 声明变量类型

```C++
auto num1 = 10;
auto num2 = 20;
decltype(num1 + num2) num3;
// auto num3; // 报错
// auto num3 = num1 + num2; // 可以, 但注意, 这里不需要它的计算结果啊
```

### 自动推导返回值类型

```C++
template <class R, class T, class U>
R add_1(T a, U b)
{
	return a + b;
}

// 函数模版, 会自动推导 参数列表的类型, 但是不会推导返回值的类型
int cs = 10;
double cs_2 = 21.1;
auto res = add_1<decltype(cs + cs_2)>(cs, cs_2);
// 等价<decltype(cs + cs_2), int, double>
```

### 尾随返回类型

考虑下面代码, 是否正确?


```C++
template <class T, class U>
decltype(a + b) add_2(T a, U b)
{
	return a + b;
}
```

*答案是错误的!*, 为什么呢?
> 因为此时 a, b 形参名均**未声明**(decltype处)

可以如下使用:
像 Go , Python 等一样, 尾随类型返回.
```C++
template <class T, class U>
static auto add_3(T a, U b) -> decltype(a + b)
{
	return a + b;
}
```

不过这个只是提高了代码的可读性. 
因为从`C++14`开始, 就不是必需的了(但是为了代码的兼容性依旧可以写呢). 如下也是可行的:

```C++
template <class T, class U>
static auto add_4(T a, U b)
{
	return a + b;
}
```
