# 可变参数模版
## 先决条件
要创建`可变参数模版`，需要理解以下几点:

- **函数参数包(parameter pack)**
- 函数参数包
- **展开(unpack)** 参数包
- 递归

## 概述
`C++11`引入了 **可变参数模板（Variadic Template）** 的概念，使得我们可以定义一个可以接受任意数量和任意类型参数的函数或类模板。它们是模板元编程<sup>[2]</sup>中非常强大的工具。

可变参数模板使用`...`语法来表示其可以接受任意数量的参数，可以用于函数模板、类模板以及别名模板等。

### 模版参数包和函数参数包
`C++`提供了这个用省略号`...`表示的**元运算符(meta-operator)**, 让您能够声明表示 **`模版参数包的标识符`**，模版参数包实际上是一个`类型列表`，而函数参数包就是一个`值列表`。

另外，也提供了一个计算可变参数模版个数的方法 `sizeof...([type])`, 具体如下

```C++
// 注: 函数模板的参数个数为0到多个参数，每个参数的类型可以各不相同
template <class... T>   // T 是模版参数包, 是 类型列表
static void fun(T... t) // t 是函数参数包, 是 值列表
{
	//  固定语法格式计算获取到模板参数的个数 (这个可以配合 编译期if表达式 来使用)
	cout << "[info]: " << sizeof...(T) << endl; // 计算模板参数包 参数个数
	cout << "[info]: " << sizeof...(t) << endl; // 计算函数参数包 参数个数
}
```

### 展开参数包
```markmap ##h180##
# 展开参数包的方法
## C++11
### 递归展开
### 逗号表达式
## C++17 新增
### 折叠表达式展开
### if constexpr
```

#### 递归展开

以编写一个 如同`printf()` 的函数为例.<sup>[1]:3{更加详细的C语言实现printf的可变参数模版讲解}</sup>

```C++
static void myPrint(void)
{
	cout << endl;
}

template <class L, class... T>
static void myPrint(L l, T... t)
{
	cout << l << " ";
	myPrint(t...);
}

int main(void)
{
	myPrintf(1433223, 3.14, "寧々", 0.721);
	return 0;
}
```

上面是正确的代码。但你一定有以下疑问:

1. 为什么要重载一个空的函数?
2. 为什么已经是可变参数了，还需要定义一个单类型的模版 `<class L>`?

> 先回复 **質問二**
> 
> 我猜你一定是想这样实现代码:
>
```C++
template <class... T>
static void myPrint(T... t)
{
	cout << t << " ";
	myPrint(t...);
}
```

<style>
  .tab {
    display: inline-block;
    width: 40px; /* 设置 tab 宽度 */
  }
</style>

> 残念ながら、这样写连编译都不通过！
> > 首先，你没有理解 t 是 什么:
> > > `t` 是 函数参数包，一个值列表，我们无法直接使用它。
>
> > 其次，你没有注意到 这个**递归 没有退出条件**!
> > > myPrint(1, 2, 3) // 这个是传参了
> > > 
> > > {
> > > 
> > > <span class="tab"></span>cout << t << " "; // 假设这个没问题
> > > 
> > > <span class="tab"></span>myPrint(1, 2, 3); // 还是这样传参
> > > 
> > > }
>
> 到这里了， 你应该理解了， 所以重载一个传参为的空函数就是为了退出递归!
>
> <span style="color:red">因为 模版参数包 允许参数个数为零传入，如果这样 还继续调用原来这个递归的函数，不就相当于没有终止递归了吗！？</span>

还有一个问题: <<递归展开， 是编译时展开， 还是运行时展开?>> (GPT说是 编译期)

#### 逗号表达式

```C++
//展开函数
template<class ...Args>
void ShowList(Args... args)
{
	int arr[] = { args... }; //列表初始化
	//打印参数包中的各个参数
	for (auto e : arr)
	{
		cout << e << " ";
	}
	cout << endl;
}
 
int main()
{
	ShowList(1);       // 1
	ShowList(1, 2);    // 1 2
	ShowList(1, 2, 3); // 1 2 3
	return 0;
}
```
不过以上这样不能适应全部的类型.

所以可以使用 完美转发 + 右值<sup>[1]:5</sup>

```C++
template<class F, class... Args>
void expand(const F& f, Args&&...args) 
{
  //这里用到了完美转发，关于完美转发，读者可以参考笔者在上一期程序员中的文章《通过4行代码看右值引用》(^[1]:5)
  initializer_list<int>{(f(std::forward< Args>(args)),0)...};
}

expand([](int i){cout<<i<<endl;}, 1,2,3);
```


#### 折叠表达式
> <span style="color:red">这个是C++17引入的新特性!</span>
>
> 参考文献: C++17 折叠表达式<sup>[1]:4</sup>
>
> 请前往这个文章: [折叠表达式](../../002-C++17常用新特性/001-折叠表达式/index.md)

#### 编译期if表达式
> <span style="color:red">这个是C++17引入的新特性!</span>
>
> 请前往这个文章:[编译期if表达式](../../002-C++17常用新特性/002-编译期if表达式/index.md)

## 参考文献 与 注解
### [1]
参考文献:

1. [CSDN:【C++11】晦涩难懂语法系列：可变参数模板](https://blog.csdn.net/m0_64280701/article/details/130241336)
2. [CSDN:【C++】C++11可变参数模板（函数模板、类模板）](https://blog.csdn.net/qq_38410730/article/details/105247065)
3. [知乎:C++的可变参数模板](https://zhuanlan.zhihu.com/p/104450480)
4. [知乎:C++17 折叠表达式](https://zhuanlan.zhihu.com/p/625884960)
5. [博客园:泛化之美--C++11可变模版参数的妙用](https://www.cnblogs.com/qicosmos/p/4325949.html)
6. C++ Primer Plus

### [2]

**元编程（Metaprogramming）** 是一种在程序运行期间生成代码的技术，它允许程序员在编写代码时使用编程语言本身作为工具来创建代码，而不是手动编写代码。元编程可以用来实现许多高级功能，比如代码生成、编译时优化、类型推导、模板特化等。

*元编程并不是泛型编程的代名词，虽然两者都是C++中的重要编程技术，但它们有不同的应用场景和使用方法。*

**泛型编程（Generic Programming）** 是一种`编程范式`，旨在通过使用模板和参数化类型来实现算法的通用性和复用性。泛型编程的目标是实现通用的数据结构和算法，以便它们可以适用于各种数据类型和应用场景，从而减少代码重复和提高开发效率。泛型编程中的模板通常用于编写函数和类模板，以便它们可以接受不同类型的参数。

虽然`元编程`和`泛型编程`有不同的应用场景，但它们都使用了C++中的模板技术。元编程通常使用 **模板元编程（Template Metaprogramming）** 技术，而泛型编程则使用 **参数化类型（Parameterized Types）** 技术。