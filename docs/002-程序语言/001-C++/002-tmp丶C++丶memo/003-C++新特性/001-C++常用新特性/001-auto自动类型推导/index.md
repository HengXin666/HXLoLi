# auto 类型自动推导

## 警告
> ##red##
> 🔴 <span style="color:green">注意: 这里主要是讲解`C++11`的新特性, 可能有办法会在新版本修复!</span>

## 介绍
在C++11之前，auto关键字用来指定`存储期/生命周期`。在新标准中，它的功能变为`类型推断`。

> 历史: `C++98` 
> > 早在C++98标准中就存在了auto关键字，那时的auto用于声明变量为自动变量，自动变量意为拥有自动的生命期，这是`多余`的，因为就算不使用auto声明，变量依旧拥有自动的生命期：
```C++
int a = 10;  		// 拥有自动生命期
auto int b = 20 ;	// 拥有自动生命期
static int c = 30 ;	// 延长了生命期
```

auto现在成了一个类型的占位符，通知`编译器`去根据`初始化代码`推断所声明变量的真实类型。

各种作用域内声明变量都可以用到它。例如，名空间中，程序块中，或是for循环的初始化语句中。

#### 示例 auto代替迭代器

```C++
// 1. auto代替迭代器
vector<int> arr = { 1,2,3,4,5 };
for (vector<int>::iterator i = arr.begin(); i != arr.end(); ++i)
{
    cout << *i << endl;
}

for (auto i = arr.begin(); i != arr.end(); ++i)
{
    cout << *i << endl;
}
```

## auto 与 const

$例:$ 请看下面代码, 尝试推导其`auto`是什么类型.

```C++
int x = 0;
const auto n = x;
auto f = n;
const auto& r1 = x;
auto& r2 = r1;
```
1. 第2行代码中，n为const int，auto 被推导为int。
2. 第3行代码中，n为const int类型，但是**auto却被推导为int类型**，这说明当=右边的表达式带有const属性时auto不会使用const属性，而是直接推导出non-const类型。
3. 第4行代码中，auto被推导为int类型，这个很容易理解，不再赘述。
4. 第5行代码中，r1是const int&类型，auto也被推导为const int类型，这说明当const和引l用结合时，auto的推导将保留表达式的const类型。

**总结：**<sup>[1]{补充}</sup>
    
1. 当类型不为`引用`时，auto的推导结果将`不保留`表达式的const属性；
 
2. 当类型为`引用`时，auto的推导结果将`保留`表达式的const属性。

## auto 高阶用法
### "半个类型"
> auto 除了可以独立使用, 还可以混合使用. 即表示半个类型, 而不是一个类型.

```C++
int x= 0;
auto *pt1 = &x;	// pt1 为 int*, auto -> int
auto pt2 = &x;  // pt2 为 int*, auto -> int*
auto &r1 = x;	// pt3 为 int&, auto -> int
auto r2 = r1;	// r2  为 int , auto -> int
```

### 充当函数返回值 `C++14`
> ##red##
> <span style="color:green">[!] 注意: 这里主要是讲解`C++11`的新特性, 这个方法在`C++11`中并 $不适用$!</span>
```C++
auto sum(int a, int b)
{
	return a + b;
}
```

## auto 的限制
### 不能在函数参数中使用
例如下面是**错误示范**:

```C++
int sum(auto a, int b) // 编译错误
{
	return a + b;
}
```

### 不能用于类的非静态成员变量

> 实践发现, 实际上静态成员变量 也只能在 初始化的时候使用auto, 声明依旧不能用.

看起来没有问题, 但是也是会报错 `a` 重定义.
```C++
static class Demo
{
public:
	// auto a = 1;        错误的: 显然不能这样
	// static auto a = 1; 错误的: 静态成员变量不能在类内初始化
	// static auto a;     错误的: 无法推导
	static int a;
};

auto Demo::a = 1; // 静态成员变量需要在类外面赋值, 此处可以自动推导.
```

参阅资料, 应该是这样:
```C++
static class Demo
{
public:
	//auto a = 1; 错误的
	//static auto a = 1; 错误的
	static const auto a = 1;
};
```

### 不能用于模版参数

**错误示范**:
```C++
vector<auto> awa = { 1,2,3,4 }; // 报错
```

### 不能用于推导为数组类型

考虑下面代码:
```C++
int arr[] = {1, 2, 3, 4, 5};
auto ptr = arr;
auto sum = arr[2];
```

ptr 的 类型是 `int*` 而不是 `int[5]`

sum 则是正常的 `int`

## 特别注意
### 变量必须在定义时初始化

谁知道你是什么类型?! ~~你以为在写py吗?~~
```C++
auto a;
```
### 定义在一个auto序列的变量必须始终推导成同一类型

例如:

```C++
auto a4 = 10, a5 = 20, a6 = 30;	   // 正确
auto b4 = 10, b5 = 20.0, b6 = 'a'; // 错误, 没有推导为同一类型

```

### auto 不是类型, 是关键字

所以下面代码是不被允许的:
```C++
sizeof(auto);
cout << typeid(auto).name() << endl; //错误
```

## 注解
### [1] 补充
使用auto关键字做类型自动推导时，依次施加以下规则:


> ##blue##
> 如果初始化表达式是引用，则去除引用语义.

```C++
int a = 10;
int &b = a;

auto c = b; // c的类型为int而非int&（去除引用）
auto &d = b;// 此时c的类型才为int&

c = 100;    // a =10;
d = 100;    // a =100;
```
---
> ##blue##
> 如果初始化表达式为const或volatile（或者两者兼有），则除去const/volatile语义.


```C++
const int a1 = 10;
auto b1 = a1;       // b1的类型为int而非const int（去除const）
const auto c1 = a1; // 此时c1的类型为const int
b1 = 100;           // 合法
c1 = 100;           // 非法
```
---
> ##blue##
> 如果auto关键字带上&号，则不去除const语意.

```C++
const int a2 = 10;
auto &b2 = a2;    // 因为auto带上&，故不去除const，b2类型为const int
b2 = 10;          // 非法
```
---
### [2] 参考

[C++新特性: auto关键字](https://www.cnblogs.com/QG-whz/p/4951177.html)
