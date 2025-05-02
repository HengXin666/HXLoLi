# 右值引用&&
## 说明
C++98 中提出了引用的概念，引用即别名，引用变量与其引用实体公共同一块内存空间，而引用的底层是通过指针来实现的<sup>[1]</sup>，因此使用引用，可以提高程序的可读性。

> 为了提高程序运行效率，C++11中引入了**右值引用**<sup>[2]</sup>，右值引用也是别名，但其只能对右值<sup>[3]</sup>引用.

作用:

> ##purple##
> 🟣 **通过将一个右值引用参数传递给函数，可以避免对该右值进行`复制`或`移动`操作，从而提高程序的效率。**(传参不会对那个值进行复制, 比如 传入类就不会多次执行构造函数与析构函数)
---
对比说明: 

1.
```C++
static int num = 10;

int& resnum(void)
{
	return num;
}

int main(void)
{
	resnum() = 111;
	cout << num << endl; // 111
	return 0;
}
```
2.
```C++
static int num = 10;

int resnum(void)
{
	return num;
}

int main(void)
{
	resnum() = 111;      // 报错
	cout << num << endl; // 10
	return 0;
}
```
3.
```C++
static int num = 10;

// 右值引用
int&& resnum(void)
{
	return num;
}

int main(void)
{
	resnum() = 111;      // 报错: 返回值不是可修改的左值
	cout << num << endl; // 10
	return 0;
}
```

对于 1. 的代码, 实际上可以看做是:

```C++
static int num = 10;

int* resnum(void)
{
	return &num;
}

int main(void)
{
	*resnum() = 111;
	cout << num << endl; // 111
	return 0;
}
```

## 应用场景示例
更多请查看 `参考文献[2]`

考虑以下代码, 尝试看透在传参的时候会执行多少次类的构造与析构...

```C++
class A
{
public:
	A(size_t N):m_p(new char[N])
	{

    }
 
	A(const A & a)
	{
		if (this != &a)
		{
			delete[]m_p;
			m_p = new char[strlen(m_p) + 1];
			memcpy(m_p, a.m_p, strlen(m_p) + 1);
		}
	}
 
	~A()
	{
		delete []m_p;
	}
 
private:
	char *m_p = nullptr;
};
 
A createA(size_t N)
{
	return A(100);
}
 
void func(A a)
{
	//
}
 
int main()
{
	func(createA(100));
 
	system("pause");
	return 0;
}
```

这里会导致大量得调用A得构造函数，不考虑编译优化，原本执行如下：

1. createA(100)，执行A(100)调用A(size_t)`构造`函数一次；
2. 退出createA，临时构造得A(100)，释放调用`析构`函数一次；
3. 赋给返回值会调用一次`拷贝构造函数`一次；
4. 返回值传入func中形参会调用`拷贝构造函数`一次；
5. func运行完成后形参释放，调用A`析构`函数一次；
6. 返回值使用完成释放，调用A`析构`函数一次；

即 生成, 返回, 传参 各一次.

> 从上面可以看出有大量得构造、析构调用 ，但是我们做的工作无非就是临时构造一个A(100)给func使用而已。那么可否将临时A(100)始终一份给到func使用呢？答案就是**右值引用**。


```C++
class A
{
public:
	A(size_t N):m_p(new char[N])
	{
      
	}
 
	~A()
	{
		delete []m_p;
	}
 
private:
	char *m_p = nullptr;
};
 
A&& createA(size_t N)
{
	return (A&&)A(100);
}
 
void func(A&& a)
{
	//
}
 
int main()
{
	func(createA(100));
 
	system("pause");
	return 0;
}
```

我们将临时A(100)强制转换为了右值引用，同时func形参也是右值引用，也就是将临时对象延长到了func中，中间避免了其他构造和析构调用，提高了效率。

*注意到我们将A得拷贝构造函数去掉了，因为已经用不到。如果原版写法，去掉拷贝构造函数会崩溃，因为会自动调用默认拷贝构造函数，是浅拷贝，中间临时对象会提前删除公共内存，后面对象再次释放是就会重复删除内存导致崩溃。*

# 移动语义
**转移语义**可以`将资源(堆、系统对象等)从一个对象转移到另一个对象`，这样可以<span style="color:red">减少不必要的临时对象的创建、拷贝及销毁</span>。移动语义与拷贝语义是相对的，可以类比文件的剪切和拷贝。在现有的C++机制中，自定义的类要实现转移语义，需要定义移动构造函数，还可以定义转移赋值操作符

以string类的移动构造函数为例:

```C++
MyString::MyString(MyString&& str)
{
    cout << "move ctor source from " << str.data << endl;
    len = str.len;
    data = str.data;
    str.len = 0;
    str.data = NULL;
}

// 使用
MyString str1("abc");
MyString str2(std::move(str1)); // 注意要类型转换, 变成右值引用
```

**和拷贝构造函数类似，有几点需要注意：**

- 参数(右值)的符号必须是&&

- 参数(右值)不可以是常量，因为我们需要修改右值

- 参数(右值)的资源链接和标记必须修改，否则，右值的析构函数就会释放资源。转移到新对象的资源也就无效了<sup>(1)</sup>

**标准库函数std::move 可以将左值变成一个右值**

编译器只对右值引用才能调用移动构造函数，那么如果已知一个命名对象`不再被使用`<sup>[4]</sup>，此时仍然想调用它的移动构造函数，也就是把一个左值引用当做右值引用来使用，该怎么做呢？用 `std::move`，这个函数以非常简单的方式将左值引用转换为右值引用。

---
注解(1):
> 在 C++ 中，右值引用是一种新的引用类型，它可以绑定到 $临时对象、函数返回值$ 等不能被修改的值。通过右值引用，可以**将一个右值的所有权转移给另一个对象，这个过程就是所谓的“移动”**。
>
> 在移动对象时，如果不修改右值的资源链接和标记，那么右值的析构函数就会释放资源。这意味着，转移到新对象的资源也就无效了，因为它们已经被释放了。这样就会导致程序出现未定义行为，可能会导致程序崩溃或者发生其他不可预测的错误。
>
> 因此，在实现移动构造函数和移动赋值运算符时，需要确保修改右值的资源链接和标记，以便正确地移动资源，并避免资源泄漏和程序错误。这通常需要使用 std::move() 来将资源所有权转移到目标对象，并将源对象的资源标记为无效状态。

即	

当我们使用移动构造函数或移动赋值运算符时，需要将资源的所有权从一个对象转移到另一个对象，同时使源对象的资源无效化。这是为了避免资源的重复释放或使用无效的资源。

下面是一个示例代码，展示了如何正确实现移动构造函数和移动赋值运算符：


```C++ By GPT-3.5
#include <iostream>
#include <string>

class MyString
{
public:
    // 移动构造函数
    MyString(MyString&& str) noexcept
        : data(str.data), len(str.len)
    {
        std::cout << "Move constructor: moving resource from " << str.data << std::endl;
        str.data = nullptr;
        str.len = 0;
    }
    
    // 移动赋值运算符
    MyString& operator=(MyString&& str) noexcept
    {
        std::cout << "Move assignment operator: moving resource from " << str.data << std::endl;
        if (this != &str)
        {
            // 释放当前对象的资源
            delete[] data;
            
            // 移动资源到当前对象
            data = str.data;
            len = str.len;
            
            // 使源对象的资源无效化
            str.data = nullptr;
            str.len = 0;
        }
        return *this;
    }
    
    // 析构函数
    ~MyString()
    {
        std::cout << "Destructor: freeing resource " << data << std::endl;
        delete[] data;
    }
    
private:
    char* data;
    int len;
};

int main(void)
{
    MyString str1("Hello");
    MyString str2(std::move(str1)); // 使用移动构造函数
    
    MyString str3("World");
    str2 = std::move(str3); // 使用移动赋值运算符
    
    return 0;
}
```
在上述代码中，移动构造函数和移动赋值运算符都使用了右值引用（&&）来接受参数，并在内部进行资源的转移操作。这样可以确保资源的正确移动，并使源对象的资源无效化。

注意，在移动赋值运算符中，我们还进行了自我赋值的检查（this != &str），以避免在释放当前对象资源之前将其赋值给自身。

通过在控制台输出语句中添加适当的消息，你可以观察到资源的移动和释放顺序，以及源对象资源的无效化。

**如果一个类被使用std::move() 来类型强制转化当做参数传递, 那么原来的这个类还可以继续使用吗?**<sup>[4]</sup>

---
## 参考文献
### [1]

[C++中“引用”的底层实现原理详解](https://blog.csdn.net/lws123253/article/details/80353197)

### [2]

超详细的解释:

[右值引用及其作用](https://blog.csdn.net/caojianfa969/article/details/118927852)

### [3]

[什么是左值（lvalue）和右值（rvalue）？](https://fishc.com.cn/thread-69833-1-1.html)

### [4]

当你使用 std::move() 将一个对象作为参数传递时，它会将对象的所有权转移到函数内部。这意味着在函数内部，你可以修改、销毁或重新分配该对象的资源。

在函数调用后，`原始对象`的状态变为**有效但未定义**的状态。这意味着*你不能再对原始对象进行任何操作，包括读取和修改对象的成员变量、调用对象的方法等*。

然而，`原始对象`的生命周期仍**未结束**。你仍然可以对原始对象使用**其他不涉及资源的操作**，例如复制该对象、查询其状态等。

下面是一个示例代码，演示了使用 `std::move()` 后原始对象的使用情况：


```C++ By GPT-3.5 (Heng_Xin 改)
#include <iostream>
#include <string>

class MyClass {
public:
	MyClass() : data(0) {
		std::cout << "Default constructor" << std::endl;
	}

	MyClass(int value) : data(value) {
		std::cout << "Value constructor: " << data << std::endl;
	}

	~MyClass() {
		std::cout << "Destructor: " << data << std::endl;
	}

	int getData() const {
		return data;
	}

private:
	int data;
};

void modifyObject(MyClass&& obj) {
	// 修改对象的资源
	obj = MyClass(100);

	// 可以继续访问对象的其他成员
	std::cout << "Modified object data: " << obj.getData() << std::endl;
}

int main() {
	MyClass obj(42);
	std::cout << "Original object data: " << obj.getData() << std::endl;

    modifyObject(std::move(obj));

	// 无法再使用原始对象，因为其资源已经被移动
	// std::cout << "Original object data: " << obj.getData() << std::endl;
	// 使用会产生未定义行为

	return 0;
}
```
在上述代码中，我们将 obj 对象作为参数传递给 modifyObject() 函数，并使用 std::move() 将其转移到函数内部。在函数内部，我们修改了对象的资源，并打印了修改后的值。然而，在函数调用后，我们无法再使用原始对象 obj，因为它的资源已经被移动。

注意，即使原始对象不再可用，但它的析构函数仍会在其作用域结束时被调用，以正确释放资源。这可以在控制台输出语句中观察到。

---
**追问**

只要是右值引用, 我就不能使用它了, 因为它被移动了, 如果传参/强制类型转化的话?


```GPT-3.5
是的，如果一个对象被转换为右值引用并传递给函数，那么在函数内部它的资源所有权就被移动了。
因此，在函数调用之后，该对象将不再拥有有效的资源，尝试使用它可能会导致未定义行为。

这意味着，如果你需要在函数调用之后继续使用原始对象，你应该使用左值引用来传递参数，或者在函数内部进行必要的拷贝或移动操作，以保留原始对象的有效性。

当你使用强制类型转换时，也需要注意对象的资源所有权。
如果你将一个左值转换为右值引用，或者将一个右值转换为左值引用，都可能会影响对象的资源所有权，并导致未定义行为。
因此，在进行强制类型转换时，你需要谨慎考虑对象的资源所有权。[5]
```

如果是 std::move(obj); 独立成一句, 那么是不是毫无意义? 因为它本身也是一个右值了,?

```GPT-3.5
是的，如果你将std::move(obj)独立成一句，它实际上是没有意义的。

std::move(obj)本身返回的就是一个右值引用，表示对象可以被移动。
当你使用std::move(obj)来转换一个对象时，你的意图是表明你希望将对象的资源所有权转移到其他地方（比如函数调用）。
但是，如果你将std::move(obj)独立成一句，它并不会对对象产生任何影响，因为没有接收它的目标。

在正确的上下文中使用std::move(obj)是有意义的，例如将其作为参数传递给接受右值引用的函数，或者在使用移动语义的容器操作中。
但是，独立使用它时，它不会产生任何实际效果。
```
### [5]
资源所有权 C++这个展开说明/介绍一下吗?


```C++ text By GPT-3.5
资源所有权是指在程序中管理和控制对象的资源分配和释放的责任。
在C++中，通过使用堆上分配的内存、文件句柄、网络连接等资源，我们可以创建各种对象并进行操作。
  这些资源需要被适当地分配和释放，以避免资源泄漏和悬空指针等问题。

传统的C++编程中，通常使用了手动管理资源的方式，也就是通过调用new运算符来分配内存，然后使用delete运算符来释放内存。
这种方式需要程序员手动追踪资源的使用和释放，并且容易出现错误，例如忘记释放资源或者多次释放同一个资源。

为了简化资源管理并提供更好的安全性和便利性，C++11引入了移动语义和智能指针等特性。
移动语义允许将资源的所有权从一个对象转移到另一个对象，而不是进行昂贵的深拷贝操作。这样可以避免不必要的资源复制，提高效率。
智能指针（如std::shared_ptr和std::unique_ptr）则提供了自动化的资源管理，能够自动释放资源，避免了手动调用delete的问题。

对于移动语义，可以使用右值引用（&&）来标识一个对象可以被移动。
通过使用std::move()函数，可以将一个对象转换为右值引用，从而表示该对象的资源可以被移动。
移动构造函数和移动赋值运算符可以在对象之间快速有效地传递资源的所有权。

智能指针则通过使用RAII（资源获取即初始化）技术来管理资源。
它们包装了原始指针，并提供了自动释放资源的机制。
std::shared_ptr使用引用计数来跟踪资源的使用情况，而std::unique_ptr则通过独占资源的方式来管理。

通过合理地使用移动语义和智能指针，可以更好地管理C++程序中的资源，避免资源泄漏和内存错误，并提高代码的可靠性和效率。
```