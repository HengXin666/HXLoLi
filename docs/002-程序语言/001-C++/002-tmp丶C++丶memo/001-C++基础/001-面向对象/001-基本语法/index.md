# C++面向对象 - 基本语法
> 这个是复习使用, 你已经熟练使用C++了! (请配合[[设计模式]目錄大綱](../../../../../../001-计佬常識/002-设计模式/002-【设计模式】目錄大綱/index.md)食用)
## 1.1 类的定义 & 访问修饰符

```C++
static class ClassName {
public:		// 属性修饰符 (公共的) 公有成员在程序中类的外部是可访问的。您可以不使用任何成员函数来设置和获取公有变量的值
	void printput(string str) {
		cout << str << endl;
	}
protected:	// 受保护的 protected（受保护）成员变量或函数与私有成员十分相似，但有一点不同，protected（受保护）成员在派生类（即子类）中是可访问的。
private:	// 私有的 私有成员变量或函数在类的外部是不可访问的，甚至是不可查看的。只有类和友元函数可以访问私有成员。默认情况下，类的所有成员都是私有的。
};

void test(void) {
	// 类的定义在上面
	ClassName cn;         // 现在有了对象 cn
	cn.printput("14333"); // 使用类方法
}
```

## 1.2 常识
### 1.2.1 构造函数 & 析构函数
```C++
class AwA {
public:
	AwA() {     // 构造函数, 在对象声明时候, 会自动调用该函数
				// 该函数没有返回值, 但是参数可变
				// 可以自定义构造函数, 构造函数的名字和类的名字相同 (不自定义编译器也会自动生成并执行)
		cout << "qwq!!" << endl;
	}
				// 这样是函数重载, 如果写了有参数的, 没有写无参数的
				// 那么是无法使用无参数在声明对象的时候
	AwA(string str, int op = 1433223) {
    	        // 当然, 函数也是可以设置默认值的
		cout << "awa!!! " << str << endl;
	}

    // 在 析构函数名前面加上~符号, 那么这个函数就是析构函数
    // 即当对象被释放的时候, 会自动执行 (例如出作用域/被delete了)
    // 析构函数没有 返回值 和 参数!
    ~AwA() {
    	cout << "释放对象" << endl;
    }
};
```

### 1.2.2 this指针 & 成员 

```C++
class MyClass {
public:
	MyClass(){}
	~MyClass(){}
	void get_int(int n) {
		this->n = n; // this 指针可以解决 形参与成员变量名相同问题
	}

private:
	int n; // n 私有成员变量, 只能在类里面使用, 不能在类外面通过 对象.n = xxx; 赋值修改
};

void text_028(void) {
	/*	<< C++ this指针的本质 >> By 
	* this实际上是成员函数的一个形参，在调用成员函数时将对象的地址作为实参传递给this。
	* 不过this这个形参是隐式的，它并不出现在代码中，而是在编译阶段由编译器默默地将它添加到参数列表中。
	* 
	* this作为隐式形参，本质上是成员函数的局部变量，
	* 所以只能用在成员函数的内部，并且只有在通过对象调用成员函数时才给this赋值。
	* 
	* this是const指针，它的值是不能被修改的，一切企图修改该指针的操作，如赋值、递增、递减等都是不允许的。
	* 
	* this只能在成员函数内部使用，用在其他地方没有意义，也是非法的。
	* 只有当对象被创建后this才有意义，因此不能在static成员函数中使用。
	* */

	MyClass me;
	me.get_int(123);

	/* << this的应用场景与注意事项 >> By GPT-3.5
	* this 指针在 C++ 中有许多应用场景，以下是一些常见的使用情况和注意事项：
	*
	* 明确指代当前对象：通过 this 指针可以明确引用当前对象，避免与局部变量或参数发生命名冲突。
	*
	* 在成员函数中访问成员变量：使用 this-> 可以显式地访问当前对象的成员变量，即使成员变量与函数参数同名也不会产生歧义。
	*
	* 返回当前对象的引用：在链式编程中，可以将 this 指针作为返回值，以便连续调用当前对象的成员函数。
	*
	* 在构造函数和析构函数中使用：在构造函数和析构函数中，可以使用 this 指针对当前对象进行操作和初始化。
	*
	* 传递给其他函数：在需要将当前对象作为参数传递给其他函数时，可以使用 this 指针将当前对象传递给目标函数。
	*
	* 注意事项:
	*
	* this 指针只能在非静态成员函数中使用，因为静态成员函数不依赖于具体对象的实例。
	*
	* 在构造函数中使用 this 指针要谨慎，因为在对象尚未完全构造完成之前，使用 this 指针可能导致不确定的结果。
	*
	* 避免在成员函数中返回局部变量的指针或引用，因为该指针或引用可能会在成员函数结束后失效。
	* 如果需要返回当前对象的引用，可以使用 *this 来返回对象自身的引用。
	*
	* 在多线程环境下，要注意使用 this 指针的线程安全性，确保多个线程对同一对象的操作不会导致竞态条件。
	*
	* 总而言之，this 指针在面向对象编程中起着重要的作用，它提供了对当前对象的隐式引用。
	* 正确使用 this 指针可以增加代码的清晰度和可读性，但也需要注意它的使用限制和潜在的问题。
	* */
}
```

### 1.2.3 静态成员

> 关联一个C++17新特性: [inline 新增用法](../../../003-C++新特性/001-C++常用新特性/015-inline新增用法/index.md) 使得可以在头文件或者类内初始化静态成员变量!

```C++
class HengXin {
public:
	static string xp;		// 使用 static 关键字 定义一个静态成员

	static void loli(void) {    // 使用 static 关键字 定义一个静态函数
		cout << xp << endl;	// 静态函数只能访问静态成员变量
	}
};

string HengXin::xp = "ロリ";	// 在全局作用域中对静态变量进行改变, 需要在前面加上类型

/*	为什么函数外,即全局作用域又需要标注类型?
* 在全局作用域中声明并定义静态成员变量时，需要显式标注变量的类型，是因为全局作用域没有类的定义来指明变量的类型。
*
* 在全局作用域中，静态成员变量实际上是属于类的，但它是在类外部进行声明和定义的。
* 因此，在全局作用域中，编译器无法根据类定义来自动推断静态成员变量的类型，需要通过显式标注类型来告知编译器。
*
* 由于全局作用域中的静态成员变量不直接属于任何特定的对象，而是与类关联，并且可以在类的任何实例之间共享。
* 因此，在全局作用域中声明和定义静态成员变量时，需要以类名作为限定符，确切地指明要声明的是哪个类的静态成员变量。
* */

/*
* 静态成员函数也是该类共享的函数。静态成员函数属于类本身，而不是类的实例对象。它们可以直接通过类名调用，无需创建对象实例。
* 因此，静态成员函数不依赖于具体的对象，可以在没有对象的情况下被调用。
*
* 静态成员函数通常用于处理与类相关的操作，例如访问静态成员变量、执行静态方法等。
* 静态成员函数的特点是在调用的时候不需要创建对象实例，可以通过类名直接调用，例如 ClassName::staticFunction()。
*
* 需要注意的是，静态成员函数内部不能直接访问非静态成员变量和非静态成员函数，因为它们与具体的对象相关联。
* 但是，静态成员函数可以访问静态成员变量和其他静态成员函数，因为它们都属于类本身，在内存中只有一份拷贝。
* 
* 总结来说，静态成员函数是属于类的函数，不依赖于具体的对象实例，可以通过类名直接调用，并且可以访问静态成员变量和其他静态成员函数。
* */

void text_026() {
	// 静态成员变量是全部该类的对象共享的
	HengXin::xp = "ロリコン"; // 可以通过类名进行访问
	HengXin::loli();
}
```

### 1.2.4 常成员函数

语法:
```C++
返回类型 成员函数名(参数表) const;

int function(int x) const {
	return x * x;
}

int function(int x) { // `const`关键字可以用于对重载函数的区分
	return x + x;
}
```
- 常成员函数的主要特点
	- 不能更新类的成员变量
	- 不能调用该类中没有用`const`修饰的成员函数，即只能调用常成员函数
	- 可以被类中其它的成员函数调用
	- 常对象只能调用常成员函数，而不能调用其他的成员函数。(常对象也包括常指针和常引用)
	- `const`关键字可以用于对重载函数的区分
    - `const`是函数类型的一部分，在实现部分也要带`const`
	- 常函数的`this`指针是`const Class*`型

注意:
- 普通成员函数才有常函数
	- 构造成员函数的用途是对对象初始化，成员函数主要是用来被对象调用的，如果构造函数被设置成`const`，就不能更改成员变量，失去了其作为构造函数的意义。(析构函数同理)
    - 全局成员函数和静态成员函数`static`其函数体内部没有`this`指针，所以也不能是常成员函数

`const`不能保证其内部不会被修改, 只是方便别人知道它没有对里面内容修改.

```C++
void text_027(void) {
	HengXin hx;
	hx.getStr(); // 输入一个 string
	hx.putStr(); // 打印这个 string

	string* s = (string*)hx.getName(); // 获取但是是常成员函数
	*s = "awa"; // 修改
	cout << *s << endl; // 打印
	hx.putStr(); // 类里面的也被修改(因为是指针)
}

const string* HengXin::getName() const {
	return &this->name; // 甚至去掉这里的 const 只强转为 (string *) 也行 (当然返回值类型也要去掉)
	// (此处的 this 是 const this, 所以 this->name 是 const string, &this->name 是 const string *)
}
```

再注: `返回值 函数名() const`的`const`**不是指返回值为**`const`!

### 1.2.5 友元函数

```C++
class MyClass {
public:
	MyClass() {}
	~MyClass() {}
	friend static void _text_029_n(MyClass m);	// 友元函数 关键字: friend
	// 在类里面声明, 在类外面实现(在类里面实现也不是不行), 可以访问 类的全部私有成员变量 和 受保护的成员变量
	
	friend void awa(MyClass m) { // 即便不加 static, 它也不是成员函数!
		cout << m.n << endl;
		// cout << this.n << endl; // 不是成员函数不能this了
	}
private:
	int n;
};

static void _text_029_n(MyClass m) { // 注意形参, 传参
									 // 使用和普通函数没有区别
	m.n = 1;
}

void text_029(void) {
	MyClass me;
	// me.n = 1;     // 无法访问
	_text_029_n(me); // 没有报错 <-- 直接使用, 不是 ".方法"
	awa(me); // 无法通过 me.awa() 使用! 因为不是成员
}
```

### 1.2.6 友元类

```C++
static class Qwq; // 声明, 说明下文会有定义, 不要着急报错
static class Awa
{
public:
	Awa() {
		// my_var = 1; // 类内部函数可以访问私有成员(废话)
	}
	friend class Qwq; // 声明 Awa 是 Qwq的友元类 即, 声明 友元类 Qwq

private:
	int my_var;
};

static class Qwq
{
public:
	// friend class Awa; // 可以互为友元类
	void awa(Awa& A) {
		cout << A.my_var << endl;
	}

	void get_var_to_awa(Awa& A, int x) { // 不写引用是拷贝对象哦
		A.my_var = x; // 友元类也可以访问 该类的私有成员
	}
};

void text_030(void) {
	Qwq q;
	Awa a;
	q.get_var_to_awa(a, 666);
	q.awa(a);
}
```

### 1.2.7 运算符重载

```C++
class MyClass {
public:
	MyClass (int v) {
		this->val = v;
	}

    // A + B 
	int operator+(MyClass& m_1)	{ // A 是自己, B 是传参, 可以认为
		return this->val + m_1.val;
	}

	// 更多运算符重载示例, 可以被函数重载, 一定要符合参数数量等于操作数数量
    // 三目运算符没有运算符重载
    void operator()(int x) { }
    
    void operator()(int x, int y) { } // 重载()被称为仿函数 (匿名函数也是这个原理)
    
    int operator!() { }
    
    void operator/(int x) { }
private:
	int val;
};

void text_031(void) {
	MyClass my1 = 1;
	MyClass my2(2);

	cout << my1 + my2 << endl;
}
```
在 C++ 中，对于大多数运算符重载，**返回值类型**通常是有要求的，它们的返回值应该与对应的原生运算符的行为保持一致，或者至少应该有一定的语义上的相关性。

例如:

- 对于算术运算符（如 `+`、`-`、`*`、`/` 等），通常返回的类型应该是能够正确表示这种运算结果的类型。
- 对于比较运算符（如 `==`、`!=`、`<`、`>` 等），通常返回的类型应该是布尔类型 `bool`。
- 对于赋值运算符（如 `=`、`+=`、`-=`、`*=` 等），通常返回的类型应该是当前对象的引用，以支持连续赋值操作。

但是也有一些特例，如：

- `operator!`（逻辑非）不一定要返回布尔类型 `bool`，但它应该返回一个表达否定的值。
- `operator[]`（下标运算符）返回值的类型可以根据需要灵活选择，但通常应该返回一个引用，以允许对元素进行读写操作。

总的来说，虽然运算符重载的返回值在一些情况下有一定的要求，但它们的确有一定的灵活性，允许根据需要进行自定义。

## 1.3 继承
### 1.3.1 构造 & 析构 の 调用

- 当 struct 继承另一个 struct 或 class 时，默认访问权限是`public`。
- 当 class 继承另一个 struct 或 class 时，默认访问权限是`private`。

```C++
class Person {};
class Student : public Person {};

// 当一个派生类被实例化为对象时, 会先调用基类的构造函数, 再调用派生类的构造函数
// 当其结束时候, 会先调用派生类的析构函数, 再调用基类的析构函数, 类似于 递归的<归>
// 值得注意的是: 调用的基类的函数是默认的构造函数, 如果没有则会报错
// 当然通过其他方法是可以在没有默认构造函数的情况下, 基类和派生类都可以正常调用带参数的构造函数
```

示例:
```C++
class _Zoo {
public:
    _Zoo(int x) {
        cout << "{ // Zoo" << endl;
    }

    ~_Zoo() {
        cout << "} // Kill Zoo" << endl;
    }
};

class _Cat : public _Zoo {
public:
    _Cat(string name) : _Zoo(1) { // 委托基类的构造函数
        cout << "\t{ // Cat" << endl;
        cout << "\t\t\t[init]: 君の名は " << name << " です~" << endl;
    }

    ~_Cat() {
        cout << "\t} // Kill Cat" << endl;
    }

    void maimai() {
        cout << "\t\t\t[NbNeKo]: giao ~ !" << endl;
    }
};

class _NbCat : public _Cat {
public:
    _NbCat(string name) : _Cat(name) { // 需要通过 `:` 这样调用
        cout << "\t\t{ // NbCat" << endl;
        cout << "\t\t\t[init]: 君の名は " << name << " です~" << endl;
    }

    ~_NbCat() {
        cout << "\t\t} // Kill NbCat" << endl;
    }
};

void _text_034(void) {
	// 这个就是不用默认的构造函数, 也不写 By
	_NbCat awa("超级喵~");
	awa.maimai();
}
```

输出:

```C++
{ // Zoo
        { // Cat
                        [init]: 君の名は 超级喵~ です~
                { // NbCat
                        [init]: 君の名は 超级喵~ です~
                        [NbNeKo]: giao ~ !
                } // Kill NbCat
        } // Kill Cat
} // Kill Zoo
```

### 1.3.2 多继承 & 访问修饰符
语法示例:
```C++
class FXXK : public CNM, protected WDF, private WDNMD {};
```

**继承の特徴**:

有public, protected, private三种继承方式，它们相应地改变了基类成员的访问属性。

1. **public** 继承: 基类`public`成员，`protected`成员，`private`成员的访问属性在派生类中分别变成: `public`,`protected`,`private`

2. **protected** 继承: 基类`public`成员，`protected`成员，`private`成员的访问属性在派生类中分别变成: `protected`,`protected`,`private`

3. **private** 继承: 基类`public`成员，`protected`成员，`private`成员的访问属性在派生类中分别变成: `private`,`private`,`private`

但无论哪种继承方式，下面两点都没有改变:

1. **private** 成员只能被本类成员（类内）和友元访问，不能被派生类访问；

2. **protected** 成员可以被派生类访问。

示例:

```C++
class Zoo {
public:
    Zoo() { cout << "The Zoo Good" << endl; }

    void printInfo(void) { }
};

class Cat {
public:
    Cat() { cout << "Cat Cat kawaii ~" << endl; }

    void printInfo(void) { }
};

// 这样就是[多继承], 可以是继承到不同的位置<属性修饰符>
class CatInZoo : public Zoo, public Cat { // 构造函数/析构函数的调用顺序是多继承处声明决定的, 和下面这个的顺序无关
// 构造顺序是: Zoo --> Cat (--> CatInZoo) 从左到右 
public:
    CatInZoo(void) : Cat(), Zoo(){} // 我就是上面那个的下面那个...
                                    // Cat(), Zoo() 的顺序不会对构造/析构的顺序产生影响
    void fun(void) {
        // <多继承名称冲突解决>
        // 这样调用 基类同名函数不会冲突
        Zoo::printInfo();
        Cat::printInfo();
    }
};

void text_035(void) {
	CatInZoo awa;
	awa.fun(); // 不报错
}
```

### 1.3.3 菱形继承

```C++
namespace {
	class A {
	public:
		int m_a;
	};

	class B : public A {
	public:
		int m_b;
	};

	class C : public A {
	public:
		int m_c;
	};

	class D : public B, public C {
	public:
		void getA(int a) {
			// this->m_a = a;	// 不明确
			// this->D::m_a = a;// 不明确

			// 可行
			this->A::m_a = a; // 你即使这样, 也无法知道 B 的 m_a 和 C 的 m_a 哪个是 A 的 m_a, 亦或者根本就没有 B / C 的 m_a

			// 可以
			this->B::m_a = a + 1;
			this->C::m_a = a + 2;

			// 这样就产生了三分成员变量m_a, 内存不堪重负... 除非某些场景有需求吧
		}

		void putA(int xz) {
			switch (xz) {
			case 1:
				cout << A::m_a << endl; // 124
				break;
			case 2:
				cout << B::m_a << endl; // 124
				break;
			case 3:
				cout << C::m_a << endl; // 125
				break;
			default:
				break; // 暂时不知道为什么会是上面的值(我电脑的输出) (姑且认为是未定义行为吧, 毕竟只有两份m_a(实际上), 你访问的A::m_a肯定是 B 或者 C 的基础上的, 只不过是誰的是不知道) (反正也不可能会真的这样写)
			}
		}
	public:
		int m_d;
	};
}

static void _text_036(void);
void text_036(void) {
	/*    菱形继承效果图:
    *            A        基类
    *           / \
    *          B   C      继承基类A的派生类B, C
    *           \ /
    *            D        继承基类B, C的派生类D
    * 
    *    危害: 如果 A中的成员变量被B, C继承并且继承到D, 那么若 调用A的变量, 就不知道是 B的基类A的 还是 C的基类A的 (变量冗余)
    * 
    *    解决方法:    1. 设计严密的继承属性 后续又有需求怎么办?我就是要用啊       [Pass]
    *                2. 如上, 显式要求使用 A::变量, 可是还是会有一份的变量冗余  [Warning]
    *                3. 改为使用虚继承 见 _text_036();                     [Info]
    *
    *    值得注意的是:
    *        "使用虚继承会增加运行时的开销，需要额外的虚指针和虚表来管理虚基类。"
    *
    * 因此，在设计类的继承结构时，只有在存在菱形继承的情况下或确实需要使用虚继承解决[二义性问题]时，才推荐使用虚继承。
    * 对于其他情况，普通的继承即可满足需求。
    *    <<值得注意>> By GPT-3.5 
    * */
	D d;
	d.getA(123);
	d.putA(1);
	d.putA(2);
	d.putA(3);
	_text_036();
}


namespace {
	class X_A {
	public:
		int m_a;
	};

	class X_B : virtual public X_A {
	public:
		int m_b;
	};

	class X_C : virtual public X_A {
	public:
		int m_c;
	};

	class X_D : virtual public X_B, virtual public X_C {
	public:
		void getA(int a) {
			this->m_a = a;
		}

		void putA(void) {
			cout << m_a << endl;
		}

	public:
		int m_d;
	};
}

static void _text_036(void) {
	X_D xd;
	xd.getA(114514);
	xd.putA();
}
```

### 1.3.4 final 禁止被继承

顾名思义, `final`关键字声明的类不能被继承~~, 是太监类~~.

```C++
class Loli final {};

class QwQ : Loli {}; // 报错
```

## 1.4 多态
### 1.4.1 多态与构成条件

```C++
namespace {
	class A {
	public:
		A(string str) : str(str) {}

		virtual void putInfo(void) {
			cout << "[Info]: " << str << endl;
		}

	protected:
		string str;
	};

	class B : public A {
	public:
		B(string name) : A(name) {}

		virtual void putInfo(void) {
			cout << "[Info]: " << str << endl;
		}
	};
}

void text_037(void) {
	/*
	*	构成多态的条件:
	*	1. 必须存在继承关系;
	*	2. 继承关系中必须有同名的[虚函数]，并且它们是覆盖关系（函数原型相同(参数的类型、顺序相同)）// 加 virtual 关键字就是虚函数
	*	3. 存在基类的指针，通过该指针调用虚函数。
	* */

	// 指针实现多态
	A* p = new A("大哥");
	p->putInfo();
	delete p;

	p = new B("小老弟");
	p->putInfo();
	delete p;
	// 这里的内存释放并不是正确的似乎,,, 所以只要看多态的代码就好 (具体解决请见 1.4.2)

	// 引用实现多态
	A awa("吖");
	B qwq("砸");

	// 类型是基类类型哦!
	A& p1 = awa;
	A& p2 = qwq;

	p1.putInfo();
	p2.putInfo();
}
```

### 1.4.2 虚函数与虚析构

```C++
namespace {
	class A {
	public:
		/* ~A() {
			cout << "call ~A" << endl;
		} */

		// 请尝试注释下面 虚析构函数 然后运行, 看看有什么效果
		virtual ~A() {
			cout << "call virtual ~A" << endl;
		}
	};

	class B : public A {
	public:
		~B() { // 如果B类还有子类, 那么也应该为虚析构
			cout << "call ~B" << endl;
		}
	};
}

void text_038(void) {
	// 虚函数主要是为了实现多态
	// 加 virtual 关键字就是虚函数

	// 虚析构是为了在 new 子类 在 delete 子类时候, 会调用基类的 析构函数
	// 如下:
	A* p = new B;
	delete p;

	// 可以把A的内存空间看成是苹果, 然后B是苹果籽, 如果delete p
	// 然后如果是在没有这个虚析构函数的情况下, 只是会释放苹果籽,但是苹果还在!
}
```

### 1.4.3 虚函数表与动态绑定
**虚函数** 是在基类中使用关键字`virtual`声明的函数。在派生类中重新定义基类中定义的虚函数时，会告诉编译器不要**静态链接**到该函数。

我们想要的是在程序中任意点可以根据所调用的对象类型来选择调用的函数，这种操作被称为**动态链接**，或**后期绑定**。
```C++
namespace {
	class Animal {
	public:
		virtual void fun1(void) { // 父类独有方法
			cout << "zoo fun1" << endl;
		}

		virtual void fun2(void) {
			cout << "zoo fun2" << endl;
		}
	};

	class Cat : public Animal
	{
	public:
		virtual void fun3(void) { // 子类独有的方法
			cout << "cat fun3" << endl;
		}

		virtual void fun2(void) override { // override 重写, 编译器会检查看看是否存在父类有这个函数(这样警告就可以看到)
			cout << "cat fun2" << endl;    // 不写 override != 没有重写
		}
	};

	// 下面是非虚函数的对比 无法实现

	class A {
	public:
		void awa(void) {
			cout << "A awa" << endl;
		}

		void qwq(void) {
			cout << "A qwq" << endl;
		}
	};

	class B : public A {
	public:
		void awa(void) {
			cout << "B awa" << endl;
		}
	};
}

void text_039(void) {
	// 多态, 要满足动态条件哦
	Animal* p = new Cat();
	p->fun1();				// 访问父类独有可以
	p->fun2();				// 可以访问子类重写的方法 多态

	((Animal*)p)->fun2();
	((Cat*)p)->fun3();		// 类型转换可以访问子类新增方法 (如果是用多态, 那么一般不设计这样用)

	// 对比: 无, 原因: 无法满足多态的条件, 不能使用指针来访问子类, 并且函数覆盖等
	A* a = new A;
	a->awa();

	A* b = new B;
	b->awa(); // 访问的是 A 的方法

	((B*)b)->awa(); // 类型转化才访问的了B
}
```

输出:

```C++
zoo fun1
cat fun2
cat fun2
cat fun3
A awa
A awa
B awa
```

> 对比: Java 的[多态](../../../../../002-Java/002-Java基础/005-面向对象丶/010-多态/index.md)是: 默认情况下，所有的方法都是虚函数，因此在 Java 中实现多态不需要特别的标记。此外，在 Java 中，引用变量始终是多态的，因此无论引用的是基类还是派生类的对象，都可以调用相同的方法，无需进行强制类型转换。这与 C++ 中需要显式地声明虚函数以及使用基类指针或引用访问对象的情况是不同的。

### 1.4.4 抽象类(接口) & 纯虚函数

給虚函数`= 0`, 然后不实现(没有函数体), 那么它就是一个 **纯虚函数**
```C++
virtual 返回值 函数名(参数列表) = 0; // 纯虚函数
```

而有一个 **纯虚函数** 的类, 就是 **抽象类(接口)** (因为你一定要继承它, 不然就无法实现虚函数!)

```C++
namespace {
	class ZhiZuoYinPin { // [制作饮品] 抽象基类 (可以将抽象基类理解为架构/函数声明, 实现是在派生类里面的)
	public:
		// 这个是纯虚函数, 只是提供一个模版, 需要在派生类中实现
		virtual void yeiTi(void) = 0;	// 液体
		virtual void ZiZuo(void) = 0;	// 制作
		virtual void ZhuanYi(void) = 0;	// 转移
		virtual void ZhuoLiao(void) = 0;// 佐料

		void MakeDrink(void) { // 规定制作过程 取液体-->制作-->装好-->放药
			yeiTi();
			ZiZuo();
			ZhuanYi();
			ZhuoLiao();
		}
	};

	class Coffee : public ZhiZuoYinPin {
	public:
		virtual void yeiTi(void) { cout << "从猫猫河里面取水, 并且进行二次蒸馏" << endl; }

		virtual void ZiZuo(void) { cout << "2Fe + Co ==加热== Java" << endl; }

		virtual void ZhuanYi(void) { cout << "抽咖啡机装入粉色麻袋" << endl; }

		virtual void ZhuoLiao(void) { cout << "放入老八餐厅顺手拿的佐料" << endl; }
	};

	class Juice : public ZhiZuoYinPin {
	public:
		virtual void yeiTi(void) { cout << "将苹果手机放入榨汁机" << endl; }

		virtual void ZiZuo(void) { cout << "Apple ==榨汁机== App + LE" << endl; }

		virtual void ZhuanYi(void) { cout << "放进杯子" << endl; }

		virtual void ZhuoLiao(void) { cout << "放入老六餐厅顺手拿的佐料" << endl; }
	};
}

static void _text_040_eat(ZhiZuoYinPin *Z) {
	Z->MakeDrink();
	delete Z;
}

void text_040(void) {	
	// 注意虚函数名不能写错哦!
	_text_040_eat(new Coffee());
	cout << endl;
	_text_040_eat(new Juice());
}
```

## 1.5 嵌套类 & 局部类

```C++
namespace {
	class WaiBuClass {
	public:
		class ClassClass { // 在类内部定义的类叫做嵌套类(内部类)
						   // 一般嵌套类是为了给外部类自己使用的, 不提供给外界
		public:
			void func(void);
		public:
			int b; // 内部类是不能访问外部类的变量的
		};

		void func(void);
	public:
		int a;
	};

	void WaiBuClass::ClassClass::func(void) { // 可以在外部实现
		cout << "这个就是嵌套类" << endl;
		// b = a; // 报错, 不能访问
		b = 1;
	}

	void WaiBuClass::func(void) {
		cout << "这个是外部类" << endl;
		a = 2;
	}
}

static void _text_043_class(void) {
	class JuBuClass { // 局部类, 定义在函数里面, 作用域在函数里面, 而不是整个文件(全局作用域)
	public:
		void func(void) { cout << "这个是局部类" << endl; }

		static void funawa(void) { // 可以定义静态成员函数, 但是要注意作用域, 不能访问类的任何变量
								   // 因为不能访问非静态成员变量, 能访问静态成员变量, 但静态成员变量又不能定义
		// [!] 但是, 它可以实现像Python一样在函数里面定义子函数, 只需要通过 "类名::函数名(参数列表)" 的方式访问即可 ^[1]
			cout << "达咩" << endl;
		}

		// static int a; // 不能声明静态成员变量 <报错>
	};

	JuBuClass awa;
	awa.func();
	JuBuClass::funawa();
}

//void JuBuClass::func(void)	// 局部类不能在类外实现函数 <报错>
//{
//	;
//}

static auto _text_043_class_() {
	class CCClass {
	public:
		void awa() { cout << "The CCClass!" << endl; }
	};

	return CCClass(); // 可以作为返回值!!!
}

void text_043(void) {
	WaiBuClass w;
	w.func();
	cout << endl;
	// 外界如果要访问内部类, 只能指名道姓:
	WaiBuClass::ClassClass cc;
	cc.func();
	cout << endl;
	// 局部类
	_text_043_class();

	auto x = _text_043_class_();
	x.awa();
}
```
- 对于上面的`[1]`, 类似python的def内的def, 可以尝试 [Lambda表达式](../../../003-C++新特性/001-C++常用新特性/006-Lambda表达式/index.md) + [function](../../../003-C++新特性/001-C++常用新特性/010-function/index.md) (可能是一个更舒服的写法)

- 局部类更灵活, 可以作为返回值, 不同于Java([嵌套类](../../../../../002-Java/002-Java基础/009-嵌套类及Lambda表达式/001-嵌套类/index.md))

- 此处有用到[auto自动类型推导](../../../003-C++新特性/001-C++常用新特性/001-auto自动类型推导/index.md)的知识, 不过请记住你已经熟练使用C++了 %%%