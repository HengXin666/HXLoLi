# 模块
## 概述
### 前言
从 C 语言中，C++ 继承了 #include 机制，依赖从头文件使用文本形式包含 C++ 源代码，这些头文件中包含了接口的文本定义。一个流行的头文件可以在大型程序的各个单独编译的部分中被 #include 数百次。基本问题是：

- (1) **不够卫生**：一个头文件中的代码可能会影响同一翻译单元中包含的另一个 #include 中的代码的含义，因此 #include 并非顺序无关。宏是这里的一个主要问题，尽管不是唯一的问题。

- (2) **分离编译的不一致性**：两个翻译单元中同一实体的声明可能不一致，但并非所有此类错误都被编译器或链接器捕获。

- (3) **编译次数过多**：从源代码文本编译接口比较慢。从源代码文本反复地编译同一份接口非常慢。

所以，在 C++ 程序中改进模块化是一个迫切的需求

例如:
```C++
#include <iostream>

int main(void) {
	std::cout << "Hello C++" << std::endl;
}
```

这段标准代码有 $70$ 个左右的字符，但是在 #include 之后，它会产生 $419909$ 个字符需要编译器来消化。尽管现代 C++ 编译器已有傲人的处理速度，但模块化问题已经迫在眉睫

模块化是什么意思？

> 顺序独立性：`import X; import Y;`
> 
> 应该与 `importY; import X;` 相同。
> 
> 换句话说，任何东西都不能隐式地从一个模块“泄漏”到另一个模块。

这是 #include 文件的一个关键问题。

#include 中的任何内容都会影响所有后续的 #include。

**顺序独立性是“代码卫生”和性能的关键**

### 说明
`C++20`引入了一个重要的特性，即**模块化（Modules）**。模块化是一种改变C++代码组织方式的方式，旨在**解决传统头文件和预处理器宏导致的一些问题**。

传统C++中，我们使用头文件来声明类、函数和变量的接口，并使用预处理器宏来进行条件编译。这种方式存在一些问题，比如头文件的包含关系复杂、编译时间长等。

模块化通过引入新的关键字`module`来解决这些问题。一个模块可以看作是一个独立的代码单元，其中包含了导出的接口和实现细节。模块之间通过导入和导出来建立依赖关系，而不再依赖于头文件的包含顺序。

使用模块化的好处包括：

1. 没有头文件。
2. 声明实现仍然可分离, 但非必要。
3. 可以显式指定导出哪些类或函数。
4. 不需要头文件重复引入宏（`include guards`）。
5. 模块之间名称可以相同，并且不会冲突。
6. 模块只处理一次，编译更快（头文件每次引入都需要处理，需要通过 `pragma once` 约束）。
7. 预处理宏只在模块内有效。
8. 模块的引入与引入顺序无关。

## 使用示例

1. **创建模块接口文件**：创建一个以`.ixx`或`.cppm`为后缀的模块接口文件，例如my_module.ixx。在模块接口文件中，定义模块的接口和导出的符号。

2. **导出符号**：在模块接口文件中，使用`export`关键字来导出需要对外暴露的类、函数和变量。例如：
```C++
export module my_module;

export void my_function();
export class MyClass;
export int my_variable;
```

3. **创建模块实现文件**：在同一个目录下创建一个与模块接口文件同名但后缀为.cpp的模块实现文件，例如my_module.cpp。在模块实现文件中，提供对导出符号的实现。(实际上在模块.ixx里面实现也可以)

4. 导入模块：在使用模块的源文件中，使用`import`关键字来导入所需的模块。例如：

```C++
import my_module;

int main() {
    my_function();
    MyClass obj;
    // 使用my_variable等模块导出的符号
    return 0;
}
```

### 实例

`my_module.ixx`内:

```C++
export module my_module; // 模块声明

// 只能使用 import 导入头文件， 不能 #include
import <iostream>;

// using namespace std; !禁止!
/*
  对于using namespace std;这样的语句，在模块接口文件中应该避免使用。
  模块化的目标之一是提供更清晰的代码组织，因此最好明确指定使用的命名空间，而不是使用using语句。
  这样可以在模块被导入时，避免与导入模块的命名空间发生冲突。
*/

// 需要写 export 关键字 才可以被外界使用, 不然只能内部调用
export void my_function() {
	std::cout << "这是模块的 my_function() 函数!" << std::endl;
}

// 不能使用 导出内部链接属性的东西
// export static int c = 1;

// 但链接属性在文件内部是可以用的，只是不能导出
static int c = 1;
// 因为模块也是可以分文件的!, 所以可以在这里声明， 在那里实现


// 外界不可见
int b = 2;

export class MyHXClass {
public:
	MyHXClass(void) {};
	void putText(void);

	// 静态函数可以导出
	static void out(void);

	// 受保护
protected:
	void fun_pro(void) {

	};

	// 私有
private:
	void fun_pri(void) {

	}
};

// 对于类只要 export 写了这个， 其方法就可以不写
void MyHXClass::out(void) {
	std::cout << "static" << std::endl;
}

void MyHXClass::putText(void) {
	std::cout << "The MyClass" << std::endl;
}

export int my_variable;
```

`.cpp`使用:

```C++
// 导入模块
import my_module;


void new_cpp20_001(void)
{
	my_function();

	MyHXClass::out();

	MyHXClass a;
	a.putText();

	// 显然不能用
	// a.fun_pro();
	// a.fun_pri();
}
```

当然, 不是模块里面无法使用 #include, 也可以这样
```C++
module;
#include <iostream>
module my_module;
```

## 注解
### [1]
参考链接

当前方法似乎还是有点超前，不同编译器有不同的使用方法<sup>[?]</sup>,如果需要使用请网上查询!

[知乎:C++20 新特性: modules 及实现现状](https://zhuanlan.zhihu.com/p/350136757)