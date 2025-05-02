# inline
## 说明

如果忘记`inline`是什么， 请看: [菜鸟教程: C++ 中的 inline 用法](https://www.runoob.com/w3cnote/cpp-inline-usage.html)

`C++17`中扩展的inline用法，使得可以在`头文件`或者`类内`**初始化**`静态`成员变量

## 示例

```C++
// 头文件.h
inline int var = 100;

// 代码.cpp
class MyClass {
public:
	inline static int var = 666;
};

// 使用
cout << ::var << endl;        // 100
cout << MyClass::var << endl; // 666
```
