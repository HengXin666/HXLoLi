# lambda表达式捕获 *this
## 说明
一般情况下，lambda表达式访问`类成员变量`时需要捕获this指针，这个this指针指向`原对象`，即相当于一个`引用`，在**多线程**情况下，有可能lambda的生命周期超过了对象的生命周期，此时，对成员变量的访问是未定义的。

因此`C++17`中增加捕获*this，此时捕获的是对象的`副本`，也可以理解为只能对原对象进行读操作，没有写权限。

## 使用

```C++
#include <iostream>
using namespace std;

class ClassTest {
public:
  	int num;
   
  	void func1() {
  		auto lamfunc = [*this]() { cout << num << endl; };
  		lamfunc();
  	}
};

int main() {
    ClassTest a;
    a.num = 100;
    a.func1();
	return 0;
}
```