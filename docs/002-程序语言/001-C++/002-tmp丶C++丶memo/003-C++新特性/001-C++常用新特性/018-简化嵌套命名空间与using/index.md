# 简化嵌套命名空间

如下

```C++
// C++11
namespace A {
	namespace B {
 		namespace C {
   			void fun(void) {
            	// ...
            }
        }
    }
}

// C++17
namespace A::B::C {
	void fun(void) {
    	// ...
    }
}
```

# using声明语句可以声明多个名称

如下

```C++
// C++11 (只能分开写)
using std::cout;
using std::cin;

// C++17 (可以通过空格隔开)
using std::cout, std::cin;
```
