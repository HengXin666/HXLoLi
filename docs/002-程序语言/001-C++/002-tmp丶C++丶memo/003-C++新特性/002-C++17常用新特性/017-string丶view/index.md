# std::string_view
## 概述
C++中字符串有两种形式，`char*`和`std::string`，`string`类型封装了`char*`字符串，让我们对字符串的操作方便了很多，但是会有些许性能的损失，而且由`char*`转为`string`类型，需要调用`string`类拷贝构造函数，也就是说**需要重新申请一片内存**，但`如果只是对源字符串做只读操作`，这样的构造行为显然是**不必要**的。

在`C++17`中，增加了`std::string_view`类型，它通过`char*`字符串构造，但是并不会去申请内存重新创建一份该字符串对象，只是`char*`字符串的一个视图，优化了不必要的内存操作。相应地，对源字符串<b style="color:red">只有读权限，没有写权限</b>

## 使用

```C++
#include <iostream>
using namespace std;

void func1(string_view str_v) {
    cout << str_v << endl;
    return;
}

int main() {
    const char* charStr = "hello world";
    string str{ charStr };
    string_view str_v(charStr, strlen(charStr));
  
    cout << "str: " << str << endl;
    cout << "str_v: " << str_v << endl;
    func1(str_v);
  
    return 0;
}
```
