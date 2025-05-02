# std::format
## 说明
iostream 库提供了类型安全的 I/O 的扩展，但是它的格式化工具比较弱。

另外，还有的人不喜欢使用 << 分隔输出值的方式。

格式化库提供了一种类 printf 的方式去组装字符串和格式化输出值，同时这种方法类型安全、快捷，并能和 iostream 协同工作。

类型中带有 << 运算符的可以在一个格式化的字符串中输出。

`C++20` 的 `std::format` 是一个新的标准库特性，它允许类似于 printf 的格式化输出字符串 (使用的时候像是Python的print)

## 示例


```C++
#include <iostream>
#include <format>   // 记得导入头文件
using namespace std;

void new_cpp20_004(void)
{
	string str = "Hello C++";
	// 按顺序分配
	cout << format("这个字符串是'{}', 长度是 {} .", str, str.size()) << endl;

	// 指定参数位置(索引， 从0开始)
	cout << format("这个字符串是'{1}', 长度是 {0} .", str.size(), str) << endl;
}
```

输出:
```OUT
这个字符串是'Hello C++', 长度是 9 .
这个字符串是'Hello C++', 长度是 9 .
```