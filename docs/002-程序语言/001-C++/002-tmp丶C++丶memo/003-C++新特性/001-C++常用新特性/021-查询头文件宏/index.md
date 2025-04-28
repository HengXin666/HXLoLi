# __has_include
## 说明
`跨平台项目`需要考虑不同平台编译器的实现，使用`__has_include`可以**判断当前环境下是否存在某个头文件**。

```C++
int main() {
#if __has_include("iostream")
	cout << "iostream exist." << endl;
#endif
#if __has_include(<cmath>)
	cout << "<cmath> exist." << endl;
#endif
	return 0;
}
```
