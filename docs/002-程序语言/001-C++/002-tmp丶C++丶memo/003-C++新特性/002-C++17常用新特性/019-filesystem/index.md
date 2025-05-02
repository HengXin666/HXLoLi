# std::filesystem
## 概述
`C++17`引入了 `std::filesystem` 库，它为文件系统操作提供了一个现代化的 C++ 接口。该库包含了一组类和函数，可用于在计算机上执行各种文件和目录操作，例如创建、移动、重命名、删除、复制和遍历文件和目录等。

*本文只是粗略介绍, 若需详细了解, 请去查阅文档.*

简单而言，就是类似Python的`OS`模块<sup>[2]</sup>

### 常用类
path 类：说白了该类只是对字符串（路径）进行一些处理，这也是文件系统的基石。

directory_entry 类：功如其名，文件入口，这个类才真正接触文件。 

directory_iterator 类：获取文件系统目录中文件的迭代器容器，其元素为 directory_entry对象（可用于遍历目录）

file_status 类：用于获取和修改文件（或目录）的属性（需要了解C++11的强枚举类型（即枚举类）<sup>[3]</sup>）

### 常用函数

需要使用，请去[C++官方中文文档](https://zh.cppreference.com/w/cpp/filesystem)查阅，此处仅是抛砖引玉.


|函数原型|作用|
|-------|---|
|[std::filesystem::current_path](https://zh.cppreference.com/w/cpp/filesystem/current_path)|返回或更改当前工作目录|
|[std::filesystem::file_time_type](https://zh.cppreference.com/w/cpp/filesystem/file_time_type)|表示文件时间|

```C++
void copy(const path& from, const path& to) ;//目录复制
 
path absolute(const path& pval, const path& base = current_path()) ;//获取相对于base的绝对路径
 
bool create_directory(const path& pval) ;//当目录不存在时创建目录
 
bool create_directories(const path& pval) ;//形如/a/b/c这样的，如果都不存在，创建目录结构
 
bool exists(const path& pval) ;//用于判断path是否存在
 
uintmax_t file_size(const path& pval) ;//返回目录的大小
 
file_time_type last_write_time(const path& pval) ;//返回目录最后修改日期的file_time_type对象
 
bool remove(const path& pval) ;//删除目录
 
uintmax_t remove_all(const path& pval) ;//递归删除目录下所有文件，返回被成功删除的文件个数
 
void rename(const path& from, const path& to) ;//移动文件或者重命名
```


## 示例
### 遍历单层/递归遍历

```C++
#include <iostream>
#include <ctime>
#include <filesystem>
using namespace std;

void new_cpp17_007(void)
{
	namespace fs = std::filesystem;

	cout << "当前工作目录是: " << fs::current_path() << endl;

	auto x64 = fs::path("./x64");

	if (!fs::exists(x64))
	{
		cout << "该路径不存在!" << endl;
	}
	else
	{
		cout << "正在对 [" << x64 << "] 路径 进行操作..." << endl;
	}

	// 创建一个 fs::directory_options 对象 opt，使用默认选项 none，表示跳过符号链接，权限拒绝将被视为错误
	fs::directory_options opt(fs::directory_options::none);

	// 创建一个 fs::directory_entry 对象 dir，表示指定的目录 x64
	fs::directory_entry dir(x64);

	/************************************************************************/
	/* 遍历一级(一层)目录示例                                                 */
	/************************************************************************/

	// 输出当前目录的名称
	std::cout << "单层遍历 [x64]:\t-->" << dir.path().filename() << endl;

	// 使用 fs::directory_iterator 迭代器遍历指定目录 x64 下的所有文件和子目录。opt 是可选的，用于指定遍历选项
	for (fs::directory_entry const& entry : fs::directory_iterator(x64, opt))
	{
		// 检查当前迭代到的路径 entry 是否为常规文件
		if (entry.is_regular_file())
		{
			// 输出文件的名称和大小。
			cout << entry.path().filename() << "\t size: " << entry.file_size() << endl;
		}
		// 如果当前迭代到的路径 entry 是一个目录，则执行该分支
		else if (entry.is_directory())
		{
			// 输出目录的名称。
			cout << entry.path().filename() << "\t dir" << endl;
		}
	}

	cout << endl;

	/************************************************************************/
	/* 递归遍历示例                                                          */
	/************************************************************************/
	// 输出当前目录的名称
	cout << "x64 all:\t-->" << dir.path().filename() << endl;

	// 使用 fs::recursive_directory_iterator 迭代器递归遍历指定目录 x64 下的所有文件和子目录。opt 是可选的，用于指定遍历选项
	for (fs::directory_entry const& entry : fs::recursive_directory_iterator(x64, opt))
	{
		// 检查当前迭代到的路径 entry 是否为常规文件
		if (entry.is_regular_file())
		{
			// 输出文件的名称、大小和父目录路径
			cout << entry.path().filename() << "\t size: " << entry.file_size() << "\t parent: " << entry.path().parent_path() << endl;
		}
		// 如果当前迭代到的路径 entry 是一个目录，则执行该分支
		else if (entry.is_directory())
		{
			// 输出目录的名称
			cout << entry.path().filename() << "\t dir" << endl;
		}
	}
}
```

## 注解
### [1]
参考链接:

[知乎:C++17新特性:了解std::filesystem的原理和使用](https://zhuanlan.zhihu.com/p/672691945)

[CSDN:C++17中的新增功能std::filesystem](https://blog.csdn.net/shaderdx/article/details/108235666)

[C++官方中文文档](https://zh.cppreference.com/w/cpp/filesystem)

### [2]
`std::filesystem` 库在 C++ 中提供了与 Python 的 os 模块类似的功能。它提供了一组类和函数，用于执行文件系统操作，包括创建、移动、重命名、删除、复制文件和目录，以及遍历目录等。

和 os 模块一样，`std::filesystem` 库也提供了一些常见的操作，例如检查文件是否存在、获取文件大小、获取文件的绝对路径等。此外，它还支持对文件时间戳的访问和修改，以及对文件权限的检查和设置等功能。

不过，`std::filesystem` 库相比于 os 模块在 C++ 中更为强大和灵活，因为它是**直接与底层文件系统交互的库**，而 os 模块则是基于**操作系统提供的接口实现的**。因此，`std::filesystem` 库可以更好地适应不同平台和文件系统的特性，并提供更多的功能选项。(By GPT-3.5)

### [3]
[知乎:C++干货系列——“强”枚举有多强](https://zhuanlan.zhihu.com/p/164634712)