# 并发
## 概述
> 注:本文不是教你多线程，只是告诉你新增了这些，如果需要学习请去参考链接<sup>[1]</sup>看看

`C++20` 并发编程 `std::promise`

`std::promise`和`std::future`是一对, 通过它们可以进行更加灵活的任务
控制

`promise`通过函数`set_value()`传入一个值, 异常, 或者通知, 并异步的获取结果(示例[1])

- `std::future`: 从`promise`获取值, 询问值是否可用, 等待通知, 创建`shared_future`

- `std::future_status`: 调用后`wait_for`或者`wait_until`返回的结果(如下) (示例[2])

```C++
enum class future_status
{
    ready,   // 成功
    timeout, // 超时
    deferred // 延迟
};
```

## 示例
### [1]

这段代码演示了如何使用promise和future来实现等待异步结果的功能
```C++
#include <iostream>
#include <future> // 对于库
#include <format>
using namespace std;

// 定义一个函数，接受两个整数参数，并通过promise传递它们的积
void product(promise<int>&& intPromise, int v1, int v2)
{
	intPromise.set_value(v1 * v2);  // 将v1和v2的积设置为promise的值
}

// 主函数
void new_cpp20_005(void)
{
	int num1 = 200;
	int num2 = 300;

	// 创建一个promise对象，并获取与之关联的future对象
	promise<int> productPromise;
	future<int> productResult = productPromise.get_future();

	// 创建一个新的线程，调用product()函数，并将promise对象作为参数传递给该函数
	jthread productThread(product, move(productPromise), num1, num2);

	// 等待异步计算完成，并输出结果到控制台
	cout << format("product is {}", productResult.get()) << endl;
}
```
### [2]

```C++
#include <iostream>
#include <future>
#include <format>
using namespace std;

void getAnswer(promise<int> intPromise)
{
	this_thread::sleep_for(2s); // 模拟耗时操作
	intPromise.set_value(100); // 设置promise的值为100
}

static void _005_new_text(void)
{
	promise<int> answerPromise; // 创建一个promise对象
	auto fut = answerPromise.get_future(); // 获取与promise关联的future对象
	jthread productThread(getAnswer, move(answerPromise)); // 创建一个新线程，并将promise对象作为参数传递给getAnswer函数

	future_status status{};
	do
	{
		status = fut.wait_for(0.5s); // 等待0.5秒，检查异步计算是否完成
		cout << "结果未准备完成 " << endl;
	} while (status != future_status::ready); // 循环等待，直到异步计算完成

	cout << format("answer is {}\n", fut.get()); // 输出异步计算的结果
}
```

## 注解
### [1]
参考链接
[C++11 多线程（std::thread）详解](https://blog.csdn.net/sjc_0910/article/details/118861539)

### [2]
[C ++ 20中的新线程（jthread）功能](https://www.jianshu.com/p/c610ad5db6b7)