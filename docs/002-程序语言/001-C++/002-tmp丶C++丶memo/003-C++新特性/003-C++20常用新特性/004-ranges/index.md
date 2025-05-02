# 范围 ranges
## 说明
`范围库`始于 `Eric Niebler` 对 `STL` 序列观念的推广和现代化的工作。它提供了更易于使用、更通用及性能更好的标准库算法。

例如，C++20 标准库为整个容器的操作提供了期待已久的更简单的表示方法。

```C++
void func1(vector<string>& s) {
	sort(s); // 而不是 sort(vs.begin(), vs.end());
}
```

`C++20` 引入了`范围（ranges）`库，它提供了一种新的方式来进行序列操作和转换。范围库使得对序列的操作更加直观、灵活和高效。

范围库引入了一组新的概念，如**范围视图（range views）、范围适配器（range adapters）和范围操作符（range operators）**。通过这些概念，我们可以使用类似于**管道操作符**的语法来**组合和链式调用序列操作**。

`范围视图是一个惰性的序列，它代表了对原始序列的某种操作或转换`。例如，filter 视图可以根据指定的条件过滤序列中的元素，transform 视图可以对序列中的元素执行某种变换操作。

`范围适配器是一种用于修改或扩展范围视图的操作符`。例如，take 适配器可以限制范围视图返回的元素数量，sort 适配器可以对范围视图中的元素进行排序。

`范围操作符是一组用于组合和组装范围视图和适配器的操作符`。通过使用这些操作符，我们可以构建出复杂的序列操作链。

范围库还提供了一些常用的范围视图和适配器，如 views::transform、views::filter、views::take、views::drop 等。此外，范围库还提供了许多用于数值范围的操作，如 views::iota、views::counted 等。

总体而言，C++20 的范围库为我们提供了一种更加现代和便捷的方式来处理序列操作。它可以使代码更加清晰、简洁和可读，同时也提供了更好的性能优化机会。

## 示例

```C++
#include <iostream>
#include <ranges>
#include <vector>

int main() {
  std::vector<int> v{1, 2, 3, 4, 5};

  // 使用 filter 视图过滤奇数
  auto filtered = v | std::views::filter([](int x) { return x % 2 == 0; });

  // 使用 transform 视图对偶数进行平方操作
  auto squared = filtered | std::views::transform([](int x) { return x * x; });

  // 使用 take 适配器获取前三个
  auto first_three = squared | std::views::take(3);

  // 输出结果
  for (auto i : first_three) {
    std::cout << i << ' ';
  }
  std::cout << '\n';

  return 0;
}
```

该程序首先创建了一个 `std::vector` 对象 `v`，然后使用 `filter` 视图将其转换为只包含偶数的序列。接着使用 `transform` 视图对偶数进行平方操作，再使用 `take` 适配器获取前三个元素。最后，程序遍历输出结果。

该程序中使用的范围库操作符和函数如下：

- `|`：用于链式调用范围视图和适配器。
- `std::views::filter`：用于创建一个过滤视图，其中只包含符合条件的元素。
- `std::views::transform`：用于创建一个变换视图，其中对每个元素进行变换操作。
- `std::views::take`：用于创建一个限制器适配器，其中只返回前 N 个元素。

### [2]

当然 管道符`|` 也可以这样用:

```C++
void new_cpp20_003(void)
{
	auto ints = views::iota(0, 10);//生成0-9
	auto even = [](int i) { return 0 == i % 2; };
	auto square = [](int i) { return i * i; };

	for (int i : ints | views::filter(even) | views::transform(square))
		cout << i << ' ';
}
```