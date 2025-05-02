# 折叠表达式
## 概述
`C++17`引入了**折叠表达式（fold expression）**，它是一种简化使用`可变参数模板`的方法。

折叠表达式允许在`编译时`对可变参数包进行操作，将它们展开并根据特定的操作符进行组合。这样可以更方便地处理可变参数包中的元素。

> *注意: 是在编译时候进行的操作，不是运行时!*

## 四种形式
当我们使用折叠表达式进行展开时，有四种不同的方式:

- 一元右折叠（Unary right fold）：`(E op ...)` 展开为 (E1 op (... op (EN-1 op EN)))

- 一元左折叠（Unary left fold）：`(... op E)` 展开为 (((E1 op E2) op ...) op EN)

- 二元右折叠（Binary right fold）：`(E op ... op I)` 展开为 (E1 op (... op (EN−1 op (EN op I))))

- 二元左折叠（Binary left fold）：`(I op ... op E)` 展开为 ((((I op E1) op E2) op ...) op EN)

请注意，在这些表达式中，原本内容: `E` 表示表达式，`op` 表示操作符，`...` 表示可变参数列表。

而展开后的`...`只是一个省略号！ 表示 (E op ...) == (E1 op (E2 op (E3 op (... op (En)))))

## 使用
### 简单使用

```C++
template<class ...T>
static int sum_fun_By17(T... num)
{
	return (num + ...);
}

sum_fun_By17(1, 2, 3, 4, 5);
```

那么编译器会在编译的时候自动帮你展开为
```C++
static int sum_fun_By17(int num_1，int num_2, int num_3, int num_4,int num_5)
{
	return num_1 + num_2 + num_3 + num_4 + num_5;
}

sum_fun_By17(1, 2, 3, 4, 5);
```

相当于一个语法糖.

### 不是那么简单

如果我们打算使用两次 num 例如:

```C++
template<class ...T>
static int sum_fun_By17(T... num)
{
	return (cout << num << " ", num + ...);
}

sum_fun_By17(1, 2, 3, 4, 5);
```
这样是非法的！编译错误！

如果确实需要在折叠表达式中使用， 请使用 匿名函数:

```C++
template<class ...T>
static int sum_fun_By17(T... num)
{
	return ([&] {
		cout << num;
		if (sizeof...(T) > 2)
			cout << " + ";
		else
			cout << " = ";

		return num;
		}() + ...);
}

cout << sum_fun_By17(1, 2, 3, 4, 5, 6, 7, 8, 9);
cout << endl << sum_fun_By17(1);
```

但是，实际上面的代码还是有有点问题:

`sizeof...(T)` 不起作用！ 它只会针对传参时候进行计数，不会像递归调用一样有不同，所以，这种情况下还是建议使用递归调用展开吧qwq...

## 参考文献

[知乎:C++17 折叠表达式](https://zhuanlan.zhihu.com/p/625884960)