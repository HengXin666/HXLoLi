# forward_list
## 概述
`std::forward_list` 是一个列表容器，使用方法和 `std::list` 基本类似。与 `std::list` 的双向链表的实现不同，`std::forward_list` 使用单向链表进行实现，提供了 O(1) 复杂度的元素插入，不支持快速随机访问，也是标准库容器中**唯一一个不提供 `size()` 方法的容器**

`forward_list`具有插入、删除表项速度快、消耗内存空间少的特点，但只能向前遍历。与其它序列容器(array、vector、deque)相比，**`forward_list`在容器内任意位置的成员的插入、提取(extracting)、移动、删除操作的速度更快，因此被广泛用于`排序算法`**

## 使用方法
和 `std::list` 没有什么差别，只是不支持 `.size()` 方法，需要手动遍历才行.

也没有 pop_back , push_back 方法


```C++
forward_list<int> list_1{4, 3, 3, 2, 2, 3};
list_1.push_front(1);

for (auto& it : list_1)
{
    cout << it << " ";
}

list_1.sort();
```

注意， 有特有方法:

|方法|描述|
|---|---|
|emplace_front()|在容器头部生成一个元素。该函数和 push_front() 的功能相同，但`效率更高`。|
|emplace_after()|在指定位置之后插入一个新元素，并返回一个指向新元素的迭代器。和 insert_after() 的功能相同，但`效率更高`。|


## 参考

1. [C++ STL总结（五）forward_list](https://blog.csdn.net/qq_36383272/article/details/120294078)
2. [C++ STL forward_list容器完全攻略](https://c.biancheng.net/view/6960.html)