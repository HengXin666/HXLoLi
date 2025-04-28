<style>
h2 {
  margin-top: 80px;
  color: rgb(101,229,255);
}
</style>

<h1>STL</h1>
<div style="text-align:center;">
<span style="text-align:center; font-size: 32px;">STL</span>
<span style="text-align:center; font-size: 17px; line-height:20px"><br>Standard Template Library<br>标准库模板</span>
</div>

#

## STL概述
STL,学名Standard Template Library，一般称它为标准模板库。

C++ 对模板（Template）支持得很好，STL 就是借助模板把常用的数据结构及其算法都实现了一遍，并且做到了数据结构和算法的分离。例如，vector 的底层为顺序表（数组），list 的底层为双向链表，deque 的底层为循环队列，set 的底层为红黑树，hash_set 的底层为哈希表。

从根本上说，STL是一些“容器”的集合，这些“容器”有list,vector,set,map等，STL也是算法和其他一些组件的集合。

## STL基本组成
STL从广义上讲分为三类
1. 容器（Container），是一种数据结构，如list，vector，和deques ，以模板类的方法提供。为了访问容器中的数据，可以使用由容器类输出的迭代器；

2. 迭代器（Iterator），提供了访问容器中对象的方法。例如，可以使用一对迭代器指定list或vector中的一定范围的对象。迭代器就如同一个指针。事实上，C++的指针也是一种迭代器。但是，迭代器也可以是那些定义了operator*()以及其他类似于指针的操作符地方法的类对象；

3. 算法（Algorithm），是用来操作容器中的数据的模板函数。例如，STL用sort()来对一个vector中的数据进行排序，用find()来搜索一个list中的对象，函数本身与他们操作的数据的结构和类型无关，因此他们可以在从简单数组到高度复杂容器的任何数据结构上使用；

# 常用数据结构
> 请先学习: [数据结构与算法-目録](../../../../../001-计佬常識/001-数据结构与算法/001-【algorithm】目録/index.md)
>
> 本章节只学习如何使用!

## std::string / std::wstring

1. 动态分配内存的字符串
2. 可以实时获取长度
3. 不同于py和java的String, 它是可变类型!(可以理解为 java的 [StringBuilder和StringBuffer](../../../../002-Java/002-Java基础/007-字符串丶/002-StringBuilder和StringBuffer/index.md) 的样子)

```C++
std::string str("字符串");
std::string str2(len, char); // 初始化为长度为 len, 值全部为 char 的字符串(因为没有分配长度的构造函数, 所以可以用这个代替!)
str.size();

str += "张三";

str.c_str(); // 获取得到C式的字符串(对应为char *类型) 可以在printf中使用%s格式化

string sub = str.substr(index, len); // 获取子字符串 从索引 [index, index + len]

str[1]; // 可以使用下标进行访问.
```

常用函数:

```C++
#include <string>

std::string str = to_string(1234.0712); // 有多个重载, 但是没有char类型, char类型请使用 str.push_back(char); 来进行
```

## std::vector

`std::vector`的底层实现通常是一个动态分配内存的数组。在创建`vector`时，它会在堆上分配一段连续的内存空间来存储元素。当需要添加新元素时，如果当前空间不足，则会重新分配一块更大的内存空间，并将所有元素移动到新的空间中。

当向 std::vector 添加新元素时，如果当前内存空间不足以容纳新元素，通常会将当前容量扩展一倍（即乘以2），然后将所有元素移动到新的更大空间中。

更多初始化: [声明时初始化二维向量](../005-声明时初始化二维向量/index.md)

```C++
std::vector<int> arr(3, -1); // 长度为3, 初始化为-1的 int数组

for (auto& it : arr)
    printf("%d ", it);

// 可以索引访问
for (int i = 0; i < arr.size(); ++i) {
    printf("%d%c", arr[i], i != arr.size() - 1 ? ' ' : '\n');
}

arr.push_back(3); 增加长度为 4, 并且放入一个元素`3`, [尾插]

arr.insert(arr.end(), v.begin(), v.end()); // vector 插入 vector
```

注: 使用`rbegin()`可以获得反向迭代器, 而`cbegin()`是获得`const begin()`! 

<span style="color:red">不内置排序算法</span>, 需要使用:

```C++
#include <algorithm>
#include <vector>

std::vector<int> arr(3, -1);

std::sort(arr.begin(), arr.end()); // 对 arr 进行排序
```

## std::stack 栈

实现了动态内存分配的栈 (它默认基于 deque 或 vector 实现)

```C++
#include <stack>

std::stack<int> s;
s.push(1);       // 入栈: 向栈顶添加元素
int x = s.top(); // 返回栈顶元素
s.pop();         // 弹栈: 移除栈顶元素
```

## std::queue 队列

实现了动态内存分配的队列 (它基于其他容器（通常是 deque 或 list）实现)

```C++
#include <queue>

std::queue<int> q;

p.push(const T& val); // 将元素 val 推入队列的尾部。
p.pop();              // 弹出队列头部的元素，不返回任何值。
p.front();            // 返回队列头部的元素的引用，不删除该元素。
p.back();             // 返回队列尾部的元素的引用，不删除该元素。
```

## std::deque 双端队列
deque双端队列，可以对头端和尾端进行插入删除操作

deque队列为一个给定类型的元素进行线性处理，像向量一样，它能够快速地随机访问任一个元素，并且能够高效地插入和删除容器的尾部元素。但它又与vector不同，deque支持高效插入和删除容器的头部元素。

- 可以使用下标访问

std::deque（双端队列）和 std::vector 都是标准库中的动态数组容器，它们有一些区别：

- 内部实现：
    - std::deque 使用多个固定大小的缓冲区来存储元素，这些缓冲区以块的形式连接在一起，从而实现高效地在两端进行插入和删除操作。
    - std::vector 使用单个连续的内存块来存储元素，因此在中间插入或删除元素时可能会导致元素的移动。
- 内存分配策略：
    - 对于大部分情况，std::vector 只需要在尾部动态扩展内存，因此在尾部添加或删除元素的性能较好。
    - std::deque 在两端都可以高效地进行插入和删除操作，但在中间插入或删除元素时也比 std::vector 更高效。
- 迭代器失效：
    - 插入或删除 std::vector 中间位置的元素可能会使得迭代器失效，需要重新获取迭代器。
    - std::deque 在两端插入或删除元素不会使迭代器失效，但在中间位置插入或删除元素时会使得除了该位置之外的所有迭代器失效。
- 存取速度：
    - std::vector 由于内存连续存储，因此随机访问速度更快。
    - std::deque 在两端进行插入和删除操作速度更快，但随机访问速度可能略慢一些。

对于 std::deque 的各种操作，其时间复杂度如下：
- 随机访问（Random Access）：std::deque 支持随机访问，即可以通过索引快速访问到指定位置的元素。时间复杂度为 O(1)。这是因为 std::deque 内部使用多个块来存储数据，每个块都是连续的内存块，因此可以通过简单的数学计算来确定元素在哪个块中，从而实现 O(1) 的随机访问。
- 头部和尾部插入/删除（Front and Back Insertion/Deletion）：对于头部和尾部的插入和删除操作，std::deque 的时间复杂度也为 O(1)。这是因为在头部或尾部进行插入和删除时，只需要调整块的连接关系而不需要移动大量元素。
- 中间插入/删除（Insertion/Deletion in the Middle）：当在 std::deque 的中间位置进行插入或删除操作时，时间复杂度为 O(n)，其中 n 是元素的总数。这是因为可能需要移动块之间的元素以及调整块的连接关系。

<span style="color:red">不内置排序算法</span>, 需要使用: sort函数!

## std::list 双向链表

双向链表, 头/尾的插入和删除为 O(1), 已知位置的删除为O(1)!;

<span style="color:red">内置排序算法, 不支持随机访问(下标访问)</span>

- 注: `std::list`没有内置`find()`类方法, 所以需要使用`<algorithm>`的`find(list.begin(), list.end(), var)`得到迭代器, (记得要和`.end()`判断一下, 看看有没有找到)

```C++
#inlcude <list>

std::list<int> L = { 1,4,3,3,2,6,7,9,-1 };
L.sort();       // 排序, 默认是从小到大
L.sort(cmp);    // 自定义排序方式, 从大到小

// 翻转list
L.reverse();
```

## std::set / std::multiset 集合

std::set 和 std::multiset 属于关联式容器, 底层使用红黑树([红黑树](../../../../../001-计佬常識/001-数据结构与算法/008-【数据结构】高阶搜索树/002-红黑树/001-红黑树丶/index.md))实现

既然是红黑树, 那么就会排序, 一般是`begin()小 ~ end()大`, 当然也可以自定义排序函数.

std::set 不支持 重复元素, 而 std::multiset 的元素可以重复

set的插入会自动排序, 所以只提供了 **插入**`.insert(const <T>& var)` 这一个方法.

set的查找 `.find(var)` 找到返回`迭代器`, 找不到返回`.end()迭代器`

set的删除 `.erase(var)` 删除值为`var`的元素 (也可以删除一个迭代器区间的元素)

std::set / std::multiset 均在 `<set>` 里面

*还有一种使用哈希实现的`unordered_set` --> [unordered系列](../004-unordered系列/index.md)*

## std::pair 对组

pair 是一个对组 <T1, T2> 于`#include <utility>`

本质上是一个结构体

它可以存储两种数据类型

示例:

```C++
#include <iostream>
#include <utility>

using namespace std;

std::pair<const char*, int> _demo_get(int a, const char* b)
{
    return make_pair(b, a);
}

void stl_011(void)
{
    // 声明
    std::pair<int, const char*> p(123, "啊?");

    // 访问
    cout << p.first << " " << p.second << endl;

    // 因为本质上是结构体, 不是类, 所以没有什么函数
    p = make_pair(456, "这...");
    cout << p.first << " " << p.second << endl;

    // 如果是函数返回值, 可以使用stl::tie接受
    int ages;
    const char* name;
    std::tie(name, ages) = _demo_get(p.first, p.second);
}
```

## std::map / std::multimap 字典/映射

一样是底层使用红黑树实现, std::multimap的`key`可以重复

- 查找, 同set, 使用`.find(const <T>& key)` 通过唯一的主键查找(std::map), std::multimap则会返回第一个符合的迭代器.

重载了`[]`运算符, 如果 m1[k] 的 键k存在, 那么会修改它, 如果不存在则创建它
```C++
#inlcude <map>

// 声明
std::map<const char*, int> map = { {"阿哲", 12}, {"宁这", 24}, {"字典", 36} };    // 内部自动排序的
// 打印
auto a = map.begin();
cout << a->first << " " << a->second << endl;
cout << map["阿哲"] << endl;
```

其他使用几乎同`set`

*还有一种使用哈希实现的`unordered_map` --> [unordered系列](../004-unordered系列/index.md)*

<table>
    <caption>
        表 1 C++ std::multimap 容器常用成员方法</caption>
    <tbody>
        <tr>
            <th>
                成员方法</th>
            <th>
                功能</th>
        </tr>
        <tr>
            <td>
                begin()</td>
            <td>
                返回指向容器中第一个（注意，是已排好序的第一个）键值对的双向迭代器。如果 multimap 容器用 const 限定，则该方法返回的是 const 类型的双向迭代器。</td>
        </tr>
        <tr>
            <td>
                end()</td>
            <td>
                返回指向容器最后一个元素（注意，是已排好序的最后一个）所在位置后一个位置的双向迭代器，通常和 begin() 结合使用。如果 multimap 容器用 const 限定，则该方法返回的是 const 类型的双向迭代器。</td>
        </tr>
        <tr>
            <td>
                rbegin()</td>
            <td>
                返回指向最后一个（注意，是已排好序的最后一个）元素的反向双向迭代器。如果 multimap 容器用 const 限定，则该方法返回的是 const 类型的反向双向迭代器。</td>
        </tr>
        <tr>
            <td>
                rend()</td>
            <td>
                返回指向第一个（注意，是已排好序的第一个）元素所在位置前一个位置的反向双向迭代器。如果 multimap 容器用 const 限定，则该方法返回的是 const 类型的反向双向迭代器。</td>
        </tr>
        <tr>
            <td>
                cbegin()</td>
            <td>
                和 begin() 功能相同，只不过在其基础上，增加了 const 属性，不能用于修改容器内存储的键值对。</td>
        </tr>
        <tr>
            <td>
                cend()</td>
            <td>
                和 end() 功能相同，只不过在其基础上，增加了 const 属性，不能用于修改容器内存储的键值对。</td>
        </tr>
        <tr>
            <td>
                crbegin()</td>
            <td>
                和 rbegin() 功能相同，只不过在其基础上，增加了 const 属性，不能用于修改容器内存储的键值对。</td>
        </tr>
        <tr>
            <td>
                crend()</td>
            <td>
                和 rend() 功能相同，只不过在其基础上，增加了 const 属性，不能用于修改容器内存储的键值对。</td>
        </tr>
        <tr>
            <td>
                find(key)</td>
            <td>
                在 multimap 容器中查找首个键为 key 的键值对，如果成功找到，则返回指向该键值对的双向迭代器；反之，则返回和 end() 方法一样的迭代器。另外，如果 multimap 容器用 const 限定，则该方法返回的是 const 类型的双向迭代器。</td>
        </tr>
        <tr>
            <td>
                lower_bound(key)</td>
            <td>
                返回一个指向当前 multimap 容器中第一个大于或等于 key 的键值对的双向迭代器。如果 multimap 容器用 const 限定，则该方法返回的是 const 类型的双向迭代器。</td>
        </tr>
        <tr>
            <td>
                upper_bound(key)</td>
            <td>
                返回一个指向当前 multimap 容器中第一个大于 key 的键值对的迭代器。如果 multimap 容器用 const 限定，则该方法返回的是 const 类型的双向迭代器。</td>
        </tr>
        <tr>
            <td>
                equal_range(key)</td>
            <td>
                该方法返回一个 pair 对象（包含 2 个双向迭代器），其中 pair.first 和 lower_bound() 方法的返回值等价，pair.second 和 upper_bound() 方法的返回值等价。也就是说，该方法将返回一个范围，该范围中包含的键为 key 的键值对。</td>
        </tr>
        <tr>
            <td>
                empty()&nbsp;</td>
            <td>
                若容器为空，则返回 true；否则 false。</td>
        </tr>
        <tr>
            <td>
                size()</td>
            <td>
                返回当前 multimap&nbsp;容器中存有键值对的个数。</td>
        </tr>
        <tr>
            <td>
                max_size()</td>
            <td>
                返回 multimap 容器所能容纳键值对的最大个数，不同的操作系统，其返回值亦不相同。</td>
        </tr>
        <tr>
            <td>
                insert()</td>
            <td>
                向 multimap 容器中插入键值对。</td>
        </tr>
        <tr>
            <td>
                erase()</td>
            <td>
                删除 multimap 容器指定位置、指定键（key）值或者指定区域内的键值对。</td>
        </tr>
        <tr>
            <td>
                swap()</td>
            <td>
                交换 2 个 multimap 容器中存储的键值对，这意味着，操作的 2 个键值对的类型必须相同。</td>
        </tr>
        <tr>
            <td>
                clear()</td>
            <td>
                清空 multimap 容器中所有的键值对，使 multimap 容器的 size() 为 0。</td>
        </tr>
        <tr>
            <td>
                emplace()</td>
            <td>
                在当前 multimap 容器中的指定位置处构造新键值对。其效果和插入键值对一样，但效率更高。</td>
        </tr>
        <tr>
            <td>
                emplace_hint()</td>
            <td>
                在本质上和 emplace() 在 multimap 容器中构造新键值对的方式是一样的，不同之处在于，使用者必须为该方法提供一个指示键值对生成位置的迭代器，并作为该方法的第一个参数。</td>
        </tr>
        <tr>
            <td>
                count(key)</td>
            <td>
                在当前 multimap 容器中，查找键为 key 的键值对的个数并返回。</td>
        </tr>
    </tbody>
</table>

<b style="color:red">和`map`容器相比，`multimap`未提供`at()`成员方法，也没有重载`[]`运算符。</b>这意味着，`map`容器中通过指定键获取指定指定键值对的方式，将不再适用于`multimap`容器。其实这很好理解，因为`multimap`容器中指定的键可能对应多个键值对，而不再是`1`个。

另外值的一提的是，由于`multimap`容器可存储多个具有相同键的键值对，因此表`1`中的`lower_bound()`、`upper_bound()`、`equal_range()`以及`count()`成员方法会经常用到。


## std::priority_queue 优先队列

实际上是对 堆 的封装

```C++
std::priority_queue<类型, std::vector<类型>/* 底层默认是使用可变数组实现堆 */, std::less<类型>/*大根堆(默认)*/ std::greater<类型>/*小根堆*/ >
```

```C++
#include <iostream>
#include <queue>
#include <vector>
#include <utility>

void stl_0144(void)
{
    {
        //std::priority_queue<int> pq{ 1,4,3,2,3,-1,2 }; // 不支持直接初始化
        std::priority_queue<int> pq; // 默认是大根堆 (大的先出)
        pq.push(2);
        pq.push(-2);
        pq.push(21);
        pq.push(5);

        while (pq.size())
        {
            std::cout << pq.top() << " ";
            pq.pop();
        }
    }
    std::cout << "\n\n";
    {
        std::priority_queue<int, std::vector<int>, std::greater<int>> pq; // 指定为小根堆
        pq.push(2);
        pq.push(-2);
        pq.push(21);
        pq.push(5);

        while (pq.size())
        {
            std::cout << pq.top() << " ";
            pq.pop();
        }
    }
    std::cout << "\n\n";
    {
        auto fun = [](const std::pair<const char*, int>& f1, const std::pair<const char*, int>& f2) {
            return f1.second > f2.second; // f1 > f2 (true) 则交换 (降序)
            };
        std::priority_queue<std::pair<const char*, int>, std::vector<std::pair<const char*, int>>, decltype(fun)> pq(fun);
        pq.push({"a", 2});
        pq.push({"b", -2});
        pq.push({"c", 21}); 
        pq.push({"d", 5});

        while (pq.size())
        {
            std::cout << pq.top().first << " " << pq.top().second << "\n";
            pq.pop();
        }
    }
}
```

# 常用算法
> 这里只说明最最最常用的
## 排序算法

```C++
#include <algorithm>

// 内部会智能选择使用排序算法, len <= 16 是 冒泡, 然后是根据 logN 的深度 选择是快排还是堆排 (具体请百度)

std::sort(起始迭代器, 结束迭代器); // 一般是对vector进行排序
```
