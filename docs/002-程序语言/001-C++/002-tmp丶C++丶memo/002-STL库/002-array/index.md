# std::array
## 说明
`std::array` 保存在栈内存中，相比堆内存中的 `std::vector`，它能够灵活的访问容器里面的元素，从而获得更高的性能.

`std::array` 会在编译时创建一个**固定大小**的数组，`std::array` 不能够被隐式的转换成指针，使用 `std::array`只需指定其类型和大小即可。

**它并不支持添加或删除元素等改变大小的操作。也就是说，当定义一个array时，除了指定元素类型，还要指定容器大小**

c++11 封装了相关的数组模板类，不同于 C 风格数组，它不会自动退化成 T* 类型，它能作为聚合类型聚合初始化。

`std::array` 是封装固定大小数组的容器，数组元素下标索引从 0 开始。

## 使用

使用方法和 vector 差不多, 只是需要声明数组大小， 并且不用使用变量.


```C++
#include <iostream>
#include <array>
using namespace std;

static void _text_008(array<int, 3> a)
{
    for (auto& it : a)
    {
        cout << it << " ";
    }
}

void new_cpp11_008(void)
{
    array<int, 3> arr_0;      // 长度为3， 内容为随机的垃圾值
    array<int, 3> arr_1 = {}; // 同上， 但是初始化全部内容为0，等价于 int arr[3] = {0};

    _text_008(arr_1);

    // 支持STL的那些方法
    cout << "size:" << arr_1.size() << endl;

    // 访问
    cout << arr_1[0] << endl     // [] 越界是未定义行为
         << arr_1.at(1) << endl  // at()方法 越界会抛出out_of_range异常
         << get<2>(arr) << endl; // std::tuple 的 get<>(), 越界会导致编译时错误
    // 使用 .data() 方法获取底层数组
    int *p = arr_1.data();
    // 现在和 C 的差不多了
    // 对于与 C 接口交互或需要传递原始指针的情况很有用
    
    int len = 3;
    array<int, len> arr_2 = {}; // 错误的，不能使用变量
}
```

## 附录
有需要的可以看看这里， 我不多写。

[【C++】C++11的std::array的详细剖析](https://blog.csdn.net/qq_38410730/article/details/102802239)