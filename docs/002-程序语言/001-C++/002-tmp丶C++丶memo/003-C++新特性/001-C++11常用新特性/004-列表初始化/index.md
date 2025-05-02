# 列表初始化
## 统一列表初始化的使用
在 `C++98` 中，标准允许使用花括号 {} 对数组元素进行统一的列表初始值设定。比如：
```C++
int arr1[] = {1,2,3,4,5};
int arr2[100] = {0};
```

但对于一些自定义类型却不行，例如:

```C++
vector<int> vc{1,2,3,4,5};
```
在`C++98`中是无法编译成功的，只能够定义vector对应之后通过循环进行插入元素达到这个目的.

`C++11`扩大了用大括号括起的列表(初始化列表)的使用范围，使其可用于所有的内置类型和用户自定 义的类型，使用初始化列表时，<span style="color:red">可添加等号(=)，也可不添加</span>.


```C++
#include<iostream>
#include<vector>
#include<map>
using namespace std;

class ClassNum {
public:
    ClassNum(int n1 = 0, int n2 = 0) : _x(n1), _y(n2)
    {}
private:
    int _x;
    int _y;
};

int main() {
    int num1 = { 100 };//定于内置类型
    int num2{ 3 };     //也可以不加=
    //数组
    int arr1[5] = { 1,3,4,5,6 };
    int arr2[] = { 4,5,6,7,8 };
    //STL中的容器
    vector<int>v{ 12,2 };
    map<int, int>mp{ {1,2},{3,4} };
    //自定义类型初始化
    ClassNum p{ 1, 2 };
    return 0;
}
```
