# C++模板类和模板函数放在多个文件中报错: LNK2019 解决方案
## 亲身体验
VS 报错如下:

```C++
错误 LNK2019 无法解析的外部符号 "public: bool __cdecl HX::ThreadPool::addTask<void (__cdecl&)(int &),int>(void (__cdecl&)(int &),int &&)" (??$addTask@A6AXAEAH@ZH@ThreadPool@HX@@QEAA_NA6AXAEAH@Z$$QEAH@Z)，函数 main 中引用了该符号 线程池 D:\command\cc++\C++\线程池\main.obj 1
```

Linux g++ 报错如下:


```Shell
[root@localhost threadPool]# g++ main.cpp HXThreadPool.cpp -l pthread -o app
/tmp/ccJv9Y48.o：在函数‘main’中：
main.cpp:(.text+0x88)：对‘bool HX::ThreadPool::addTask<void (&)()>(void (&)())’未定义的引用
collect2: 错误：ld 返回 1
```

问遍gpt, 查遍度娘, 依据无果...

偶然间, 把`main`函数移到 `HXThreadPool.cpp` 里 (由多文件, 变为单文件)

突然就编译通过了... 但是分开依旧报错... 很是不解...

但是可以锁定问题是模版产生的, 因为是找不到模版实例后的函数.

故抱着试一试的心态, 搜索 `c++可变参数模版在多文件中出现链接问题` 后 找到了 [C++模板类和模板函数放在多个文件中的实践](https://blog.csdn.net/jinking01/article/details/117534158) 宝藏文章, 立刻就解决了我的问题!

我的问题代码:

```C++
template<typename Function, typename... Args>
bool HX::ThreadPool::addTask(Function&& func, Args&&... args) {
    // ...
}
```


## 解决方案
### 在 .h 文件中实现模版函数/模版类 【推荐】
一般情况下我们编写C++代码的时候都会习惯性的将声明性的代码放在`.h`文件中，然后将实现文件放在`.cpp`文件中，只需加载相关的`.h`文件即可。

但是在**类模板**中这个是行不通的，如果遇到这种情况，只需要将类模板的实现在其对应的声明文件中就可以解决问题。
### #include ".cpp" 而不是 .h

```C++
#include "xxx.cpp"

int main() {
    // ...
}
```

### 把类模版写到main函数文件中
是方法, 但不好.