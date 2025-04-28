# CMake 的使用
## 在程序中链接动态库
在程序编写过程中，除了在项目中引入静态库，好多时候也会使用一些标准的或者第三方提供的一些动态库，关于动态库的制作、使用以及在内存中的加载方式和静态库都是不同的，在此不再过多赘述，如有疑惑请参考: [Linux 静态库和动态库](https://subingwen.cn/linux/library/)

在cmake中链接动态库的命令如下:

```CMake
target_link_libraries(
    <target> 
    <PRIVATE|PUBLIC|INTERFACE> <item>... 
    [<PRIVATE|PUBLIC|INTERFACE> <item>...]...)
```

- **target**: 指定要加载动态库的文件的名字
    - 该文件可能是一个源文件
    - 该文件可能是一个动态库文件
    - 该文件可能是一个可执行文件

- **PRIVATE|PUBLIC|INTERFACE**: 动态库的访问权限，默认为`PUBLIC`
    - 如果各个动态库之间没有依赖关系，无需做任何设置，三者没有没有区别，<span style="color:red">一般无需指定，使用默认的`PUBLIC`即可</span>。
    - <span style="color:red">动态库的链接具有传递性</span>，如果动态库 A 链接了动态库B、C，动态库D链接了动态库A，此时动态库D相当于也链接了动态库B、C，并可以使用动态库B、C中定义的方法。
    ```CMake
    target_link_libraries(A B C)
    target_link_libraries(D A)
    ```
    - **PUBLIC**: 在`public`后面的库会被link到前面的target中，并且里面的**符号**(即类/函数的签名/全局变量等)也会被导出，提供给第三方使用。


    - **PRIVATE**: 在`private`后面的库仅被link到前面的target中，并且终结掉，第三方不能感知你调了啥库
        - (比如你代码使用了`std::vector`但是你**不希望**别人用了你的库就可以**不用**`#include <vector>`就可以使用到`std::vector`)


  - **INTERFACE**: 在`interface`后面引入的库不会被链接到前面的target中，只会导出符号。

### 链接系统动态库
动态库的链接和静态库是完全不同的：
- 静态库会在生成可执行程序的链接阶段被打包到可执行程序中，所以可执行程序启动，静态库就被加载到内存中了。
- 动态库在生成可执行程序的链接阶段不会被打包到可执行程序中，当可执行程序被启动并且调用了动态库中的函数的时候，动态库才会被加载到内存

因此，在cmake中指定要链接的动态库的时候，**应该将命令写到生成了可执行文件之后**:

```CMake
cmake_minimum_required(VERSION 3.0)
project(TEST)
file(GLOB SRC_LIST ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
# 添加并指定最终生成的可执行程序名
add_executable(app ${SRC_LIST})
# 指定可执行程序要链接的动态库名字
target_link_libraries(app pthread)
```
在`target_link_libraries(app pthread)`中:

- **app**: 对应的是最终生成的可执行程序的名字
- **pthread**: 这是可执行程序要加载的动态库，这个库是系统提供的线程库，全名为`libpthread.so`，在指定的时候一般会掐头（lib）去尾（.so）。

### 链接第三方动态库

准备了下面, 代码(之前的静态库变成了动态库):
```bash
.
├── build
├── CMakeLists.txt
├── include
│   └── test.h
├── lib
│   └── libaddCode.so
└── src
    └── main.cpp
```

执行CMake:

```CMake
cmake_minimum_required(VERSION 3.0)
project(Test)

file(GLOB HAND_FILE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
set(CMAKE_CXX_STANDARD 17)

add_executable(app ${SRC_FILE})
# 链接动态库
target_link_libraries(app addCode pthread) # 指定链接的库名称
```

注: <span style="color:red">链接动态库的命令应该放在`CMakeLists.txt`的最后一行 (即 生成了app后, 才可以进行链接)</span>

当可执行程序app生成之后并执行该文件，会提示有如下错误信息:
```bash
[root@localhost build]# make
[ 50%] Building CXX object CMakeFiles/app.dir/src/main.cpp.o
[100%] Linking CXX executable app
/usr/bin/ld: 找不到 -laddCode
collect2: 错误：ld 返回 1
make[2]: *** [CMakeFiles/app.dir/build.make:97：app] 错误 1
make[1]: *** [CMakeFiles/Makefile2:83：CMakeFiles/app.dir/all] 错误 2
make: *** [Makefile:91：all] 错误 2
```
这是因为可执行程序启动之后，去加载`addCode`这个动态库，但是不知道这个动态库被放到了什么位置. 所以就加载失败了，在 CMake 中可以在生成可执行程序之前，通过命令指定出要链接的动态库的位置，指定静态库位置使用的也是这个命令:

```CMake
link_directories(path)
```

故:

```CMake
cmake_minimum_required(VERSION 3.0)
project(Test)

file(GLOB HAND_FILE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
set(CMAKE_CXX_STANDARD 17)

link_directories(${PROJECT_SOURCE_DIR}/lib) # 指定链接库的位置

add_executable(app ${SRC_FILE})
# 链接动态库
target_link_libraries(app addCode pthread) # 指定链接的库名称
```

注: <span style="color:red">链接动态库的命令应该放在`CMakeLists.txt`的最后一行 (即 生成了app后, 才可以进行链接)</span>

> [!TIP]
> 温馨提示: 使用`target_link_libraries`命令就可以链接动态库，也可以链接静态库文件。

## 参考链接
### [1]
[爱编程的大丙-CMake 保姆级教程（上）](https://subingwen.cn/cmake/CMake-primer/)