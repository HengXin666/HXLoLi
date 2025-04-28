# CMake 的使用
## 包含头文件
### 情景引入
你现在手着一个大型项目, 其文件树如下:

```bash
[root@localhost cmake_test]# tree
.
├── build
├── CMakeLists.txt
├── include
│   └── test.h
└── src
    ├── add.cpp
    └── main.cpp
```

(main.cpp add.cpp 依赖 test.h, 而他们不在同一个路径下)
```bash
[root@localhost build]# make
[ 33%] Building CXX object CMakeFiles/app.dir/src/add.cpp.o
/root/dev/cmake_test/src/add.cpp:1:10: 致命错误：test.h：没有那个文件或目录
 #include "test.h"
          ^~~~~~~~
编译中断。
make[2]: *** [CMakeFiles/app.dir/build.make:76：CMakeFiles/app.dir/src/add.cpp.o] 错误 1
make[1]: *** [CMakeFiles/Makefile2:83：CMakeFiles/app.dir/all] 错误 2
make: *** [Makefile:91：all] 错误 2
```
那么现在就有两种方案:
1. 对文件的`#include`的路径进行修改, 如: 修改为`#include "../include/test.h"`, (如果有1w个文件都这样, 我会直接跑路!)

2. 使用 cmake 编写: 包含头文件

### include_directories 包含头文件
在编译项目源文件的时候，很多时候都需要将源文件对应的头文件路径指定出来，这样才能保证在编译过程中编译器能够找到这些头文件，并顺利通过编译。在CMake中设置要包含的目录也很简单，通过一个命令就可以搞定了，他就是`include_directories`:

```CMake
include_directories(headpath)
```

例如:
```CMake
cmake_minimum_required(VERSION 3.0)
project(Test)

file(GLOB HAND_FILE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

# include_directories(${HAND_FILE}) 错误的! 不应该是具体的文件, 而应该是文件夹(路径)
include_directories(${PROJECT_SOURCE_DIR}/include)


set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ./DEBUG)
add_executable(app ${HAND_FILE} ${SRC_FILE})
```

注意: *`include_directories`的内容不应该是具体的文件, 而应该是文件夹(路径)*

### PROJECT_SOURCE_DIR 宏

`PROJECT_SOURCE_DIR`宏对应的值就是我们在使用`cmake`命令时，后面紧跟的目录，一般是工程的根目录。

---

`PROJECT_SOURCE_DIR`是CMake内置变量之一，它代表**当前项目根目录的路径**。当使用project命令指定项目名称时，CMake会自动设置PROJECT_SOURCE_DIR的默认值为项目根目录的路径（CMakeLists.txt所在的路径）。如果你使用的是子目录CMakeLists.txt，那么PROJECT_SOURCE_DIR将会是子目录的路径。<sup>[[CMake PROJECT_SOURCE_DIR变量](https://blog.csdn.net/Dontla/article/details/129264798)]</sup>

## 参考链接
### [1]
[爱编程的大丙-CMake 保姆级教程（上）](https://subingwen.cn/cmake/CMake-primer/)