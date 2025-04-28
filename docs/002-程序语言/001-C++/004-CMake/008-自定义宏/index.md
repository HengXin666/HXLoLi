# CMake 的使用
## 自定义宏

在进行程序测试的时候，我们可以在代码中添加一些宏定义，通过这些宏来控制这些代码是否生效，如下所示:
```C++
#include <cstdio>
#include "test.h"

int main() {

#ifdef DEBUG
    printf("AAO!\n");
#endif

    printf("%d + %d = %d\n", 1, 2, add(1, 2));
    return 0;
}
```

上面代码, 我们可以通过定义一个`DEBUG`宏, 来查看测试日志; 而如果去掉它, 就相当于被注释了, 实际上的代码不会编译它.

为了让测试更灵活，我们可以不在代码中定义这个宏，而是在测试的时候去把它定义出来，其中一种方式就是在`gcc/g++`命令中去指定，如下:

```bash
$ gcc test.c -DDEBUG -o app
```

在`gcc/g++`命令中通过参数`-D`指定出要定义的宏的名字，这样就相当于在代码中定义了一个宏，其名字为`DEBUG`。

### add_definitions
在CMake中我们也可以做类似的事情，对应的命令叫做`add_definitions`:

```CMake
add_definitions(-D宏名称)
```

示例:

```CMake
cmake_minimum_required(VERSION 3.0)
project(Test)

file(GLOB HAND_FILE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_definitions(-DDEBUG) # 定义DEBUG宏, 用于日志输出

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ./DEBUG)
add_executable(app ${HAND_FILE} ${SRC_FILE})
```

输出:
```bash
[root@localhost DEBUG]# ./app
AAO!
1 + 2 = 3
```

## 参考链接
### [1]
[爱编程的大丙-CMake 保姆级教程（上）](https://subingwen.cn/cmake/CMake-primer/)