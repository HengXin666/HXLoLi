# CMake 的使用
## 在程序中链接静态库
我们之前制作了一个`addCode`的静态库, 今天我们尝试使用它吧 **owo**

准备如下内容
```bash
.
├── build
├── CMakeLists.txt
├── include
│   └── test.h
├── lib
│   └── libaddCode.a
└── src
    └── main.cpp
```

在cmake中，链接静态库的命令如下:

```CMake
link_libraries(<static lib> [<static lib>...])
```
- 参数1: 指定出要链接的静态库的名字
    - 可以是全名 `libxxx.a`
    - 也可以是掐头(lib) 去尾(.a) 之后的名字 `xxx`
- 参数2~N: 要链接的其它静态库的名字

如果该静态库不是系统提供的（自己制作或者使用第三方提供的静态库）可能出现静态库找不到的情况，此时可以将静态库的路径也指定出来:

```CMake
link_directories(<lib path>) # 同include_directories, 需要的是路径而不是单个文件
```

示例
```CMake
cmake_minimum_required(VERSION 3.0)
project(Test)

file(GLOB HAND_FILE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
set(CMAKE_CXX_STANDARD 17)

# 链接静态库
link_directories(${PROJECT_SOURCE_DIR}/lib) # 指定链接库的位置
link_libraries(addCode) # 指定链接的库名称

add_executable(app ${SRC_FILE})
```

值得注意的是, **链接的静态库的代码会一并打包到可执行程序中**.

## 参考链接
### [1]
[爱编程的大丙-CMake 保姆级教程（上）](https://subingwen.cn/cmake/CMake-primer/)