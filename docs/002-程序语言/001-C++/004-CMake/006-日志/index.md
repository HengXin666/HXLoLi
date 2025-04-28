# CMake 的使用
## 日志
在CMake中可以用用户显示一条消息，该命令的名字为`message`:

```CMake
message([STATUS|WARNING|AUTHOR_WARNING|FATAL_ERROR|SEND_ERROR] "message to display" ...)
```

- **(无)**: 重要消息
- **STATUS**: 非重要消息
- **WARNING**: CMake 警告, 会继续执行
- **AUTHOR_WARNING**: CMake 警告 (dev), 会继续执行
- **SEND_ERROR**: CMake 错误, 继续执行，但是会跳过生成的步骤
- **FATAL_ERROR**: CMake 错误, 终止所有处理过程

CMake的命令行工具会在stdout上显示STATUS消息，在stderr上显示其他所有消息。CMake的GUI会在它的log区域显示所有消息。

CMake警告和错误消息的文本显示使用的是一种简单的标记语言。文本没有缩进，超过长度的行会回卷，段落之间以新行做为分隔符。

```CMake
# 输出一般日志信息
message(STATUS "source path: ${PROJECT_SOURCE_DIR}")
# 输出警告信息
message(WARNING "source path: ${PROJECT_SOURCE_DIR}")
# 输出错误信息
message(FATAL_ERROR "source path: ${PROJECT_SOURCE_DIR}")
```

示例:

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

message("Hello\tCMake!")
# 输出一般日志信息
message(STATUS "source path: ${PROJECT_SOURCE_DIR}")
# 输出警告信息
message(WARNING "source path: ${PROJECT_SOURCE_DIR}")
# 输出错误信息
message(FATAL_ERROR ${SRC_FILE})
```

```bash
[root@localhost build]# cmake ..
Hello    CMake!
-- source path: /root/dev/cmake_lib_stTest
CMake Warning at CMakeLists.txt:20 (message):
  source path: /root/dev/cmake_lib_stTest


CMake Error at CMakeLists.txt:22 (message):
  /root/dev/cmake_lib_stTest/src/main.cpp


-- Configuring incomplete, errors occurred!
```

可以发现, 寻找到的文件(`${SRC_FILE}`)等, 使用的是绝对路径!

## 参考链接
### [1]
[爱编程的大丙-CMake 保姆级教程（上）](https://subingwen.cn/cmake/CMake-primer/)