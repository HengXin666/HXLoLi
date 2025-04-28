# CMake 的使用
## 搜索文件
如果一个项目里边的源文件很多，在编写`CMakeLists.txt`文件的时候不可能将项目目录的各个文件一一罗列出来，这样太麻烦也不现实。所以，在CMake中为我们提供了搜索文件的命令，可以使用`aux_source_directory`命令或者`file`命令。

### aux_source_directory 查询某路径的`所有源文件`

```CMake
aux_source_directory(< dir > < variable >)
```

- **dir**: 要搜索的目录
- **variable**: 将从dir目录下搜索到的源文件列表存储到该变量中

注: *并不支持递归搜索*, 得到的结果是**追加**到变量中, *而不是覆盖*.

示例:

```CMake
cmake_minimum_required(VERSION 3.0)
project(Test)
# set(CODE_SRC main.cpp add.cpp)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} CODE_SRC) # 从 cmake 命令执行指定的目录寻找所有的目标文件(不能使用 *.cpp 进行过滤)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ./DEBUG)
add_executable(app ${CODE_SRC})
```

### CMAKE_CURRENT_SOURCE_DIR 宏
表示一个路径, 该路径为执行`cmake`指令时候指定的路径, 如:

```bash
[root@localhost build]# cmake ..
```

此时`CMAKE_CURRENT_SOURCE_DIR`=`..`

### file 搜索指定类型文件
如果一个项目里边的源文件很多，在编写`CMakeLists.txt`文件的时候不可能将项目目录的各个文件一一罗列出来，这样太麻烦了。所以，在CMake中为我们提供了搜索文件的命令，他就是`file`（当然，除了搜索以外通过 file 还可以做其他事情）。

```CMake
file(GLOB/GLOB_RECURSE 变量名 要搜索的文件路径和文件类型)
```
- **GLOB**: 将指定目录下搜索到的满足条件的所有文件名生成一个列表，并将其存储到变量中。
- **GLOB_RECURSE**: **递归**搜索指定目录，将搜索到的满足条件的文件名生成一个列表，并将其存储到变量中。

注: <b style="color:red">得到的结果会覆盖原变量, 而不是追加!</b>

因此需要使用`set`进行合并:

```CMake
set(a ${a} ${b})
```

示例:

```CMake
file(GLOB MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB MAIN_HEAD ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

# 关于要搜索的文件路径和类型可加双引号，也可不加:
file(GLOB MAIN_HEAD "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")
```

## 参考链接
### [1]
[爱编程的大丙-CMake 保姆级教程（上）](https://subingwen.cn/cmake/CMake-primer/)