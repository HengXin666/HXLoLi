# CMake 的使用
## 在静态库中链接动态库

依旧在[在静态库中链接静态库](../010-在静态库中链接静态库/index.md)的基础上, 我们这次把`addCode.h`作为动态库, 然后让`sort.cpp`链接它!

1. 修改`addCode/CMakeLists.txt`, 使其生成的是动态库:
 
```CMake
cmake_minimum_required(VERSION 3.15)
project(demo_add)

aux_source_directory(./ SRC)         # 搜索当前目录所有源文件
include_directories(${HEAD_PATH})    # 包含头文件
set(LIBRARY_OUTPUT_PATH ${LIB_PATH}) # 设置库输出路径
add_library(${ADD_CODE_LIB} SHARED ${SRC}) # 生成动态库文件
```

2. 修改`sort/CMakeLists.txt`, 让静态库链接动态库(注: *记得`target_link_libraries`是放在cmake文件最后一行*)

```CMake
cmake_minimum_required(VERSION 3.15)
project(demo_sort)

aux_source_directory(./ SRC)         # 搜索当前目录所有源文件
include_directories(${HEAD_PATH})    # 包含头文件
set(LIBRARY_OUTPUT_PATH ${LIB_PATH}) # 设置库输出路径
link_directories(${LIB_PATH})        # 链接库的目录
add_library(${SORT_LIB} STATIC ${SRC}) # 生成静态库文件
target_link_libraries(${SORT_LIB} ${ADD_CODE_LIB}) # 链接需要的动态库 (为${SORT_LIB}静态库链接)
```

3. 直接生成即可, 结束!

## 注解
### [1]
参考链接: [CMake 保姆级教程（下）](https://subingwen.cn/cmake/CMake-advanced/)