# CMake 的使用
## 制作动态库或静态库
有些时候我们编写的源代码并不需要将他们编译生成可执行程序，而是生成一些静态库或动态库提供给第三方使用，下面来讲解在cmake中生成这两类库文件的方法。

### 制作静态库
在cmake中，如果要制作静态库，需要使用的命令如下:

```CMake
add_library(库名称 STATIC 源文件1 [源文件2] ...) 
```

在Linux中，静态库名字分为三部分: <span style="color:red">lib</span> + <span style="color:red">库名字</span> + <span style="color:red">.a</span>，此处只需要指定出库的名字就可以了，另外两部分在生成该文件的时候会自动填充。

在Windows中虽然库名和Linux格式不同，但也**只需指定出名字**即可。

示例:

```CMake
cmake_minimum_required(VERSION 3.0)
project(Test)

file(GLOB HAND_FILE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)


set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ./DEBUG)
# add_executable(app ${HAND_FILE} ${SRC_FILE})
add_library(addCode STATIC ${SRC_FILE}) # 生成对应静态库文件
```


```bash
[root@localhost build]# make
[ 50%] Building CXX object CMakeFiles/addCode.dir/src/add.cpp.o
[100%] Linking CXX static library libaddCode.a
[100%] Built target addCode
[root@localhost build]# ll
总用量 32
-rw-r--r-- 1 root root 13760 4月  16 15:32 CMakeCache.txt
drwxr-xr-x 5 root root   232 4月  16 15:32 CMakeFiles
-rw-r--r-- 1 root root  1624 4月  16 15:32 cmake_install.cmake
-rw-r--r-- 1 root root  1394 4月  16 15:32 libaddCode.a
-rw-r--r-- 1 root root  5164 4月  16 15:32 Makefile
```

### 制作动态库
在cmake中，如果要制作动态库，需要使用的命令如下:

```CMake
add_library(库名称 SHARED 源文件1 [源文件2] ...) 
```

在Linux中，动态库名字分为三部分: <span style="color:red">lib</span> + <span style="color:red">库名字</span> + <span style="color:red">.so</span>，此处只需要指定出库的名字就可以了，另外两部分在生成该文件的时候会自动填充。

在Windows中虽然库名和Linux格式不同，但也只需指定出名字即可。

示例:

```CMake
cmake_minimum_required(VERSION 3.0)
project(Test)

file(GLOB HAND_FILE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)


set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ./DEBUG)
add_library(addCode SHARED ${SRC_FILE})
```

```bash
[root@localhost build]# make
[ 50%] Building CXX object CMakeFiles/addCode.dir/src/add.cpp.o
[100%] Linking CXX shared library libaddCode.so
[100%] Built target addCode
[root@localhost build]# ll
总用量 36
-rw-r--r-- 1 root root 13760 4月  16 15:35 CMakeCache.txt
drwxr-xr-x 5 root root   232 4月  16 15:35 CMakeFiles
-rw-r--r-- 1 root root  1624 4月  16 15:35 cmake_install.cmake
-rwxr-xr-x 1 root root  8072 4月  16 15:35 libaddCode.so
-rw-r--r-- 1 root root  5164 4月  16 15:35 Makefile
```

值得注意的是, 动态库是有可执行权限的(并且也是需要这个权限的)

## 指定输出的路径
### 方式1 - 适用于动态库
对于生成的库文件来说和可执行程序一样都可以指定输出路径。**由于在Linux下生成的动态库默认是有执行权限的**，所以可以按照生成可执行程序的方式去指定它生成的目录:

```CMake
cmake_minimum_required(VERSION 3.0)
project(CALC)
include_directories(${PROJECT_SOURCE_DIR}/include)
file(GLOB SRC_LIST "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
# 设置动态库生成路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(calc SHARED ${SRC_LIST})
```
对于这种方式来说，其实就是通过`set`命令给`EXECUTABLE_OUTPUT_PATH`宏设置了一个路径，这个路径就是可执行文件生成的路径。

~~*不过我在我的Linux里面测试了, 它并没有生成啊...*~~

### 方式2 - 都适用
由于在Linux下生成的静态库默认不具有可执行权限，所以在指定静态库生成的路径的时候就不能使用`EXECUTABLE_OUTPUT_PATH`宏了，而应该使用`LIBRARY_OUTPUT_PATH`，**这个宏对应静态库文件和动态库文件都适用**。



```CMake
cmake_minimum_required(VERSION 3.0)
project(Test)

file(GLOB HAND_FILE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)


set(CMAKE_CXX_STANDARD 17)
# 设置库的生成路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(addCode SHARED ${SRC_FILE})
# add_library(addCode STATIC ${SRC_FILE})
```

```bash
[root@localhost build]# make
[ 50%] Building CXX object CMakeFiles/addCode.dir/src/add.cpp.o
[100%] Linking CXX shared library ../lib/libaddCode.so
[100%] Built target addCode
```

## 参考链接
### [1]
[爱编程的大丙-CMake 保姆级教程（上）](https://subingwen.cn/cmake/CMake-primer/)