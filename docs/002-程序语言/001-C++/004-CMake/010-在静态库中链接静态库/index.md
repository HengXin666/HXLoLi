# CMake 的使用
## 在静态库中链接静态库
依旧是在[嵌套的CMake](../009-嵌套的CMake/index.md)的基础上, 现在修改`sort.cpp`, 让它链接`addCode.h`, 即:

```C++
#include <vector>
#include <cstdio>
#include "addCode.h"

void sort(std::vector<int>& arr) {
    printf("In sort do add Fun: 1 + 1 = %d\n", add(1, 1));

    for (int i = 0; i < arr.size(); ++i) {
        for (int j = 0; j < arr.size(); ++j) {
            if (arr[i] < arr[j]) {
                arr[i] ^= arr[j];
                arr[j] ^= arr[i];
                arr[i] ^= arr[j];
            }
        }
    }
}
```

然后也只需要一个`test`即可, 删除`test1`, 并将`test2`重命名为`test`, 然后修改`sort`目录的`CMakeLists.txt`, 因为它需要链接对应的静态库:

```CMake
# sort/CMakeLists.txt
cmake_minimum_required(VERSION 3.15)
project(demo_sort)

aux_source_directory(./ SRC)         # 搜索当前目录所有源文件
include_directories(${HEAD_PATH})    # 包含头文件
set(LIBRARY_OUTPUT_PATH ${LIB_PATH}) # 设置静态库输出路径
link_directories(${LIB_PATH})        # 链接库的目录 [添加]
link_libraries(${ADD_CODE_LIB})      # 链接需要的库 [添加]
add_library(${SORT_LIB} STATIC ${SRC}) # 生成静态库文件
```

然后不要忘记把根目录的`CMakeLists.txt`有关的部分也修改一下, 然后就OK了!

## 注解
### [1]
参考链接: [CMake 保姆级教程（下）](https://subingwen.cn/cmake/CMake-advanced/)