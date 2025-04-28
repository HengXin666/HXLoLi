# CMake 的使用
## 变量操作
### 追加
有时候项目中的源文件并不一定都在同一个目录中，但是这些源文件最终却需要一起进行编译来生成最终的可执行文件或者库文件。如果我们通过file命令对各个目录下的源文件进行搜索，最后还需要做一个字符串拼接的操作，关于字符串拼接可以使用set命令也可以使用list命令。

#### 使用`set`拼接
如果使用`set`进行字符串拼接，对应的命令格式如下:

```CMake
set(变量名1 ${变量名1} ${变量名2} ...)

# 等价于: string s1, s2, s3, ....;
# s1 = s1 + s2 + s3 + ...; // (原本的s1会被覆盖 == 拼接)
```

关于上面的命令其实就是将从第二个参数开始往后所有的字符串进行拼接，最后将结果存储到第一个参数中，如果第一个参数中原来有数据会对原数据就行覆盖。

#### 使用`list`拼接
如果使用`list`进行字符串拼接，对应的命令格式如下:

```CMake
list(APPEND <list> [<element> ...])
```

`list`命令的功能比`set`要强大，字符串拼接只是它的其中一个功能，所以需要在它第一个参数的位置指定出我们要做的操作，`APPEND`表示进行数据追加，后边的参数和`set`就一样了。

示例:

```CMake
set(s1 "张三")
set(s2 "李四")

list(APPEND s1 ${s2}) # s1 += ${s2}

message(${s1})
```

```bash
[root@localhost build]# cmake ..
张三李四
-- Configuring done
-- Generating done
-- Build files have been written to: /root/dev/cmake_lib_stTest/build
```

注意:

在CMake中，使用`set`命令可以创建一个`list`。一个在`list`内部是一个由分号`;`分割的一组字符串。例如，`set(var a b c d e)`命令将会创建一个`list:a;b;c;d;e`，但是最终打印变量值的时候得到的是`abcde`。

这个是底层, 主要是为了区分字符串, 比如保证在删除路径的时候, 不会 把 $S_1$ 的后半 和 $S_2$ 的前半 删除; 而是应该删除一整个子字符串, 因此需要区分, 特别是那些路径(比如: `file(GLOB SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)`), 我们会发现`message`的`SRC_FILE`是没有任何分隔的, 那么内部是怎么区分呢? 就是靠`;`!

### 字符串移除
#### list
我们在通过`file`搜索某个目录就得到了该目录下所有的源文件，但是其中有些源文件并不是我们所需要的，比如: 我们并不需要测试程序`main.cpp`

```bash
.
├── build
├── CMakeLists.txt
├── include
│   └── test.h
└── src
    ├── add.cpp
    └── main.cpp
```
此时，就需要将`main.cpp`从搜索到的数据中剔除出去，想要实现这个功能，也可以使用`list`:

```CMake
list(REMOVE_ITEM <list> <value> [<value> ...])
```
通过上面的命令原型可以看到删除和追加数据类似，只不过是第一个参数变成了`REMOVE_ITEM`。

示例:

```CMake
cmake_minimum_required(VERSION 3.0)
project(Test)

file(GLOB HAND_FILE ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
set(CMAKE_CXX_STANDARD 17)

link_directories(${PROJECT_SOURCE_DIR}/lib) # 指定链接库的位置

list(REMOVE_ITEM SRC_FILE ${PROJECT_SOURCE_DIR}/src/main.cpp) # 移除 main.cpp
add_executable(app ${SRC_FILE})

# 链接动态库
target_link_libraries(app addCode pthread) # 指定链接的库名称
```

注意: <span style="color:red">通过`file`命令搜索源文件的时候得到的是文件的绝对路径（在`list`中每个文件对应的路径都是一个`item`，并且都是**绝对路径**），那么在移除的时候也要将该文件的绝对路径指定出来才可以，否是移除操作不会成功。</span>

关于`list`命令还有其它功能，但是并不常用，在此就不一一进行举例介绍了。

基本上类似于STL, 具体可以看官方文档!

1. 获取 list 的长度。

```CMake
list(LENGTH <list> <output variable>)
```
- `LENGTH`: 子命令LENGTH用于读取列表长度
- `<list>`: 当前操作的列表
- `<output variable>`: 新创建的变量，用于存储列表的长度。

2. 读取列表中指定索引的的元素，可以指定多个索引

```CMake
list(GET <list> <element index> [<element index> ...] <output variable>)
```

- `<list>`: 当前操作的列表
- `<element index>`: 列表元素的索引
    - 从0开始编号，索引0的元素为列表中的第一个元素；
    - 索引也可以是负数，-1表示列表的最后一个元素，-2表示列表倒数第二个元素，以此类推
    - 当索引（不管是正还是负）超过列表的长度，运行会报错
- `<output variable>`: 新创建的变量，存储指定索引元素的返回结果，也是一个列表。

3. 将列表中的元素用连接符（字符串）连接起来组成一个字符串

```CMake
list (JOIN <list> <glue> <output variable>)
```
- `<list>`: 当前操作的列表
- `<glue>`: 指定的连接符（字符串）
- `<output variable>`: 新创建的变量，存储返回的字符串

4. 查找列表是否存在指定的元素，若果未找到，返回-1

```CMake
list(FIND <list> <value> <output variable>)
```
- `<list>`: 当前操作的列表
- `<value>`: 需要再列表中搜索的元素
- `<output variable>`: 新创建的变量
    - 如果列表`<list>`中存在`<value>`，那么返回`<value>`在列表中的索引
    - 如果未找到则返回-1。

5. 将元素追加到列表中

```CMake
list (APPEND <list> [<element> ...])
```

6. 在list中指定的位置插入若干元素

```CMake
list(INSERT <list> <element_index> <element> [<element> ...])
```

7. 将元素插入到列表的0索引位置

```CMake
list (PREPEND <list> [<element> ...])
```

8. 将列表中最后元素移除

```CMake
list (POP_BACK <list> [<out-var>...])
```

9. 将列表中第一个元素移除

```CMake
list (POP_FRONT <list> [<out-var>...])
```

10. 将指定的元素从列表中移除

```CMake
list (REMOVE_ITEM <list> <value> [<value> ...])
```

11. 将指定索引的元素从列表中移除

```CMake
list (REMOVE_AT <list> <index> [<index> ...])
```

12. 移除列表中的重复元素

```CMake
list (REMOVE_DUPLICATES <list>)
```

13. 列表翻转

```CMake
list(REVERSE <list>)
```

14. 列表排序

```CMake
list (SORT <list> [COMPARE <compare>] [CASE <case>] [ORDER <order>])
```
- `COMPARE`: 指定排序方法。有如下几种值可选:
    - `STRING`: 按照字母顺序进行排序，为默认的排序方法
    - `FILE_BASENAME`: 如果是一系列路径名，会使用basename进行排序
    - `NATURAL:` 使用自然数顺序排序

- `CASE`: 指明是否大小写敏感。有如下几种值可选:
    - `SENSITIVE`: 按照大小写敏感的方式进行排序，为默认值
    - `INSENSITIVE`: 按照大小写不敏感方式进行排序

- `ORDER`:指明排序的顺序。有如下几种值可选:
    - `ASCENDING`: 按照升序排列，为默认值
    - `DESCENDING`: 按照降序排列

## 参考链接
### [1]
[爱编程的大丙-CMake 保姆级教程（上）](https://subingwen.cn/cmake/CMake-primer/)