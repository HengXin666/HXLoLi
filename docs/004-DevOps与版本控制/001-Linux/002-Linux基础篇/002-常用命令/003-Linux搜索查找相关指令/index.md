# Linux搜索查找相关指令
## find
`find`: 将从指定目录下递归地 $遍历$ 各个目录，将所有满足条件的目录显示在控制台。

> [缺点]: 如果从 根目录 `/` 开始搜索的话, 可能会比较慢.

|选项|功能|
|:-:|:-:|
|`-name`|按照文件的名称查找文件|
|`-user`|查找指定用户所属的文件|
|`-size`|按照指定的大小查找文件|

语法:

```Bash
find [开始目录] [选项] [目标文件/用户/大小]

# 详细解释（示例）:
# -name
find / -name loli.c   # 从根目录开始查找名字为 loli.c 的文件
find / -name '*.text' # 支持通配符 * 与 ?

# -user
find ./ -user 'root'  # 从当前目录开始查找 root用户所属的文件

# -size
find ./ -size +100M   # 从当前目录开始查找 大于 100M 的文件
find ./ -size -100M   # 从当前目录开始查找 小于 100M 的文件
find ./ -size 100M    # 从当前目录开始查找 等于 100M 的文件
# 值得注意的是，文件大小单位默认是块（block），而非字节。
# 如果想要按照字节计算，可以在数字后面加上 c。
# 此外，还可以使用 k（千字节）、M（兆字节）和 G（吉字节）等后缀来指定不同的单位
```

## locate
`locate`: 该指令可以快速定位文件路径。`locate`指令利用事先建立好的系统中所有文件名称及路径的`locate数据库`实现快速定位给定的文件。**locate指令无需遍历整个文件系统，查询速度较快**。

> [!TIP]
> **特别说明**:
> 
> 由于`locate`指令基于数据库进行查询。所以第一次查询运行前，必须使用`updatedb`指令创建`locate数据库`。

示例:
```Shell
[root@hxlinux ~]# updatedb
[root@hxlinux ~]# locate qwq.c
/root/dev/mm/qwq.c
```

## which
`which`: 该指令可以**查看某个指令在哪个目录下**.

示例:
```Shell
[root@hxlinux ~]# which ls
alias ls='ls --color=auto'
    /usr/bin/ls
[root@hxlinux ~]# which cd
/usr/bin/cd
```

## grep
`grep`: **过滤查找**，管道符，”|“,表示前一个指令的处理结果输出传递给后面的指令处理。**一般我们将 `|` 和`grep`一起结合起来使用**。

|选项|功能|
|:-:|:-:|
|`-n`|显示行号|
|`-i`|忽略自动大小写|

语法:
```Bash
grep [选项] [查找内容]
```

示例:
```Shell
[root@hxlinux mm]# cat qwq.c | grep -ni heLLO
30:    printf("Hello C!\n");
```