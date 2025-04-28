# Linux解压与压缩命令
## .gz 文件

- `gzip`: 压缩文件（将文件压缩为`*.gz`的文件，原文件被压缩后不存在。）

- `gunzip`: 解压`.gz`文件(原压缩包会自动删除)

示例:
```Shell
[root@hxlinux mm]# ll
总用量 8
-rw-r--r--. 1 root root  12 1月   4 22:41 awa.txt
-rw-r--r--. 1 root root 393 1月   4 22:26 qwq.c
[root@hxlinux mm]# gzip qwq.c 
[root@hxlinux mm]# ll
总用量 8
-rw-r--r--. 1 root root  12 1月   4 22:41 awa.txt
-rw-r--r--. 1 root root 321 1月   4 22:26 qwq.c.gz
[root@hxlinux mm]# gunzip qwq.c.gz 
[root@hxlinux mm]# ll
总用量 8
-rw-r--r--. 1 root root  12 1月   4 22:41 awa.txt
-rw-r--r--. 1 root root 393 1月   4 22:26 qwq.c
```

## .zip 文件

这个功能打包发布中很有用.

- `zip`: 压缩文件

  - `-r`: 递归**压缩**，即压缩目录

- `unzip`: 解压文件

  - `-d`: 指定**解压**后文件存放方目录


```Bash
zip [压缩后文件名称] [待压缩文件名称]
```

示例:
```Shell
[root@hxlinux mm]# zip qwq.zip qwq.c 
  adding: qwq.c (deflated 24%)
[root@hxlinux mm]# ll
总用量 12
-rw-r--r--. 1 root root  12 1月   4 22:41 awa.txt
-rw-r--r--. 1 root root 393 1月   4 22:26 qwq.c
-rw-r--r--. 1 root root 457 1月   9 03:31 qwq.zip

[root@hxlinux mm]# mkdir tmp
[root@hxlinux mm]# unzip -d ./tmp qwq.zip 
Archive:  qwq.zip
  inflating: ./tmp/qwq.c  
```

## .tar.gz 文件

`tar`: 该指令是打包指令，最后打包后的文件是.tar.gz的文件。

语法格式：`tar [选项] XXX.tar.gz [打包的内容]` （功能描述：打包目录，压缩后的文件格式.tar,gz）

|选项|功能|
|:-:|:-:|
|`-c`|产生tar打包文件|
|`-v`|显示详细信息|
|`-f`|指定压缩后的文件名|
|`-z`|打包同时压缩|
|`-x`|解压tar包文件|

示例:
```Shell
[root@hxlinux mm]# tar -cvzf qwq.tar.gz qwq.c # 压缩文件
qwq.c
[root@hxlinux mm]# ll
总用量 12
-rw-r--r--. 1 root root  12 1月   4 22:41 awa.txt
-rw-r--r--. 1 root root 393 1月   4 22:26 qwq.c
-rw-r--r--. 1 root root 421 1月   9 03:58 qwq.tar.gz

[root@hxlinux mm]# tar -zxvf qwq.tar.gz -C tmp/ # 解压文件到指定目录
qwq.c
[root@hxlinux mm]# cd tmp/
[root@hxlinux tmp]# ll
总用量 4
-rw-r--r--. 1 root root 393 1月   4 22:26 qwq.c
```

注: 使用 大写参数 `-C` 可以解压到指定目录

```Bash
tar -zxvf [压缩文件] -C [解压到目标目录]
```