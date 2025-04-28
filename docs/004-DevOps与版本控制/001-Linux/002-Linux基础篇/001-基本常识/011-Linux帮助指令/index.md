# Linux帮助指令
## man 获得帮助信息

```Bash
man [指令名称]
```

示例: 查询 `ls` 使用方法
```Shell
[root@hxlinux /]# man ls


LS(1)                                            General Commands Manual                                            LS(1)

NAME
       ls, dir, vdir - 列目录内容

提要
       ls [选项] [文件名...]

       POSIX 标准选项: [-CFRacdilqrtu1]

GNU 选项 (短格式):
       [-1abcdfgiklmnopqrstuxABCDFGLNQRSUX]  [-w  cols] [-T cols] [-I pattern] [--full-time] [--format={long,verbose,com‐
       mas,across,vertical,single-column}]  [--sort={none,time,size,extension}]  [--time={atime,access,use,ctime,status}]
       [--color[={none,auto,always}]] [--help] [--version] [--]

描述（ DESCRIPTION ）
       程序ls先列出非目录的文件项，然后是每一个目录中的“可显示”文件。如果
       没有选项之外的参数【译注：即文件名部分为空】出现，缺省为        "."        （当前目录）。         选项“         -d
       ”使得目录与非目录项同样对待。除非“ -a ” 选项出现，文 件名以“.”开始的文件不属“可显示”文件。

       以当前目录为准，每一组文件（包括非目录文件项，以及每一内含文件的目录）分      别按文件名比较顺序排序。如果“     -l
       ”选项存在，每组文件前显示一摘要行: 给出该组文件长度之和（以 512 字节为单位）。

       输出是到标准输出（             stdout             ）。除非以“             -C             ”选项要求按多列输出，输出
       将是一行一个。然而，输出到终端时，单列输出或多列输出是不确定的。可以分别      用选项“      -1     ”     或“     -C
       ”来强制按单列或多列输出。

       -C     多列输出，纵向排序。

       -F     每个目录名加“ / ”后缀，每个 FIFO 名加“ | ”后缀， 每个可运行名加“ * ”后缀。

       -R     递归列出遇到的子目录。

       -a     列出所有文件，包括以 "." 开头的隐含文件。

       -c     使用“状态改变时间”代替“文件修改时间”为依据来排序 （使用“ -t ”选项时）或列出（使用“ -l ”选项时）。

       -d     将目录名象其它文件一样列出，而不是列出它们的内容。
```

注: `ls -l` 的简写是 `ll`

参数可以组合, 并且顺序不分先后:

`ls -al` == `ls -la` // 列出当前目录文件/文件夹信息, 并且包含隐藏文件

## help 获得shell内置命令的帮助信息

```Bash
help [命令名]
```

示例:
```Shell
[root@hxlinux /]# help cd
cd: cd [-L|[-P [-e]]] [dir]
    Change the shell working directory.
    
    Change the current directory to DIR.  The default DIR is the value of the
    HOME shell variable.
    
    The variable CDPATH defines the search path for the directory containing
    DIR.  Alternative directory names in CDPATH are separated by a colon (:).
    A null directory name is the same as the current directory.  If DIR begins
    with a slash (/), then CDPATH is not used.
    
    If the directory is not found, and the shell option `cdable_vars' is set,
    the word is assumed to be  a variable name.  If that variable has a value,
    its value is used for DIR.
    
    Options:
        -L    force symbolic links to be followed
        -P    use the physical directory structure without following symbolic
        links
        -e    if the -P option is supplied, and the current working directory
        cannot be determined successfully, exit with a non-zero status
    
    The default is to follow symbolic links, as if `-L' were specified.
    
    Exit Status:
    Returns 0 if the directory is changed, and if $PWD is set successfully when
    -P is used; non-zero otherwise.
```