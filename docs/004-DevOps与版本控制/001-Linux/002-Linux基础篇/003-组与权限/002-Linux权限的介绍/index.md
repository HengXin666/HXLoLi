# Linux权限的介绍
## 概述
当我们使用`ll`命令查询文件信息的时候，我们发现是这样的:

```Shell
[root@hxlinux ~]# ll
总用量 8
-rw-------. 1 root root 1907 12月 21 22:44 anaconda-ks.cfg
drwxr-xr-x. 3 root root   16 1月   4 22:50 dev
drwxr-xr-x. 2 root root    6 12月 22 02:14 HXBook
-rw-r--r--. 1 root root 1925 12月 21 22:53 initial-setup-ks.cfg
```

总共 $10$ 位，我们使用 $0-9$ 来描述:

**说明:**

- 第 $0$ 位确定文件类型(d,-,l,c,b)
  - `l`是软连接，相当于windows的快捷方式
  - `d`是目录，相当于windows的文件夹
  - `c`是字符设别，鼠标，键盘(/dev 目录里面查看)
  - `b`是块设备，比如说硬盘(/dev 目录里面查看)
    
- 第 $1-3$ 位确定**所有者(该文件的所有者)** 拥有该文件的权限 --User
- 第 $4-6$ 位确定**所属组, (同用户组的)** 又有该文件的权限 --Group
- 第 $7-9$ 位确定**其他用户** 拥有改文件的权限 --Other

---

$ rwx权限详解（难点）$:

rwx作用到`文件`
- `r` 代表**可读(read)** 可以读取，查看
- `w` 代表**可写(write)** 可以修改，但是*不代表可以删除改文件*，删除一个文件的前提条件是对该文件`所在的目录有写权限`，才能删除文件
- `x` 代表**可执行(execute)** 可被执行

rwx作用到`目录`
- `r` 代表可读 可以读取 **`ls`查看目录的内容**
- `w` 代表可写 对目录内 **进行创建 + 删除 + 重命名该目录**
- `x` 代表可执行 可以 **进入该目录**

实例:
```Shell 
-rw-r--r--. 1 root root 0 Aug 28 15:59 Demo1.java

# 第一个字符表示文件类型 - 指的是文件。
# rw- 文件所有者对当前文件是可读可写不可执行的权限。
# r-- 与文件拥有者同一组的用户的权限是可读不可写不可执行。
# r-- 当前文件的其他用户的权限是可读不可写不可执行。

# 1：文件：硬连接数或 目录：子目录数
# root：当前文件所属的用户
# root：当前用户所属的组
# 0： 文件大小单位是字节
# Aug 28 15:59：文件最后修改时间
# Demo1.java：文件的名称
```

## 权限的修改

`chmod`: 可以修改文件或者目录的权限。

有三种设置写法:
1. 全写

- `u`: 所有者 (1-3)
- `g`: 所有者的组的其他成员 (4-6)
- `o`: 其他组成员 (7-9)
- `a`: 全部(`ugo`) (1-9) 均修改

```Bash
chmod u=rwx,g=rw-,u=--- [文件名]
```

2. +- 权限式

```Bash
chmod u+w,g-w,u-wxr [文件名]
chmod a+rwx [文件名] # 全部设置
```

3. 数字简写

$r=4,w=2,x=1$ ; (最后以求和的值作为权限)

$rwx$ => $4+2+1=7$ 

$rw-$ => $4+2=6$

$r-x$ => $4+1=5$

$-wx$ => $2+1=3$

```Bash
chmod 751 [文件名] # 等价于 chmod u=rwx,g=rx,o=x [文件目录名]
```

方法[1]的示例:
```Shell
[root@hxlinux mm]# chmod u=rwx,g=rwx,o=rwx qwq.tar.gz 
[root@hxlinux mm]# ll
总用量 12
-rw-r--r--. 1 root root  12 1月   4 22:41 awa.txt
-rw-r--r--. 1 root root 393 1月   4 22:26 qwq.c
-rwxrwxrwx. 1 root root 421 1月   9 03:58 qwq.tar.gz
```

方法[2]的示例:
```Shell
[root@hxlinux mm]# chmod o-wrx qwq.tar.gz 
[root@hxlinux mm]# ll
总用量 12
-rw-r--r--. 1 root root  12 1月   4 22:41 awa.txt
-rw-r--r--. 1 root root 393 1月   4 22:26 qwq.c
-rwxrwx---. 1 root root 421 1月   9 03:58 qwq.tar.gz
```

方法[3]的示例:
```Shell
[root@hxlinux mm]# chmod 711 qwq.tar.gz 
[root@hxlinux mm]# ll
总用量 12
-rw-r--r--. 1 root root  12 1月   4 22:41 awa.txt
-rw-r--r--. 1 root root 393 1月   4 22:26 qwq.c
-rwx--x--x. 1 root root 421 1月   9 03:58 qwq.tar.gz
```