# Linux定时任务调度
## crond任务调度快速入门

我可以使用crontab命令进行定时任务调度的设置。

> 何为任务调度?
>
> 任务调度：指在某个时间执行特定的命令或者程序。

任务调度的分类：

1. **系统任务**：有些重要的工作必须周而复始的执行，比如病毒扫描。
2. **个别用户工作**: 个别用户希望执行某些程序，比如对mysql数据库的备份。

### crontab

语法:

```Bash
crontab [选项]
```

常用选项:

- `-e`: 编辑定时任务.
- `-l`: 查询定时任务.
- `-r`: 删除当前用户所有定时任务

示例:

进入当前用户定时任务编辑页面.
```Shell
[root@hxlinux ~]# crontab -e
```

设置一个定时任务, 每分钟在这个路径保存当前时间.<sup>[1]</sup>
```Shell
*/1 * * * * date >> /root/dev/time.txt
```

尝试查看: (文件不存在也会自己创建)
```Shell
[root@hxlinux dev]# ll # 之前
总用量 0
drwxr-xr-x. 2 root root 52 1月   9 04:36 mm
[root@hxlinux dev]# date
2024年 01月 10日 星期三 03:53:39 CST
[root@hxlinux dev]# date
2024年 01月 10日 星期三 03:54:03 CST
[root@hxlinux dev]# ll # 一分钟后
总用量 4
drwxr-xr-x. 2 root root 52 1月   9 04:36 mm
-rw-r--r--. 1 root root 43 1月  10 03:54 time.txt
[root@hxlinux dev]# cat time.txt 
2024年 01月 10日 星期三 03:54:01 CST

# 查看当前任务 && 删除任务
[root@hxlinux dev]# crontab -l
*/1 * * * * date >> /root/dev/time.txt
[root@hxlinux dev]# crontab -r
[root@hxlinux dev]# crontab -l
no crontab for root
```

### [1] 占位符说明

占位符说明:
|项目|含义|范围|
|:-:|:-:|:-:|
|第一个 `“*”`|一个小时当中的第几分钟|0-59|
|第二个 `“*”`|一天当中的第几个小时|0-23|
|第三个 `“*”`|一个月当中的第几天|1-31|
|第四个 `“*”`|一年当中的第几个月|1-12|
|第五个 `“*”`|一周当中的星期几|0-7 (0和7都代表星期日)|

特殊符号说明:
|特殊符号|含义|
|:-:|:-:|
|`*`|代表任何时间，比如第一个“*”就代表1小时中每分钟都执行1次的意思。
|`,`|代表不连续的时间，比如“8,12,16 * * *”就代表1天中的8点过0分，12点过0分，16点过0分都会执行1次。
|`-`|代表连续的世间范围，比如“0,5 * * 1-6”就代表星期1到星期6的凌晨5点过0分都会执行。
|`*/n`| 代表每隔多久执行1次。比如“*/10 * * * *” 就代表每隔10分钟就执行1次命令。

### 定时执行脚本

示例:
```Shell
# -------------------- 编写 .sh 脚本 --------------- #
[root@hxlinux dev]# vim sikoto.sh
[root@hxlinux dev]# cat sikoto.sh 
date >> /root/dev/awa.txt
cal >> /root/dev/awa.txt

# -------------------- 给予用户可执行权限(x)(root 也要)#
[root@hxlinux dev]# chmod u+x sikoto.sh 

# -------------------- 添加到定时任务中 ------------- #
[root@hxlinux dev]# crontab -e
no crontab for root - using an empty one
crontab: installing new crontab
[root@hxlinux dev]# crontab -l
*/1 * * * * /root/dev/sikoto.sh

# -------------------- 查看 .sh 脚本是否被定时执行 --- #
[root@hxlinux dev]# ll
总用量 4
drwxr-xr-x. 2 root root 52 1月   9 04:36 mm
-rwxr--r--. 1 root root 51 1月  10 04:05 sikoto.sh
[root@hxlinux dev]# date
2024年 01月 10日 星期三 04:07:57 CST
[root@hxlinux dev]# date
2024年 01月 10日 星期三 04:08:01 CST
[root@hxlinux dev]# ll
总用量 8
-rw-r--r--. 1 root root 191 1月  10 04:08 awa.txt
drwxr-xr-x. 2 root root  52 1月   9 04:36 mm
-rwxr--r--. 1 root root  51 1月  10 04:05 sikoto.sh
[root@hxlinux dev]# cat awa.txt 
2024年 01月 10日 星期三 04:08:01 CST
      一月 2024     
日 一 二 三 四 五 六
    1  2  3  4  5  6
 7  8  9 10 11 12 13
14 15 16 17 18 19 20
21 22 23 24 25 26 27
28 29 30 31
```

## at任务调度机制的简单介绍
1. at命令是一次性定时执行任务计划，at的守护线程atd以后台的模式运行，检查作业队列来运行。
2. 默认情况下，atd守护线程每60秒检查作业队列，有作业时会检查作业运行时间，(遍历整个队列~~(为什么不称它做列表(list)?)~~)如果时间与当前时间匹配，则运行此作业。
3. at命令是一次性定制的计划任务，**执行完一个任务后就不再执行此任务**了。
4. 在使用at命令的时候，一定要保证atd进程的启动，可以用相关指令来查看:

`ps -ef | grep atd`
```Shell
[root@hxlinux dev]# ps -ef | grep atd
root      1808     1  0 03:38 ?        00:00:00 /usr/sbin/atd -f
root      5545  2582  0 04:29 pts/0    00:00:00 grep --color=auto atd
```

at时间定义:
|格式|含义|举例|
|:-:|:-:|:-:|
|HH:MM|当天 HH:MM 执行, 如果当天时间已过, 则在明天 HH:MM 执行|当天 4:00 (若超时则为明天 4:00)|
|英文粗略时间单词|midnight(午夜，00:00)、noon(中午，12:00)、teatime(下午茶时间，16:00)、tomorrow（明天)|midnight、noon、teatime|
|英文月名A 日期B [年份C]|C年A月B日执行|在 2018 年 1 月 15 日执行:<br>January 15 2018|
|日期时间<br>戳形式|绝对计时法<br>时间+日期<br>时间：HH:MM<br>日期：MMDDYY或MM/DD/YY或MM.DD.YY|在 2018 年 1 月 15 日执行:<br>011518或01/15/18或01.15.18|
|now + 数量 单位|相对计时法<br>以 minutes、hours、days 或 weeks 为单位|5 天后的此时此刻执行:<br>now + 5 days|

### at
语法:
```Bash
at [选项] [时间]
at> 命令 (输入两次 Ctrl + D)
```

> [!TIP]
> 释义：
>
> 第一行：at 指令输入结束后，回车到下一行输入指令
>
> 第二行：开头的 at> 无需输入，是系统自动添加的
>
> 命令输入结束后：Ctrl + D 结束命令的输入，要输入两次

常用选项:
- `-m`: 当前任务执行后，向用户发送邮件
- `-l` (= `atq` 指令)**list**: 列出当前用户的 at 任务队列
- `-d` (= `atrm` 指令)**delete**: 删除 at 任务-V显示任务的将被执行的时间
- `-c`: 输出任务内容 (任务指令)
- `-V`: 显示版本信息
- `-f <文件>`: 从指定的文件读入，而不是从标准输入
- `-t <时间参数>`: 以时间参数的形式提交要运行的任务，时间参数 `MMDDhhmm` (月日时分)

示例:
```Shell
[root@hxlinux dev]# at now + 1minutes
at> ll

[root@hxlinux dev]# atq
1    Wed Jan 10 04:46:00 2024 a root
```

### atq
`atq`: 查看当前任务调度.

### atrm
`atrm`: 删除当前任务队列的某个任务.

```Bash
atrm [索引]
```

示例:

```Shell
[[root@hxlinux dev]# atq
1 Wed Jan 10 04:48:00 2024 a root
2 Wed Jan 10 04:51:00 2024 a root
4 Wed Jan 10 04:52:00 2024 a root

[[root@hxlinux dev]# atrm 1 # 删除1号任务
[[root@hxlinux dev]# atq
2 Wed Jan 10 04:51:00 2024 a root
4 Wed Jan 10 04:52:00 2024 a root
```