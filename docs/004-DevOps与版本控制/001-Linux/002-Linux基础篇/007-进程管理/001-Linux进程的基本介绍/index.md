# Linux进程的基本介绍
在linux中，每个执行的程序都为一个进程，每个进程都分配了一个id号（pid，进程号）。

每个进程都可能以两种形式存在，前台和后台,所谓前台进程就是及用户在目录上可以进行操作的（占用屏幕 比如我们的top指令），后台是无法在屏幕上操作的进程。

一般系统的服务进程都是以后台进程的方式存在，而且会常驻在系统中直到关机才结束。

## 查看系统运行的进程

`ps`: 该命令是用来查看系统中哪些正在运行，以及他们的运行的状况，可以不加任何参数。(`ps`还有其他用法: [Linux进程相关指令](../002-Linux进程相关指令/index.md))

```Shell
[root@hxlinux ~]# ps
  PID TTY          TIME CMD
 3741 pts/1    00:00:00 bash
 5176 pts/1    00:00:00 ps
```

|字段|说明|
|:-:|:--|
|PID|进程识别号|
|TTV|终端机号|
|TIME|此进程所消耗cpu时间|
|CMD|正在执行命令或进程名|

我们也可以加上下面几个参数，来查看进程信息:

- `-a`: 显示终端所用的进程信息
- `-u`: 以用户的格式显示进程的信息
- `-x`: 显示后台程序运行的参数

```Shell
[root@hxlinux ~]# ps -aux | more 
USER       PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND
root         1  0.1  0.0 194208  7420 ?        Ss   16:58   0:13 /usr/lib/systemd/systemd --switched-root --system --deserialize 22
root         2  0.0  0.0      0     0 ?        S    16:58   0:00 [kthreadd]
root         4  0.0  0.0      0     0 ?        S<   16:58   0:00 [kworker/0:0H]
root         5  0.0  0.0      0     0 ?        S    16:58   0:00 [kworker/u32:0]
root         6  0.0  0.0      0     0 ?        S    16:58   0:00 [ksoftirqd/0]
root         7  0.0  0.0      0     0 ?        S    16:58   0:00 [migration/0]
root         8  0.0  0.0      0     0 ?        S    16:58   0:00 [rcu_bh]
root         9  0.0  0.0      0     0 ?        S    16:58   0:04 [rcu_sched]
root        10  0.0  0.0      0     0 ?        S<   16:58   0:00 [lru-add-drain]
```

- `USER`: 进程所属的用户名称。
- `PID`: 进程号。
- `%CPU`: 进程占用CPU的百分比。
- `%MEM`: 进程占用物理内存的百分比。
- `VSZ`: 进程占用虚拟内存的大小(KB)。
- `RSS`: 进程占用物理内存的大小(KB)。
- `STAT`: 进程状态，S-代表睡眠 R-正在运行 D-短期等待 Z-僵死进程 T-被停止的线程。
- `START`: 进程启动的时间。
- `TIME`: 进程使用CPU的时间。
- `COMMAND`: 进程启动所需要的命令和参数。
