# 设置主机名和host映射
为了方便记忆，我们可以给linux系统设置主机名，也可以根据需要修改主机名。

我们可以通过`hostname`查看主机名的名称。

```Shell
[root@hxlinux ~]# hostname
hxlinux
```

也可以修改`/etc/hostname`指定主机名名称。注意，修改完成之后，需要**重启**linux系统才能生效。

思考：前面我们可以通过ping linux的ip地址能ping通linux。那么我们可不可以通过ping linux的主机名来ping通linux呢? 答案是不可以的。

| ##container## |
|:--:|
|![Clip_2024-01-10_17-21-48.png ##w500##](./Clip_2024-01-10_17-21-48.png)|

所以我们需要**将主机名称和ip地址进行映射**，如何映射? 我们需要修改windows的hosts文件。找到`C:\Windows\System32\drivers\etc`下面的`hosts`文件，进行相关的编辑:

添加如下内容在新的一行.
```
192.168.213.66 hxlinux # ip地址是linux的ip地址
```

保存并退出后, 就可以在windows里面ping 了:
```CMD
C:\Users\Heng_Xin>ping hxlinux

正在 Ping hxlinux [192.168.213.66] 具有 32 字节的数据:
来自 192.168.213.66 的回复: 字节=32 时间<1ms TTL=64
来自 192.168.213.66 的回复: 字节=32 时间<1ms TTL=64
来自 192.168.213.66 的回复: 字节=32 时间<1ms TTL=64
来自 192.168.213.66 的回复: 字节=32 时间=1ms TTL=64

192.168.213.66 的 Ping 统计信息:
    数据包: 已发送 = 4，已接收 = 4，丢失 = 0 (0% 丢失)，
往返行程的估计时间(以毫秒为单位):
    最短 = 0ms，最长 = 1ms，平均 = 0ms
```

我们如何在linux里面通过本机的主机名来ping通主机呢？我们需要编辑`/etc/hosts`文件:

```Shell
vim /etc/hosts
```

```编辑
127.0.0.1   localhost localhost.localdomain localhost4 localhost4.localdomain4
::1         localhost localhost.localdomain localhost6 localhost6.localdomain6
192.168.213.1 hx_win # ip是vmnet8的ip地址 后面的是windows的主机名称
```

```Shell
[root@hxlinux ~]# ping hx_win
PING hx_win (192.168.213.1) 56(84) bytes of data.
64 bytes from hx_win (192.168.213.1): icmp_seq=1 ttl=128 time=0.740 ms
64 bytes from hx_win (192.168.213.1): icmp_seq=2 ttl=128 time=0.914 ms
64 bytes from hx_win (192.168.213.1): icmp_seq=3 ttl=128 time=0.666 ms
64 bytes from hx_win (192.168.213.1): icmp_seq=4 ttl=128 time=0.813 ms
64 bytes from hx_win (192.168.213.1): icmp_seq=5 ttl=128 time=0.665 ms
^C
--- hx_win ping statistics ---
5 packets transmitted, 5 received, 0% packet loss, time 4006ms
rtt min/avg/max/mdev = 0.665/0.759/0.914/0.099 ms
```

*注意：如果ping不通，需要关闭windows的防火墙。*

思考：为什么通过主机名(域名)就能找到对应的ip地址呢，接下来我们就分析一下主机名(域名)解析机制：

应用实例：用户在浏览器输入 www.baidu.com 如何找到百度服务器地址的?

1. 浏览器先检查浏览器缓存中有没有该域名解析ip地址，有就先调用这个ip完成域名解析。如果没有就检查DNS解析器缓存，如果有就直接返回ip完成解析。这两个缓存可以理解为**本地解析器缓存**。

2. 如果本地解析器缓存没有找到对应的映射，再去检查系统中的**hosts文件**中有没有配置对应的域名ip映射，如果有，就完成域名解析。

3. 如果本地DNS缓存和hosts文件中均没有找到对应的ip，则到**DNS域名服务器**完成域名解析。

4. 如果公网的DNS域名解析器也没有完成域名解析，就返回资源**找不到**的信息。