# Linux 日志轮替
日志轮替就是按照一定的规则，将一些不需要的旧的文件删掉。

日志轮替，我们使用了`/etc/logrotate.conf`这个配置文件进行管理的。

```bash
[root@localhost log]# cat /etc/logrotate.conf 
# see "man logrotate" for details
# rotate log files weekly
weekly

# keep 4 weeks worth of backlogs
rotate 4

# create new (empty) log files after rotating old ones
create

# use date as a suffix of the rotated file
dateext

# uncomment this if you want your log files compressed
#compress

# RPM packages drop log rotation information into this directory
include /etc/logrotate.d

# system-specific logs may be also be configured here.
```

这里有几个重要的默认参数:
- weekly: 表示每周轮替一次
- rotate 4: 表示同一个日志文件最多保存四个版本 多了会删除
- create: 产生轮替之后生成一个新的空白的文件放在其后
- dateext: 日志轮替文件名字的命名方式
    - 如果配置文件中有dateext参数:日志会用日期作为日志文件的后缀，例如“message-20220801”。
    - 如果没用dateext:日志需要进行改名，当第一次日志轮替时，当前的“secure”改名为“secure.1”，然后新建“secure”日志用来保存新的日志。第二次日志轮替时，当前的“secure.1”会自动更名为“secure.2”，“secure”更名为“secure.1”，新建“secure”以保存新的日志。以此类推。
    - **include /etc/logrotate.d**: 可以将自定义的日志轮替规则写到这个文件里去。

我们也可以自定义日志轮替规则。

```bash
日志文件地址 {
    参数
}
```

参数:

```bash
daily: 轮替周期 每天
weekly: 轮替周期 每周
monthly: 轮替周期 每月
rotate [num]: 保存日志文件的个数
compress: 轮替时对旧日志进行压缩
create mode owner group: 建立新日志的同时指定权限 所有者 所属组
mail address: 日志轮替时输出内容通过邮件发送到指定的邮件地址
missingok: 如果日志不存在则忽略日志的警告信息
notifempty: 如果日志为空文件则不进行日志轮替
minsize [size]: 日志轮替的最小值 即超过该大小才会轮替 否则到达轮替周期也不会轮替
size [size]: 日志达到指定大小进行轮替 而不是按照轮替的时间周期
dateext: 使用日期作为日志轮替文件的后缀
sharedscripts: 在此关键字之后的脚本只执行一次
prerotate/endscripts: 在日志轮替之前执行脚本命令
postrotate/endscripts: 在日志轮替之后执行脚本命令
```

我们查看可以自定义的日志轮替文件:

```bash
[root@localhost log]# cd /etc/logrotate.d/
[root@localhost logrotate.d]# vim hxlog
[root@localhost logrotate.d]# cat hxlog
/var/log/hx.log {
    missingok    # 如果日志不存在则忽略该日志的警告信息
    daily        # 每天轮替一次
    copytruncate # 先把原始文件拷贝一份重命名，然后把原始文件清空
    rotate 7     # 仅保留7个日志备份
    notifempty   # 如果日志为空文件则不进行日志轮替
}
```

注意: `logrotate.conf`只是定义了日志轮替的规则，那么日志轮替(在指定的时间备份日志)的这个动作，依赖于系统定时任务。可以在`/etc/cron.daily/`中发现一个可执行文件logrotate。

```bash
[root@localhost logrotate.d]# cd /etc/cron.daily/
[root@localhost cron.daily]# ll
总用量 4
-rwxr-xr-x. 1 root root 189 1月   4 2018 logrotate
```