# Linux 内存日志
在linux中，有一部分日志信息是没有写到日志文件里面去的，而是写在内存中的。这些日志的特点是日志信息都在随时发生变化。比如linux内核的日志信息。内存日志还有一个特点是linux系统在重新启动的时候，内存日志就会被清空。

操作内存日志的常用指令如下:

- `journalctl` 查看所有的内存日志
- `journalctl -n 3` 查看最新3条
- `journalctl --since 15:00 --until 15:10` 查看区间时间内的日志 可加日期
- `journalctl -p err` 查看报错日志
- `journalctl -o verbose` 日志详细内容
