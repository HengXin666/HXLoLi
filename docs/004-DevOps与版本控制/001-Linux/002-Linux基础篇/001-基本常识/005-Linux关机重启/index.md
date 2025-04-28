# Linux关机重启
## 前言

> 这个不是简单? 我直接在 VM 关闭虚拟机吧就行了?

大哥，你现在是远程连接呢?

> ...

## 命令
### shutdown

```Bash
shutdown -h now # 表示立即关机

shutdown -h     # 表示一分钟后关机

shutdown -r now # 立即重启
```

### halt

就是直接使用，效果等同于关机. (与 `shutdown -h now` 等价)

### sync

把内存的数据同步到磁盘

### reboot

就是重启系统

## 注意细节

1. 不管是`重启系统`还是`关闭系统`，首先要运行`sync`命令，把内存中的数据写入到磁盘中。

2. 目前的`shutdown`、`reboot`、`halt`命令在关机前都进行了`sync`。