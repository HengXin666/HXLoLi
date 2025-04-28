# 【Anolis OS】龙蜥操作系统(Anolis OS) 安装指南

参考文献:

[【Anolis OS】龙蜥操作系统(Anolis OS) 8.6安装指南](https://blog.csdn.net/qq_45392321/article/details/127168085)

## 前言

因为意外的关机 / 安装了gcc和vim插件/git 等操作 + kawaii_gcc ...

然后就无法登录 root 账号了, 不论是远程连接/图像界面都不行/但是普通用户可以登录, 但是也无法 `su root`

以下是xshell的输出:

```cmd
[C:\~]$ 

Connecting to 192.168.213.66:22...
Connection established.
To escape to local shell, press 'Ctrl+Alt+]'.

Last login: Sun Jan 14 14:13:36 2024
Connection closing...Socket close.

Connection closed by foreign host.

Disconnected from remote host(本机L学习) at 14:20:31.

Type `help' to learn how to use Xshell prompt.
[C:\~]$ 

Connecting to 192.168.213.66:22...
Connection established.
To escape to local shell, press 'Ctrl+Alt+]'.

Last login: Sun Jan 14 14:20:24 2024 from hx_win
ABRT has detected 1 problem(s). For more info run: abrt-cli list --since 1705213224
Connection closing...Socket close.

Connection closed by foreign host.

Disconnected from remote host(本机L学习) at 14:20:47.

Type `help' to learn how to use Xshell prompt.
```

查阅资料, 好像是一个 bush 的错误? 内核的错误...

需要尝试的操作需要root权限... 我一登录root就报错 / 段错误 / ...

一气之下就直接跑路了 (反正 它也已经停止维护了, 接下来是 龙蜥 !!!(aly))

## 龙蜥操作系统(Anolis OS)简介
> Anolis OS 8 是 OpenAnolis 社区推出的完全开源、中立、开放的发行版，它支持多计算架构，也面向云端场景优化，兼容 CentOS 软件生态。Anolis OS 8 旨在为广大开发者和运维人员提供稳定、高性能、安全、可靠、开源的操作系统服务。

## 下载Anolis镜像

下载链接：https://mirrors.openanolis.cn/anolis/8.6/isos/QU1/x86_64/

## 安装操作
> 本篇以 Anolis OS 8.6 版本为例，使用AnolisOS-8.6-QU1-x86_64-dvd.iso
>
> 和之前的没有什么区别