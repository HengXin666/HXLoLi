# Redis
## 一、「Redis」って何?
### 1.1 NoSql数据库的基本概述
#### 1.1.1 什么是nosql数据库
**NoSQL(NoSQL = Not Only SQL)**，意即“不仅仅是SQL”，泛指非关系型的数据库。 NoSQL 不依赖业务逻辑方式存储，而以简单的key-value模式存储。因此大大的增加了数据库的扩展能力。nosql数据库有以下特点:
- 不遵循SQL标准。
- 不支持ACID。
- 远超于SQL的性能。

那么nosql有哪些适用场景呢?
- 对数据高并发的读写
- 海量数据的读写
- 对数据高可扩展性的

当然nosql也有不适用的场景:
- 需要事务支持
- 基于sql的结构化查询存储，处理复杂的关系,需要sql查询。

#### 1.2.2 常见的nosql数据库
Memcache
- Memcache很早出现的NoSql数据库， 数据都在内存中，`一般不持久化`。支持简单的key-value模式，支持类型单一。一般是作为缓存数据库辅助持久化的数据库

redis
- redis几乎覆盖了Memcached的绝大部分功能,数据都在内存中，`支持持久化`，主要用作备份恢复。 除了支持简单的key-value模式，还支持`多种数据结构的存储`，比如 list、set、hash、zset等。一般是作为缓存数据库辅助持久化的数据库。

MongoDB
- MongoDB 高性能、开源、模式自由(schema free)的**文档型数据库(类似于json)**。数据都在内存中， 如果内存不足，把不常用的数据保存到硬盘。虽然是key-value模式，但是对value（尤其是`json`）提供了丰富的查询功能。MongoDB支持二进制数据及大型对象。

[数据库排行榜](https://db-engines.com/en/ranking)

### 1.2 redis的诞生历史

简单的说, 就是redis的作者之前开发了一个`访客信息`网站, 可以查询到访客的信息数据, 只需要在html里面嵌入js即可, 但是后来因为使用的人很多, 数据量和并发量很大, sql等磁盘数据库就很慢了, 因为磁盘io很慢, 无论再怎么优化sql都不行; 然后redis作者就写了个在内存上的数据库, 发现很不错, 就使用c语言系统的重写这个数据库, 然后redis横空出世!

### 1.3 redis简介

<p style="text-align:center"><iframe src="//player.bilibili.com/player.html?aid=444898573&bvid=BV1Jj411D7oG&cid=1169823650&p=2" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe></p>

- 性能极高
- 数据类型丰富，单键值对最大支持512M大小的数据
- 简单易用，支持所有主流编程语言
- 支持数据持久化、主从复制、哨兵模式等高可用特性

### 1.4 redis的概述
- Redis是一个开源的key-value存储系统。
- 和Memcached类似，它支持存储的value类型相对更多，包括string(字符串)、list(链表)、set(集合)、zset(sorted set --有序集合)和hash（哈希类型）。
- 这些数据类型都支持push/pop、add/remove及取交集并集和差集及更丰富的操作，而且这些操作都是原子性的。
- 在此基础上，Redis支持各种不同方式的排序。
- 与memcached一样，为了保证效率，数据都是缓存在内存中。
- 区别的是Redis会周期性的把更新的数据写入磁盘或者把修改操作写入追加的记录文件。
- 并且在此基础上实现了master-slave(主从)同步。

> [Redis 6.0 新特性-多线程连环13问！](https://www.cnblogs.com/mumage/p/12832766.html)

基于这些特性，redis的使用场景非常广泛:

| 场景 | 解决方案 |
| :--- | :--- |
| 获取最新数据 | 通过List实现按自然时间排序的数据 |
| 排行榜 | 利用Zset有序集合 |
| 时效性的数据，比如手机验证码 | Expire过期 |
| 计数器、秒杀 | 原子性 自增方法INCR DECR |
| 去除大量数据中的重复数据 | 利用Set集合 |
| 构建队列 | List集合 |
| 发布订阅消息系统 | pub/sub模式 |

## 二、どうすれば「Redis」安装が出来る?
### 2.1 服务端
Linux:
```bash
yum install gcc # 先决条件: 安装了gcc
yum install redis
```

一般安装好后会在`/usr/bin`下:

```bash
[root@localhost bin]# ll | grep -i redis
-rwxr-xr-x  1 root root       656264 10月 31 2022 redis-benchmark
lrwxrwxrwx  1 root root           12 10月 31 2022 redis-check-aof -> redis-server
lrwxrwxrwx  1 root root           12 10月 31 2022 redis-check-rdb -> redis-server
-rwxr-xr-x  1 root root       827592 10月 31 2022 redis-cli
lrwxrwxrwx  1 root root           12 10月 31 2022 redis-sentinel -> redis-server
-rwxr-xr-x  1 root root      1800840 10月 31 2022 redis-server

```

这里我们简单了解这几个文件的用途:
- redis-benchmark: 性能测试工具，可以在自己本子运行，看看自己本子性能如何
- redis-check-aof: 修复有问题的AOF文件，rdb和aof后面讲
- redis-check-dump: 修复有问题的dump.rdb文件
- redis-sentinel: Redis集群使用
- redis-server: Redis服务器启动命令
- redis-cli: 客户端，操作入口

~~docker, 非常简单 run 就完事~~ Docke: [安装redis](../../../../004-DevOps与版本控制/002-Docker容器/003-Docker基础篇/008-Docker中常见软件的常规安装/003-安装redis/index.md)

windows: 自己想办法!
### 2.2 客户端(操作数据)
#### 2.2.1 命令行 CLI(Command Line Interface)

#### 2.2.2 程序语言调用 APT

#### 2.2.3 图形化界面 GUI
大推荐: [Another Redis Desktop Manager](https://github.com/qishibo/AnotherRedisDesktopManager/)(支持中文!(国人开发))