---
hxid: "hx-3cb86e7e"
title: "SQLite WAL的检查点饥饿"
kind: "info"
created_at: "2026-11-01"
source: "sqlite.org wal.html / pragma.html / fileformat2.html; wal_starve.py 实测"
tags: ["SQLite"]
---

# SQLite WAL的检查点饥饿 知识点

## SQLite WAL 模式把改动追加进 -wal 文件, 原数据库文件在检查点之前不动

- 提交等于往 WAL 追加一条提交记录, 不写原数据库文件.
- 检查点 (checkpoint) 是把 WAL 里的事务搬回原数据库文件的操作.
- 依据: sqlite.org/wal.html §2, §2.1.
- 满足: G1

## SQLite WAL 模式下读者不挡写者, 因为每个读事务固定一个 end mark

- 读事务开始时记住 WAL 里最后一条有效提交记录的位置, 称为 end mark, 整个读事务期间不变.
- 读一页时先在 WAL 里找 end mark 之前的最新版本, 找不到才读原数据库文件.
- 写者只往 WAL 末尾追加, 同一时刻只能有一个写者.
- wal-index 放在共享内存里, 所以所有读者必须在同一台机器上.
- 依据: sqlite.org/wal.html §2.2.
- 满足: G1

## SQLite 默认在 WAL 达到 1000 页时自动跑 PASSIVE 检查点, 4KB 页时约 4MB

- 自动检查点在一次提交让 WAL 达到 1000 页时触发, 阈值可用 `PRAGMA wal_autocheckpoint` 改, 设为 0 或负数则关闭.
- 自动检查点都是 PASSIVE 模式: 尽量搬帧, 不等任何读者或写者.
- 检查点通常不截断 WAL 文件, 只让 SQLite 从头覆盖写.
- 依据: sqlite.org/wal.html §3.1, §6; sqlite.org/pragma.html wal_autocheckpoint, wal_checkpoint.
- 满足: G1

## SQLite 检查点搬到任何一个读者的 end mark 就停下, 读者一直不走时 WAL 无限增长

- 只要有连接开着读事务, 检查点就不能跑完并重置 WAL; sqlite.org 称之为 checkpoint starvation.
- 有长读者时 -wal 大小与提交次数成正比: 每次提交是一条 1KB 的 INSERT, 3000 次提交 20362 KB, 6000 次 40737 KB; 没有长读者时两种次数都停在 4031 KB.
- 关掉自动检查点也会让 WAL 无限增长.
- 依据: sqlite.org/wal.html §2.2, §6; wal_starve.py 实测 (SQLite 3.46.1).
- 满足: G1, G2

## Python sqlite3 里一个没读完的 SELECT 游标也算读事务, 不写 BEGIN 照样造成检查点饥饿

- 只 `execute("SELECT * FROM t")` 并 `fetchone()`, 表里还有没读的行, -wal 与 BEGIN 不提交那组数字相同: 3000 次提交 20362 KB, 6000 次 40737 KB.
- 游标 `close()` 之后, 检查点能把全部帧搬回去.
- 依据: wal_starve.py 实测 "游标没读完" 一组 (SQLite 3.46.1).
- 满足: G2

## 读者还在时 TRUNCATE 检查点返回 busy=1, 读者结束后 PASSIVE 能搬完但不缩小文件

- `PRAGMA wal_checkpoint(TRUNCATE)` 返回三列: busy, WAL 里已写入的页数, 已搬回原数据库文件的页数; 读者还在时实测为 (1, 5061, 52), 即只搬回 52 页.
- 读者结束后 PASSIVE 实测为 (0, 5061, 5061), -wal 文件仍是 20362 KB.
- 读者结束后 TRUNCATE 实测为 (0, 0, 0), -wal 变成 0 KB; TRUNCATE 的定义就是在 RESTART 成功后把 WAL 截成 0 字节.
- RESTART 与 TRUNCATE 跑的时候会阻塞并发写者; sqlite.org 建议留出没有读者的空档跑检查点, 或手动跑 RESTART / TRUNCATE.
- 依据: sqlite.org/pragma.html wal_checkpoint; sqlite.org/wal.html §6; wal_starve.py 实测.
- 满足: G1, G2

## wal_starve.py 用三组对照复现检查点饥饿

- 用法: `python3 wal_starve.py [提交次数]`, 默认 3000; 三组分别是没有长读者、BEGIN 不提交、游标没读完.
- 满足: G2

```python [wal-starve-核心]
def case(label: str, reader: str, commits: int = 3000) -> None:
    path, w = fresh_db()
    r = sqlite3.connect(path, isolation_level=None)
    cur = None
    if reader == "begin":
        r.execute("BEGIN")
        r.execute("SELECT count(*) FROM t").fetchone()
    elif reader == "cursor":
        cur = r.execute("SELECT * FROM t")
        cur.fetchone()  # 没读完的游标, 没有 BEGIN 也占着读事务
    write(w, commits)
```

- 依据: 本笔记同目录 wal_starve.py.

## 边界

- 实测只在 Linux 本地文件系统上跑过, 没测 macOS 与 Windows.
- WAL 不能用在网络文件系统上, 因为 wal-index 依赖同机共享内存.
- [不确定] 读多写少的应用里 sqlite.org 估计 WAL 比回滚日志慢 1% 到 2%, 原文是估计值 (perhaps), 依据 sqlite.org/wal.html §1.
- 读者不挡写者有例外: 独占锁模式、最后一个连接正在清理、正在恢复时, 查询会返回 SQLITE_BUSY, 依据 sqlite.org/wal.html §9.
- 游标结论依赖表里还有没读的行; 只剩一行时 fetchone 之后语句已结束, 实测不造成饥饿.

## 来源

- SQLite Write-Ahead Logging https://www.sqlite.org/wal.html
- SQLite Pragma https://www.sqlite.org/pragma.html
- SQLite Database File Format https://www.sqlite.org/fileformat2.html
- Ben Johnson, How SQLite Scales Read Concurrency https://fly.io/blog/sqlite-internals-wal/
