---
hxid: "hx-3cb86e7e"
title: "SQLite WAL的检查点饥饿"
created_at: "2026-11-01"
model: "Unknown"
skill: ["hx-note"]
authors: "Heng_Xin"
tags: ["SQLite"]
---

# SQLite WAL的检查点饥饿

同样 3000 次 1KB 插入, 没有长读者时 -wal 停在 4031 KB. 旁边开着一个没提交的读事务, 跑完一看是 20362 KB.

## 0x00 读者为什么不挡写者

![写者追加, 读者各停在自己的 end mark (示意) ##w800##](wal-readers.drawio.svg)

写者只往 -wal 末尾追加. 每个读事务开始时记下一个 end mark, 之后只看它之前的帧. 所以读者挡不住写者. 例外有三种: 独占锁模式、最后一个连接正在清理、正在恢复, 这时查询会报 SQLITE_BUSY.

坏处在检查点: 它搬到任何一个读者的 end mark 就得停, 后面的帧都得留着.

## 0x01 一个读者撑大 WAL

| 读者 | 3000 次提交后 -wal | 6000 次提交后 -wal |
|---|---|---|
| 没有长读者 | 4031 KB | 4031 KB |
| BEGIN 不提交 | 20362 KB | 40737 KB |
| 游标没读完 | 20362 KB | 40737 KB |

没有长读者时, 默认 1000 页的自动检查点把 -wal 压在 4MB 左右. 只要有一个读者不走, 大小就跟着提交次数线性涨, 提交翻倍它也翻倍. sqlite.org 管这叫 checkpoint starvation.

## 0x02 一个没读完的游标

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

`cursor` 那一支没有 BEGIN, 只是 SELECT 之后取了一行就放着. 表里还有没读的行, 这个游标就是一个读事务. 3000 次和 6000 次的 -wal 都和 BEGIN 不提交那组相同. 完整脚本是 [wal_starve.py](wal_starve.py), 带一个提交次数参数.

## 0x03 等读者走再截断

```text [wal-starve-收尾]
读者还在时 TRUNCATE -> (1, 5061, 52)
读者结束后 PASSIVE  -> (0, 5061, 5061), -wal = 20362 KB
读者结束后 TRUNCATE -> (0, 0, 0), -wal = 0 KB
```

三列是 busy, WAL 里的页数, 搬回去的页数. 读者还在时 TRUNCATE 只搬回 52 页就报 busy. 读者走了, PASSIVE 能搬完, 但文件还是 20 MB. TRUNCATE 搬完再截成 0 字节.

sqlite.org 给了两条路. 一是留出没有读者的空档让检查点跑完, 二是手动跑 RESTART / TRUNCATE. 后两种跑的时候会挡住并发写者. 而且只要读者不走, 检查点就跑不完, 所以先关游标、结束读事务.
