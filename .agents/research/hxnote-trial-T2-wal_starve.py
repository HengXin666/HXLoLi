"""复现 SQLite 的检查点饥饿: 一个没结束的读事务让 -wal 文件一直长。"""
import os
import sqlite3
import sys
import tempfile


def wal_kb(path: str) -> int:
    return os.path.getsize(path + "-wal") // 1024 if os.path.exists(path + "-wal") else 0


def fresh_db() -> tuple[str, sqlite3.Connection]:
    path = os.path.join(tempfile.mkdtemp(), "demo.db")
    w = sqlite3.connect(path, isolation_level=None)
    w.execute("PRAGMA journal_mode=WAL")
    w.execute("CREATE TABLE t(id INTEGER PRIMARY KEY, v BLOB)")
    for _ in range(50):
        w.execute("INSERT INTO t(v) VALUES (zeroblob(1))")  # 给游标留下没读完的行
    return path, w


def write(w: sqlite3.Connection, commits: int) -> None:
    for _ in range(commits):
        w.execute("INSERT INTO t(v) VALUES (zeroblob(1024))")  # 每条一个提交


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
    print(f"{label:<16} {commits} 次提交后 -wal = {wal_kb(path)} KB")
    if reader == "none":
        return
    print(f"{'':<16} 读者还在时 TRUNCATE -> {w.execute('PRAGMA wal_checkpoint(TRUNCATE)').fetchone()}")
    if cur:
        cur.close()
    else:
        r.execute("COMMIT")
    print(f"{'':<16} 读者结束后 PASSIVE  -> {w.execute('PRAGMA wal_checkpoint(PASSIVE)').fetchone()}, -wal = {wal_kb(path)} KB")
    print(f"{'':<16} 读者结束后 TRUNCATE -> {w.execute('PRAGMA wal_checkpoint(TRUNCATE)').fetchone()}, -wal = {wal_kb(path)} KB")


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3000
    print("SQLite", sqlite3.sqlite_version)
    case("没有长读者", "none", n)
    case("BEGIN 不提交", "begin", n)
    case("游标没读完", "cursor", n)
