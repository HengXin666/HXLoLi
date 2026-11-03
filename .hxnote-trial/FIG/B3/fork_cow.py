"""fork 之后子进程"只读"一批 Python 对象, 看它自己弄脏了多少内存页.

用法: python3 fork_cow.py [对象个数, 默认 1_000_000]
只在 Linux 上能跑 (读 /proc/self/smaps_rollup).
"""
import gc
import os
import sys
import time

N = int(sys.argv[1]) if len(sys.argv) > 1 else 1_000_000


def private_dirty_kb() -> int:
    # 本进程独占且被写过的页. fork 之后子进程每写一页, 这里就多 4KB
    with open("/proc/self/smaps_rollup") as f:
        for line in f:
            if line.startswith("Private_Dirty:"):
                return int(line.split()[1])
    raise RuntimeError("no Private_Dirty")


def build(kind: str):
    if kind == "float":
        return [float(i) + 0.5 for i in range(N)]  # 不归 gc 管
    return [[i] for i in range(N)]  # list 永远归 gc 管, 头上多一块 PyGC_Head


def child(data, action: str, w: int) -> None:
    before = private_dirty_kb()
    if action == "iterate":
        s = 0
        for x in data:  # 只读: 取出来, 不改
            s += 1
    elif action == "gc":
        gc.collect()
    elif action == "iterate+gc":
        for x in data:
            pass
        gc.collect()
    after = private_dirty_kb()
    os.write(w, f"{after - before}\n".encode())
    os._exit(0)


def run(kind: str, action: str, freeze: bool) -> int:
    gc.disable()
    data = build(kind)
    gc.collect()
    if freeze:
        gc.freeze()  # 把现存对象挪进永久代, 子进程的 gc 不再碰它们
    r, w = os.pipe()
    pid = os.fork()
    if pid == 0:
        os.close(r)
        gc.enable()
        child(data, action, w)
    os.close(w)
    out = os.read(r, 64).decode().strip()
    os.waitpid(pid, 0)
    if freeze:
        gc.unfreeze()
    del data
    gc.collect()
    return int(out)


CASES = [
    ("float", "nothing", False),
    ("float", "iterate", False),
    ("list", "iterate", False),
    ("list", "gc", False),
    ("list", "gc", True),
    ("list", "iterate+gc", True),
]

if __name__ == "__main__":
    print(f"python {sys.version.split()[0]}, N={N}, uname={os.uname().release} {os.uname().machine}")
    for kind, action, freeze in CASES:
        kb = run(kind, action, freeze)
        tag = "freeze" if freeze else "-"
        print(f"{kind:5} {action:11} {tag:7} child_dirty={kb / 1024:8.1f} MB")
        time.sleep(0.2)
