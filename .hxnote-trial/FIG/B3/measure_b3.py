"""B3 三张图的实测. 输出 b3_data.json, 出图见 make_b3.py.

用法: python3 measure_b3.py
只在 Linux 上能跑 (读 /proc/self/smaps_rollup). 用 ctypes 直接读对象内存核对字段偏移.
"""
import collections
import ctypes
import gc
import json
import os
import sys

PAGE = os.sysconf("SC_PAGE_SIZE")


def private_dirty_kb() -> int:
    with open("/proc/self/smaps_rollup") as f:
        for line in f:
            if line.startswith("Private_Dirty:"):
                return int(line.split()[1])
    raise RuntimeError("no Private_Dirty")


def build(kind: str, n: int):
    if kind == "float":
        return [float(i) + 0.5 for i in range(n)]
    return [[i] for i in range(n)]


def in_child(data, step: int) -> int:
    """fork 一个子进程, 每隔 step 个对象读一个, 返回子进程新增脏页 KB."""
    r, w = os.pipe()
    pid = os.fork()
    if pid == 0:
        os.close(r)
        before = private_dirty_kb()
        for k in range(0, len(data), step):
            x = data[k]  # 只读
        os.write(w, str(private_dirty_kb() - before).encode())
        os._exit(0)
    os.close(w)
    out = int(os.read(r, 64))
    os.waitpid(pid, 0)
    return out


def layout():
    """字段偏移: 用 ctypes 从 id(obj) 出发读内存, 和已知值对上才算数."""
    f = 12345.5
    l = [7]
    rc_before = ctypes.c_ssize_t.from_address(id(f)).value
    g = f  # 多一个引用
    rc_after = ctypes.c_ssize_t.from_address(id(f)).value
    del g
    return {
        "page_size": PAGE,
        "float": {
            "getsizeof": sys.getsizeof(f),
            "ob_refcnt@0 before/after one more ref": [rc_before, rc_after],
            "ob_type@8 == id(float)": ctypes.c_void_p.from_address(id(f) + 8).value == id(float),
            "ob_fval@16": ctypes.c_double.from_address(id(f) + 16).value,
            "gc_tracked": gc.is_tracked(f),
        },
        "list": {
            "getsizeof": sys.getsizeof(l),
            "ob_type@8 == id(list)": ctypes.c_void_p.from_address(id(l) + 8).value == id(list),
            "ob_size@16": ctypes.c_ssize_t.from_address(id(l) + 16).value,
            "ob_item@24 -> items[0] is the int 7": ctypes.c_void_p.from_address(
                ctypes.c_void_p.from_address(id(l) + 24).value).value == id(7),
            "allocated@32": ctypes.c_ssize_t.from_address(id(l) + 32).value,
            "gc_tracked": gc.is_tracked(l),
            "PyGC_Head": "16 bytes before id(obj), sys.getsizeof includes it",
        },
    }


def stride(kind: str, n: int = 200_000):
    """相邻两个对象地址差的众数 = pymalloc 给一个对象的槽位大小."""
    data = build(kind, n)
    c = collections.Counter(id(data[k + 1]) - id(data[k]) for k in range(n - 1))
    top, cnt = c.most_common(1)[0]
    return {"mode_bytes": top, "share": round(cnt / (n - 1), 4)}


def main():
    gc.disable()
    out = {
        "env": f"python {sys.version.split()[0]}, {os.uname().sysname} {os.uname().release} {os.uname().machine}",
        "layout": layout(),
        "stride": {k: stride(k) for k in ("float", "list")},
        "step_sweep": {},
        "n_sweep": {},
    }
    n = 1_000_000
    for kind in ("float", "list"):
        data = build(kind, n)
        out["step_sweep"][kind] = {s: in_child(data, s) for s in (1, 2, 8, 32, 64, 128, 256, 512, 1024)}
        del data
    for kind in ("float", "list"):
        out["n_sweep"][kind] = {}
        for m in (125_000, 250_000, 500_000, 1_000_000, 2_000_000):
            data = build(kind, m)
            out["n_sweep"][kind][m] = in_child(data, 1)
            del data
    json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "b3_data.json"), "w"),
              indent=1, ensure_ascii=False)
    print(json.dumps(out, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
