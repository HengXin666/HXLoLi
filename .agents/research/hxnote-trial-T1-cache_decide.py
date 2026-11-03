"""按 RFC 9111 判定: 一份已存响应, 在给定年龄与网络状态下, 缓存能怎么处理下一个请求。"""
import sys


def parse(cc: str) -> dict:
    out = {}
    for part in cc.split(","):
        k, _, v = part.strip().partition("=")
        if k:
            out[k.lower()] = int(v) if v.isdigit() else True
    return out


def decide(cc: str, age: int, online: bool = True) -> str:
    d = parse(cc)
    if "no-store" in d:
        return "不存, 每次完整请求源站 (200)" if online else "手里没有副本, 请求失败"
    lifetime = d.get("max-age")
    fresh = lifetime is not None and lifetime > age
    if "no-cache" in d:
        return "先发条件请求校验 (304 复用 / 200 替换)" if online else "报错, 不许用陈旧副本"
    if fresh:
        return "直接复用, 不碰网络"
    if online:
        swr = d.get("stale-while-revalidate", 0)
        if lifetime is not None and "must-revalidate" not in d and age < lifetime + swr:
            return "先给陈旧副本, 后台校验"
        return "先发条件请求校验 (304 复用 / 200 替换)"
    if "must-revalidate" in d:
        return "报错 (建议 504), 不许用陈旧副本"
    return "允许用陈旧副本"


CASES = ["no-store", "no-cache", "max-age=0", "max-age=0, must-revalidate",
         "max-age=600", "max-age=600, stale-while-revalidate=30",
         "private, no-cache, no-store, max-age=0, must-revalidate"]

if __name__ == "__main__":
    age = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    for cc in CASES:
        print(f"{cc:<58} age={age:<4} 在线: {decide(cc, age)}")
        print(f"{'':<58} {'':<8} 断网: {decide(cc, age, online=False)}")
