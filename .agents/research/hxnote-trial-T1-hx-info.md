---
hxid: "hx-33087de0"
title: "HTTP缓存四个指令的真实语义"
kind: "info"
created_at: "2026-11-01"
source: "RFC 9111 / RFC 5861 / RFC 9110"
tags: ["HTTP"]
---

# HTTP缓存四个指令的真实语义 知识点

## HTTP 缓存判定一份已存响应是否新鲜, 只比较新鲜寿命与当前年龄

- 公式为 `response_is_fresh = (freshness_lifetime > current_age)`.
- 新鲜度只作用于缓存, 不能用来强迫浏览器刷新已显示的页面.
- 依据: RFC 9111 §4.2.
- 满足: G1

## Cache-Control no-store 禁止缓存存储请求与响应的任何部分

- no-store 对浏览器私有缓存与 CDN 等共享缓存都生效.
- no-store 不是可靠的隐私机制, 不守规矩的缓存与窃听不受它约束.
- 一串指令里只要出现 no-store, 判定结果就与单写 no-store 一致: `private, no-cache, no-store, max-age=0, must-revalidate` 等价于 `no-store`.
- no-store 的代价是每个请求都传完整响应体, 拿不到 304 的省流.
- 依据: RFC 9111 §5.2.2.5; MDN Cache-Control "Preventing storing" 一节; cache_decide.py 实测.
- 满足: G1

## Cache-Control no-cache 允许缓存存储响应, 但每次复用前必须先找源站校验

- 无参数的 no-cache 要求缓存先转发条件请求并拿到成功响应, 才能复用已存响应.
- 即使缓存被配置为可以发陈旧响应, no-cache 也不允许跳过校验.
- 断网时 no-cache 响应只能报错, 不能拿陈旧副本顶上.
- 带字段名的 no-cache (`no-cache="Set-Cookie"`) 实现很少, 缓存通常按无参数形式处理.
- 依据: RFC 9111 §5.2.2.4, §4.2.4.
- 满足: G1

## max-age=0 让响应一进缓存就陈旧, 但断网时仍允许缓存用陈旧副本

- max-age=0 在线时与 no-cache 一样, 每次先校验.
- RFC 9111 §4.2.4 允许断网的缓存产出陈旧响应, 除非有 no-cache 或 must-revalidate 禁止; max-age=0 单独出现时不构成禁止.
- 依据: RFC 9111 §5.2.2.1, §4.2.4; cache_decide.py 实测.
- 满足: G1

## must-revalidate 只管已陈旧的响应: 陈旧后未经源站校验不得复用, 断网时必须报错

- 断网时缓存必须返回错误而不是陈旧响应, 建议状态码 504.
- RFC 9111 建议只在校验失败会导致错误操作时使用, 例如静默未执行的金融交易.
- `max-age=0, must-revalidate` 在在线与断网两种情况下的判定都与 `no-cache` 一致.
- 依据: RFC 9111 §5.2.2.2; cache_decide.py 实测.
- 满足: G1

## stale-while-revalidate 允许缓存在陈旧后的一段窗口内先返回旧副本, 同时后台校验

- `max-age=600, stale-while-revalidate=30` 表示 600 秒新鲜, 之后最多再陈旧服务 30 秒.
- 窗口内没有完成校验时, 窗口过后不应继续陈旧服务.
- 依据: RFC 5861 §3, §3.1.
- 满足: G1

## cache_decide.py 用四十行把以上判定写成可以跑的表

- 用法: `python3 cache_decide.py <年龄秒数>`, 对七组常见指令分别打印在线与断网两种判定.
- 依据: 本笔记同目录 cache_decide.py, 年龄 10 与 615 两次实测.
- 满足: G2

```python [cache-decide-核心]
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
```

## 边界

- 结论是规范层面的, 没有在 Chrome / Firefox / 具体 CDN 上实测断网行为.
- 没有显式过期时间的响应, 缓存可以用启发式新鲜度 (Last-Modified 距今间隔的约 10%), 此时 cache_decide.py 的判定不适用.
- [不确定] stale-while-revalidate 与 must-revalidate 同时出现时谁优先, RFC 9111 与 RFC 5861 都没有写; 脚本按 must-revalidate 优先实现.
- 请求方向的 `Cache-Control: no-cache` (浏览器强制刷新时发出) 是另一组指令, 本笔记不覆盖.

## 来源

- RFC 9111 HTTP Caching https://www.rfc-editor.org/rfc/rfc9111
- RFC 5861 HTTP Cache-Control Extensions for Stale Content https://www.rfc-editor.org/rfc/rfc5861
- RFC 9110 HTTP Semantics https://www.rfc-editor.org/rfc/rfc9110
- MDN Cache-Control https://developer.mozilla.org/en-US/docs/Web/HTTP/Reference/Headers/Cache-Control
