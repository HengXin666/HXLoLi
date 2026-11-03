---
hxid: "hx-33087de0"
title: "HTTP缓存四个指令的真实语义"
created_at: "2026-11-01"
model: "Unknown"
skill: ["hx-note"]
authors: "Heng_Xin"
tags: ["HTTP"]
---

# HTTP缓存四个指令的真实语义

RFC 9111 允许缓存存下 `no-cache` 响应, 只是复用前必须先校验. 下面把四个指令按"在线"和"断网"各走一遍.

## 0x00 新鲜度公式

缓存判断一份存着的响应新不新鲜, RFC 9111 第 4.2 节给的公式只有一行

```text [新鲜度-公式]
response_is_fresh = (freshness_lifetime > current_age)
```

没有别的指令插手时, 新鲜就直接用, 不碰网络. 不新鲜就先发条件请求去源站问, 拿到 304 就接着用旧的, 拿到 200 就换新的. 下面几个指令改的正是这条默认路线.

## 0x01 no-store: 唯一不存

`no-store` 要求缓存不存请求和响应的任何部分, 浏览器和 CDN 都算. 所以每次请求都是完整的 200, 拿不到 304 省下的响应体.

MDN 上有一串常见的"保险写法", 五个指令叠在一起

```http [no-store-叠加]
Cache-Control: private, no-cache, no-store, max-age=0, must-revalidate
```

按规范判下来它就等于单写一个 `no-store`, 其余四个是噪声. 另外 RFC 明说 `no-store` 不是可靠的隐私手段, 不守规矩的缓存照样能存.

## 0x02 no-cache: 存, 但复用前校验

`no-cache` 允许存, 但复用前必须先找源站校验, 就算缓存被配置成"可以发旧内容"也不行. 好处是内容没变时源站只回一个 304, 不重传响应体.

断网的时候它更硬: 问不到源站就只能报错, 不能拿旧副本顶上.

## 0x03 max-age=0 的断网缺口

`max-age=0` 让响应一进缓存就过期, 在线时和 `no-cache` 一样每次都问. 差别在断网. RFC 9111 第 4.2.4 节允许断网的缓存发旧内容, 除非有 `no-cache` 或 `must-revalidate` 明确禁止. `max-age=0` 单独出现不算禁止.

![四个指令在在线与断网时各走到哪一步 ##w800##](cache-offline.drawio.svg)

上图里只有 `max-age=0` 那条线在断网时走到了"用陈旧副本", 另外两条校验路线都停在报错.

想要和 `no-cache` 完全一样, 就补上 `must-revalidate`. 它的规定是过期后没校验成功就不许复用, 断网时必须报错, 建议回 504.

```http [等价写法-no-cache]
Cache-Control: no-cache
```

```http [等价写法-max-age-0-must-revalidate]
Cache-Control: max-age=0, must-revalidate
```

## 0x04 把判定写成能跑的脚本

上面这些规则写成了 [cache_decide.py](cache_decide.py). 给它一个年龄秒数, 它会对七组常见指令打印在线与断网的结果

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

```text [cache-decide-年龄615秒]
max-age=0                                  在线: 先发条件请求校验 (304 复用 / 200 替换)
                                           断网: 允许用陈旧副本
max-age=600, stale-while-revalidate=30     在线: 先给陈旧副本, 后台校验
                                           断网: 允许用陈旧副本
```

第二行是 RFC 5861 的 `stale-while-revalidate`: 600 秒新鲜, 过期后最多再先给 30 秒旧内容, 同时后台去校验.

> [!TIP]
> `stale-while-revalidate` 和 `must-revalidate` 同时出现时谁说了算, RFC 9111 和 RFC 5861 都没写. 脚本按 `must-revalidate` 优先, 这一点规范里找不到依据.

脚本只管有显式 `max-age` 的情况. 没写过期时间的响应, 缓存可以按 `Last-Modified` 距今间隔的大约 10% 自己估, 那时结果就不一定了. 浏览器和 CDN 在断网时是不是真按规范走, 本文没有实测.
