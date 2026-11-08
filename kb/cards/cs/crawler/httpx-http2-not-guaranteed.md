---
id: cs/crawler/httpx-http2-not-guaranteed
concepts: [httpx, HTTP2]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L22, raw/2026-11-08-crawler/material.md#L73]
action: check
---
httpx 开了 `http2=True` 不等于走 HTTP/2, 服务端不支持就回落 HTTP/1.1
> httpx 开了 `http2=True` 不等于走 HTTP/2, 服务端不支持就回落 HTTP/1.1
> HTTP/2 默认关闭, 原因是 HTTP/1.1 实现更成熟
动作:
- `pip install httpx[http2]`
- [ ] 开启 `http2=True` 后用 `response.http_version` 查实际协议
