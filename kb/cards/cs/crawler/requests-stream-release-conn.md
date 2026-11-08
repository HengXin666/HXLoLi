---
id: cs/crawler/requests-stream-release-conn
concepts: [requests, 连接池]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L10]
action: check
---
`stream=True` 时只有读完 body 或调用 `close` 才把连接还给池
> `stream=True` 时只有读完 body 或调用 `close` 才把连接还给池
动作:
- [ ] 用 `stream=True` 的代码路径确认每个响应都读完 body 或调用了 `close`
