---
id: cs/crawler/requests-session-reuse
concepts: [requests, 连接池]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L9]
action: check
---
对同一主机多次请求用 `Session`, 它复用 urllib3 连接池并跨请求保存 cookie
> `Session` 复用 urllib3 连接池并跨请求保存 cookie, 对同一主机多次请求会复用 TCP 连接
动作:
- [ ] 同一主机的多次请求共用一个 `Session` 对象
