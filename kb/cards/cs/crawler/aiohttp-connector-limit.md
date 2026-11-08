---
id: cs/crawler/aiohttp-connector-limit
concepts: [aiohttp, 连接池, 并发控制]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L29, raw/2026-11-08-crawler/provenance.md#L35]
action: check
---
`TCPConnector(limit=...)` 默认总连接 100, `limit_per_host` 默认 0 即不按主机限
> `TCPConnector(limit=...)` 默认总连接 100, `limit=0` 不限; `limit_per_host` 默认 0 即不按主机限
动作:
- [ ] 要限制单站压力时显式设 `limit_per_host`
- [ ] 记住 `limit=0` 是不限, 不是 0 个连接
