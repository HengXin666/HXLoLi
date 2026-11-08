---
id: cs/crawler/aiohttp-default-timeout
concepts: [aiohttp, HTTP超时]
type: measure
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L32, raw/2026-11-08-crawler/provenance.md#L35]
action: check
---
aiohttp 实测默认 `ClientTimeout(total=300, sock_connect=30, ...)`
> 实测默认值: `ClientTimeout(total=300, sock_connect=30, ...)`, `limit=100`, `limit_per_host=0`
动作:
- [ ] 单请求不该等 300 秒时显式传 `ClientTimeout(total=...)`
