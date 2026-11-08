---
id: cs/crawler/httpx-gather-pooltimeout
concepts: [httpx, 并发控制, 连接池]
type: measure
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L24, raw/2026-11-08-crawler/material.md#L75, raw/2026-11-08-crawler/provenance.md#L54]
action: check
---
`AsyncClient` 默认配置下 gather 700 个请求, 502 个 `PoolTimeout`, 套 `Semaphore(100)` 后 700 个全成功
> `AsyncClient` 默认配置下 `asyncio.gather` 700 个请求打 1 秒延迟的服务, 198 个成功, 502 个 `PoolTimeout`; 套 `Semaphore(100)` 后 700 个全成功, 7.8s
> PoolTimeout 的精确个数受机器调度影响, 只当量级参考
动作:
- [ ] 大批 `asyncio.gather` 前先套 `asyncio.Semaphore`, 信号量不超过 `max_connections`
- [ ] 502 这个数只当量级参考
