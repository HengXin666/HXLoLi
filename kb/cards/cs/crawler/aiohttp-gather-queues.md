---
id: cs/crawler/aiohttp-gather-queues
concepts: [aiohttp, 并发控制, 连接池]
type: measure
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L33, raw/2026-11-08-crawler/provenance.md#L43]
action: check
---
aiohttp 默认配置 gather 700 个打 1 秒延迟服务全部成功, 超出 limit 的请求排队等待, 没有抛池超时
> 默认配置 `gather` 700 个打 1 秒延迟服务, 全部成功, 7.1s (超出 limit 的请求排队等待, 没有抛池超时)
动作:
- [ ] 从 aiohttp 换到 httpx 时补上信号量, 两者超出连接上限时行为不同
