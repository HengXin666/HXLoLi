---
id: cs/crawler/httpx-pool-limits-default
concepts: [httpx, 连接池]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L21, raw/2026-11-08-crawler/provenance.md#L34]
action: check
---
httpx 连接池默认 `max_connections=100`, `max_keepalive_connections=20`, `keepalive_expiry=5`
> 连接池默认 `max_connections=100`, `max_keepalive_connections=20`, `keepalive_expiry=5`
动作:
- [ ] 并发超过 100 时用 `httpx.Limits(max_connections=...)` 调整, 或在外面套信号量
