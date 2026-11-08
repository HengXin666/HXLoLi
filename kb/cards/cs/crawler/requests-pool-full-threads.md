---
id: cs/crawler/requests-pool-full-threads
concepts: [requests, 连接池, 并发控制]
type: measure
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L15, raw/2026-11-08-crawler/provenance.md#L36-L38]
action: check
---
32 线程共用一个默认 Session 打同一主机, 出现 22 次 `Connection pool is full, discarding connection`, `pool_maxsize` 调到 32 后为 0 次
> 32 线程共用一个默认 Session 打同一主机, 请求都成功, 但出现 22 次 `Connection pool is full, discarding connection`; 把 `pool_maxsize` 调到 32 后为 0 次
动作:
- `HTTPAdapter(pool_maxsize=32)`
- [ ] 多线程共用 Session 时把 `pool_maxsize` 调到线程数, 再看日志里有无该警告
