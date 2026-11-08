---
id: cs/crawler/requests-defaults-measured
concepts: [requests, HTTP超时, 连接池, 重试]
type: measure
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L14, raw/2026-11-08-crawler/provenance.md#L33]
action: check
---
requests 2.34.2 实测 `timeout` 默认 None, `DEFAULT_POOLSIZE = 10`, `DEFAULT_RETRIES = 0`
> 实测默认值: `timeout` 默认 None, `DEFAULT_POOLSIZE = 10`, `DEFAULT_RETRIES = 0` (requests 2.34.2)
动作:
- [ ] 新项目显式设 `timeout`、`pool_maxsize` 与 `max_retries`, 不依赖这三个默认值
- [ ] 升级 requests 后重新自省这三个默认值
