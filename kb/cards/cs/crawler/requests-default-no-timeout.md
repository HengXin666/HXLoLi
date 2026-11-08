---
id: cs/crawler/requests-default-no-timeout
concepts: [requests, HTTP超时]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L7]
action: check
---
requests 默认不设超时, 不显式传 `timeout` 时请求可以挂几分钟甚至更久
> requests 默认不设超时, 不显式传 `timeout` 时请求可以挂几分钟甚至更久; 单值同时作用于 connect 与 read, 也可传元组分开设
动作:
- [ ] 每个 requests 调用都显式传 `timeout`, 需要时传元组分开设 connect 与 read
