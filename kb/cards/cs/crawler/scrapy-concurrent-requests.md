---
id: cs/crawler/scrapy-concurrent-requests
concepts: [Scrapy, 并发控制]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L35, raw/2026-11-08-crawler/material.md#L37]
action: check
---
Scrapy 2.20 的 `CONCURRENT_REQUESTS` 默认 16
> Scrapy (2.20.0)
> `CONCURRENT_REQUESTS` 默认 16
动作:
- [ ] 在项目 settings 里显式写 `CONCURRENT_REQUESTS`
