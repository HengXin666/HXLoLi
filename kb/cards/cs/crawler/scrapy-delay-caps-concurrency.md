---
id: cs/crawler/scrapy-delay-caps-concurrency
concepts: [Scrapy, 并发控制]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L39]
action: check
---
响应时间小于 DOWNLOAD_DELAY 时该域实际并发为 1
> 响应时间小于 DOWNLOAD_DELAY 时该域实际并发为 1
动作:
- [ ] 设了 `DOWNLOAD_DELAY` 后, 不指望调高 `CONCURRENT_REQUESTS_PER_DOMAIN` 来提速
