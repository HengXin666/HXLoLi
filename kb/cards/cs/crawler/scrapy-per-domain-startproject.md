---
id: cs/crawler/scrapy-per-domain-startproject
concepts: [Scrapy, 并发控制]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L38]
action: check
---
`CONCURRENT_REQUESTS_PER_DOMAIN` 在 `startproject` 生成的 settings 里是 1, 全局 fallback 是 8
> `CONCURRENT_REQUESTS_PER_DOMAIN`: `startproject` 生成的 settings 里是 1, 全局 fallback 是 8
动作:
- [ ] 查项目 settings 是否保留了 `startproject` 生成的值, 没有就按 8 算
