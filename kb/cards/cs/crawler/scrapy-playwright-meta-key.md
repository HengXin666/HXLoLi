---
id: cs/crawler/scrapy-playwright-meta-key
concepts: [Playwright, Scrapy]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L59]
action: check
---
scrapy-playwright 只处理 `meta` 里带 `playwright` 键的请求, 其余仍走普通下载器, 需要 asyncio reactor
> scrapy-playwright 是一个 Scrapy 下载处理器, 只处理 `meta` 里带 `playwright` 键的请求, 其余仍走普通下载器; 需要 asyncio reactor
动作:
- [ ] 要渲染的请求在 `meta` 里带 `playwright` 键
- [ ] 确认项目用的是 asyncio reactor
