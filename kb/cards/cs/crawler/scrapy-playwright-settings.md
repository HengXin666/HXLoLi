---
id: cs/crawler/scrapy-playwright-settings
concepts: [Playwright, Scrapy, 并发控制]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L60]
action: check
---
`PLAYWRIGHT_MAX_PAGES_PER_CONTEXT` 默认等于 `CONCURRENT_REQUESTS`, `PLAYWRIGHT_ABORT_REQUEST` 可按谓词丢弃资源请求
> `PLAYWRIGHT_MAX_PAGES_PER_CONTEXT` 默认等于 `CONCURRENT_REQUESTS`; `PLAYWRIGHT_ABORT_REQUEST` 可按谓词丢弃资源请求
动作:
- [ ] 调 `CONCURRENT_REQUESTS` 时一起看浏览器页数
- [ ] 用 `PLAYWRIGHT_ABORT_REQUEST` 丢弃不需要的资源请求
