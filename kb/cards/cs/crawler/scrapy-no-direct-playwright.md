---
id: cs/crawler/scrapy-no-direct-playwright
concepts: [Scrapy, Playwright]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L49, raw/2026-11-08-crawler/material.md#L76]
action: check
---
在 Scrapy 回调里直接用 playwright-python 会绕过大部分 Scrapy 组件, 官方推荐 scrapy-playwright
> 在 Scrapy 回调里直接用 playwright-python 会绕过大部分 Scrapy 组件 (中间件、去重等), 官方推荐 scrapy-playwright
动作:
- [ ] Scrapy 项目需要浏览器时接 scrapy-playwright, 不在回调里直接调 playwright-python
