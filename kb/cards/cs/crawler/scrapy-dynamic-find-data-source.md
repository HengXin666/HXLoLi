---
id: cs/crawler/scrapy-dynamic-find-data-source
concepts: [Scrapy, 动态页面, Playwright]
type: decide
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L47]
action: check
---
动态加载的内容官方推荐先找到数据源, 实在难复现且不在乎速度时才用无头浏览器
> 动态加载的内容, 官方推荐先找到数据源 (浏览器 Network 面板 → 复现请求, 可 `Request.from_curl()`), 实在难复现且不在乎速度时才用无头浏览器
动作:
- [ ] 先在浏览器 Network 面板找到数据请求并复现
- `Request.from_curl()`
