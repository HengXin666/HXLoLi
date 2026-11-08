---
id: cs/crawler/scrapy-playwright-windows
concepts: [Playwright, Scrapy]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L61]
action: check
---
Windows 上 Playwright 必须跑在另一个线程的 ProactorEventLoop, 因为 asyncio reactor 要 SelectorEventLoop
> Windows 上 Playwright 必须跑在另一个线程的 ProactorEventLoop, 因为 asyncio reactor 要 SelectorEventLoop
动作:
- ? scrapy-playwright 是否要跑在 Windows 上
