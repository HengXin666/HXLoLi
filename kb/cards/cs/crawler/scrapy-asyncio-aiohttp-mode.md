---
id: cs/crawler/scrapy-asyncio-aiohttp-mode
concepts: [Scrapy, aiohttp]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L45]
action: check
---
`TWISTED_REACTOR_ENABLED=False` 时直接用 asyncio 事件循环, 默认 http/https 下载器换成 aiohttp 实现, 该模式标为实验性
> `TWISTED_REACTOR_ENABLED=False` 时直接用 asyncio 事件循环, 默认 http/https 下载器换成 aiohttp 实现; 该模式标为实验性
动作:
- ? 这个项目能否接受实验性模式
