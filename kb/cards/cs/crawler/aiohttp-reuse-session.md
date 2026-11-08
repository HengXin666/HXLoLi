---
id: cs/crawler/aiohttp-reuse-session
concepts: [aiohttp, 连接池]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L28]
action: check
---
aiohttp 的 `ClientSession` 持有 cookie 存储与连接池, 应复用同一个 session
> `ClientSession` 持有 cookie 存储与连接池, 应复用同一个 session
动作:
- [ ] 整个抓取过程只建一个 `ClientSession`
