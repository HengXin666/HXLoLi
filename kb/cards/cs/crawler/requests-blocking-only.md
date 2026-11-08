---
id: cs/crawler/requests-blocking-only
concepts: [requests, 并发控制]
type: decide
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L12]
action: check
---
默认 Transport Adapter 下 requests 不提供任何非阻塞 IO, 文档把 httpx 等列为异步替代
> 默认 Transport Adapter 下 requests 不提供任何非阻塞 IO, 文档自己把 httpx 等列为异步替代
动作:
- [ ] 需要异步并发时选 httpx 等异步库, 不在 requests 上做
