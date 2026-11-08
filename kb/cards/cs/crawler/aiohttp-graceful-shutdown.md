---
id: cs/crawler/aiohttp-graceful-shutdown
concepts: [aiohttp]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L31]
action: check
---
aiohttp 关闭 session 后底层连接不会立即关, 带 SSL 时文档建议等约 250ms
> 关闭 session 后底层连接不会立即关; 事件循环先停会有 `ResourceWarning`, 带 SSL 时文档建议等约 250ms
动作:
- [ ] 关闭 session 后、停事件循环前等约 250ms, 避免 `ResourceWarning`
