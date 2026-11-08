---
id: cs/crawler/httpx-default-timeout-5s
concepts: [httpx, HTTP超时]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L19, raw/2026-11-08-crawler/provenance.md#L34]
action: check
---
httpx 默认对一切网络操作强制超时, 网络无活动 5 秒抛 `TimeoutException`
> httpx 默认对一切网络操作强制超时: 网络无活动 5 秒抛 `TimeoutException`
动作:
- [ ] 目标接口可能 5 秒以上无数据时显式传 `httpx.Timeout`
