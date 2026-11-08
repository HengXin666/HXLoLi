---
id: cs/crawler/curl-cffi-api
concepts: [curl_cffi, requests, HTTP2]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L63, raw/2026-11-08-crawler/material.md#L66]
action: check
---
curl_cffi 提供与 requests 相似的 API 与 `AsyncSession`, 支持 HTTP/2、HTTP/3、websocket
> 提供与 requests 相似的 API 与 `AsyncSession`; 支持 HTTP/2、HTTP/3、websocket
动作:
- [ ] 从 requests 迁移时按相似 API 替换, 异步用 `AsyncSession`
