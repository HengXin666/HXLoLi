---
id: cs/crawler/aiohttp-trust-env-proxy
concepts: [aiohttp, 代理]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L30]
action: check
---
aiohttp 默认不读环境变量代理, 需要 `trust_env=True`
> aiohttp 默认不读环境变量代理, 需要 `trust_env=True`
动作:
- [ ] 需要走环境变量代理时给 `ClientSession` 传 `trust_env=True`
