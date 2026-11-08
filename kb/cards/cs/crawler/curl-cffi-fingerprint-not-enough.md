---
id: cs/crawler/curl-cffi-fingerprint-not-enough
concepts: [curl_cffi, 反爬]
type: pitfall
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L72]
action: check
---
指纹模仿只解决 TLS/HTTP2 这一层, Akamai / DataDome / Kasada / Incapsula 还要另外的 token
> 对现代反爬, 只做 TLS 指纹不够, Akamai / DataDome / Kasada / Incapsula 还要另外的 token
> 属广告性质, 仅作反例线索
> 结论: 指纹模仿只解决 TLS/HTTP2 这一层
动作:
- [ ] 目标站用这类反爬时, 不指望只换指纹库就能过
