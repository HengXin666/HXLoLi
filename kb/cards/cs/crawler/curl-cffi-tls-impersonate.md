---
id: cs/crawler/curl-cffi-tls-impersonate
concepts: [curl_cffi, 反爬, HTTP2]
type: decide
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L65]
action: check
---
curl_cffi 可模仿浏览器的 TLS/JA3 与 HTTP/2 指纹, 纯 Python 客户端 (httpx / requests) 做不到
> curl_cffi 绑定 curl-impersonate, 可模仿浏览器的 TLS/JA3 与 HTTP/2 指纹, 纯 Python 客户端 (httpx / requests) 做不到
动作:
- [ ] 需要浏览器 TLS 指纹时选 curl_cffi, 不在 httpx / requests 上调
