---
id: cs/crawler/playwright-network-stack-headers
concepts: [Playwright, 反爬]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L56]
action: check
---
`Cookie`、`Host`、`Sec-Fetch-*` 等请求头由浏览器网络栈在发送前附加, route 处理函数看不到也改不了
> `Cookie`、`Host`、`Sec-Fetch-*` 等请求头由浏览器网络栈在发送前附加, route 处理函数看不到也改不了
动作:
- [ ] 要改这些请求头时不走 route 处理函数
