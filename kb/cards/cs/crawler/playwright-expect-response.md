---
id: cs/crawler/playwright-expect-response
concepts: [Playwright, 动态页面]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L54]
action: check
---
Playwright 可监听页面发出的全部请求与响应, 可用 `page.expect_response("**/api/...")` 在点击后等到某个接口的响应
> Playwright 可监听页面发出的全部请求与响应 (含 XHR / fetch), 可用 `page.expect_response("**/api/...")` 在点击后等到某个接口的响应
动作:
- [ ] 点击触发加载的页面用 `page.expect_response` 等目标接口的响应
