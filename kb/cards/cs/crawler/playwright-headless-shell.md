---
id: cs/crawler/playwright-headless-shell
concepts: [Playwright, 反爬]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L57, raw/2026-11-08-crawler/material.md#L74]
action: check
---
Playwright 默认无头模式用的是 chromium headless shell, 与 Chrome 的 new headless 不同, 行为会有差异
> Playwright 默认无头模式用的是 chromium headless shell, 与 Chrome 的 new headless (真 Chrome) 不同, 行为会有差异
动作:
- [ ] 不假设上了浏览器就像真人, 先确认用的是哪种无头模式
