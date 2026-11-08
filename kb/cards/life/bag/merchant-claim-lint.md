---
id: life/bag/merchant-claim-lint
concepts: [商家宣称]
type: cmd
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-bag/claim_lint.py#L1-L2, raw/2026-11-08-bag/material.md#L49]
action: cmd
tested: 2026-11-09 本地 Python 3.12, 对 peakdesign_everyday_backpack.txt 报防水、容量两条, 退出码 2
---
电脑包商品页声明检查: 每个声明词必须带上它的测法, 否则报一条
> 电脑包商品页声明检查: 每个声明词必须带上它的测法, 否则报一条
> two weatherproof UltraZips
动作:
- `python3 kb/raw/2026-11-08-bag/claim_lint.py < 商品页文本.txt`
- [ ] 退出码 = 未带测法的声明条数, 非 0 时逐条找商家要测法
