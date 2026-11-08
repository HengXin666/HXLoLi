---
id: life/bag/claim-lint-rules
concepts: [商家宣称]
type: check
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-bag/claim_lint.py#L1-L16]
action: check
---
每个声明词必须带上它的测法: 防水 IEC 60529 或 mm/kPa, 容量 ASTM F2153, 面料强度 ASTM D5034
> 每个声明词必须带上它的测法, 否则报一条
> ("防水", r"waterproof|weatherproof|water[- ]resistant|防水|防泼水",
> r"IP[X\d]\d|IEC\s*60529|\d+\s*(mm|kPa)\b"),
> ("容量", r"\b\d+(\.\d+)?\s*(L|liters?|升)\b",
> r"ASTM\s*F2153"),
> ("面料强度", r"\b\d{2,4}\s*D\b|denier|旦",
> r"ASTM\s*D5034|\d+\s*N\b"),
动作:
- [ ] 商品页写 weatherproof / 防水: 找 IP 等级、IEC 60529 或 mm / kPa 数值
- [ ] 商品页写 20L 这类容量: 找是否按 ASTM F2153 测
- [ ] 商品页写 1000D 这类旦数: 找 ASTM D5034 或 N 值
