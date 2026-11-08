"""电脑包商品页声明检查: 每个声明词必须带上它的测法, 否则报一条.
用法: python claim_lint.py < 商品页文本.txt   退出码 = 未带测法的声明条数
"""
import re, sys

RULES = [  # (声明, 声明正则, 必须同时出现的测法正则)
    ("防水", r"waterproof|weatherproof|water[- ]resistant|防水|防泼水",
             r"IP[X\d]\d|IEC\s*60529|\d+\s*(mm|kPa)\b"),
    ("军规", r"MIL-STD|military[- ]grade|军规|军工",
             r"Method\s*5\d\d"),
    ("容量", r"\b\d+(\.\d+)?\s*(L|liters?|升)\b",
             r"ASTM\s*F2153"),
    ("适配尺寸", r"\b1[0-8](\.\d)?\s*(\"|''|inch|英寸|寸)",
             r"\d{3}\s*[x×]\s*\d{3}\s*mm|\d{2}(\.\d)?\s*[x×]\s*\d{2}(\.\d)?\s*cm"),
    ("面料强度", r"\b\d{2,4}\s*D\b|denier|旦",
             r"ASTM\s*D5034|\d+\s*N\b"),
]

def lint(text: str) -> list[str]:
    out = []
    for name, claim, method in RULES:
        hits = re.findall(claim, text, flags=re.I)
        if hits and not re.search(method, text, flags=re.I):
            out.append(f"{name}: 声明出现 {len(hits)} 次, 未找到测法 /{method}/")
    return out

if __name__ == "__main__":
    problems = lint(sys.stdin.read())
    print("\n".join(problems) or "全部声明都带测法")
    sys.exit(len(problems))
