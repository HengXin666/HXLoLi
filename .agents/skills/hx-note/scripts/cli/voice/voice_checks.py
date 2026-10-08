"""检测器的实现层: 掩码、各类 checker、CHECKERS 注册表。

每个 checker 接收掩码后的正文, 返回 [(行号, 说明)]。
"""
from __future__ import annotations

import re

from voice.voice_rules import ABSTRACT_NOUNS


# 掩码: 把 frontmatter / 代码块 / 行内代码 / URL / 链接目标换成等长空格,
# 这样正则不会误伤代码, 而偏移量仍能换算回原始行号。
MASK_PATTERNS = [
    re.compile(r"\A---[ \t]*\r?\n.*?\r?\n---[ \t]*(?:\r?\n|\Z)", re.S),  # frontmatter
    re.compile(r"^```.*?^```[^\n]*$", re.S | re.M),                       # 围栏代码块
    re.compile(r"`[^`\n]+`"),                                             # 行内代码
    re.compile(r"https?://\S+"),                                          # 裸 URL
    re.compile(r"\]\([^)\n]*\)"),                                         # 链接/图片目标
]

def mask(text: str) -> str:
    chars = list(text)
    for pat in MASK_PATTERNS:
        for m in pat.finditer("".join(chars)):
            for i in range(m.start(), m.end()):
                if chars[i] != "\n":
                    chars[i] = " "
    return "".join(chars)

def line_of(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1

def _visible_lines(masked: str) -> list[str]:
    return masked.splitlines()

def check_bold_bullet(masked: str, limit: int) -> list[tuple[int, str]]:
    bullets = [(i + 1, ln) for i, ln in enumerate(masked.splitlines())
               if re.match(r"^\s*[-*+]\s+", ln)]
    if len(bullets) < 4:
        return []
    boldish = [b for b in bullets if re.match(r"^\s*[-*+]\s+\*\*[^*]+\*\*\s*[::]", b[1])]
    pct = len(boldish) * 100 // len(bullets)
    if pct <= limit:
        return []
    return [(boldish[0][0], f"{len(boldish)}/{len(bullets)} 个列表项都是 `- **标题**:` 格式 ({pct}%)")]

def check_bold_density(masked: str, limit: int) -> list[tuple[int, str]]:
    body = [ln for ln in masked.splitlines() if ln.strip()]
    if len(body) < 10:
        return []
    bolds = len(re.findall(r"\*\*[^*\n]+\*\*", masked))
    pct = bolds * 100 // len(body)
    if pct <= limit:
        return []
    return [(1, f"{bolds} 处加粗 / {len(body)} 行有效正文 ({pct}%)")]

EMOJI_RE = re.compile(
    "[\U0001F300-\U0001FAFF\U00002600-\U000027BF\U0001F000-\U0001F2FF\U0000FE0F]"
)

def check_emoji(masked: str, limit: int) -> list[tuple[int, str]]:
    hits = EMOJI_RE.findall(masked)
    if len(hits) <= limit:
        return []
    return [(1, f"{len(hits)} 个 emoji, 上限 {limit}: {''.join(hits[:8])}")]

def check_long_para(masked: str, limit: int) -> list[tuple[int, str]]:
    out: list[tuple[int, str]] = []
    lineno = 0
    for raw in masked.splitlines():
        lineno += 1
        ln = raw.strip()
        if not ln or ln.startswith(("#", ">", "|", "-", "*", "+")) or re.match(r"^\d+\.", ln):
            continue
        n = len(re.sub(r"\s", "", ln))
        if n > limit:
            out.append((lineno, f"单段 {n} 字, 上限 {limit}"))
    return out

# 句长: 依据真人口播语料的实测分布 (中位 29 字 / p90 63), 本仓中文正文用半角句点结尾。
def check_long_sentence(masked: str, limit: int) -> list[tuple[int, str]]:
    """找出过长的句子 (以中英文句末标点切分)。"""
    out: list[tuple[int, str]] = []
    for lineno, raw in enumerate(masked.splitlines(), 1):
        ln = raw.strip()
        if not ln or ln.startswith(("#", ">", "|", "```")):
            continue
        ln = re.sub(r"^\s*(?:[-*+]|\d+[.)])\s+", "", ln)
        if not ln:
            continue
        # 关键: 本仓库的中文正文用**半角句点**结尾 (标点规范把 。 归一成 .),
        # 所以切句必须同时认全角与半角, 否则整段会被当成一句、全是误报。
        # 实测教训: 只认 。 时, 32 篇人写笔记里 31 篇被误报共 700 处。
        for piece in re.split(r"(?<=[。！？!?;；])|(?<=[.])(?=\s|$)", ln):
            t = piece.strip()
            if not t:
                continue
            n = len(re.sub(r"\s", "", t))
            if n > limit:
                head = t[:30] + ("..." if len(t) > 30 else "")
                out.append((lineno, "单句 %d 字 (上限 %d): %s" % (n, limit, head)))
    return out

def check_abstract_density(masked: str, limit: int) -> list[tuple[int, str]]:
    """按每千段算抽象名词密度 (段落 = 非空正文行)。"""
    paras: list[int] = []
    for raw in masked.splitlines():
        ln = raw.strip()
        if not ln or ln.startswith(("#", ">", "|", "```")):
            continue
        paras.append(len(re.findall(ABSTRACT_NOUNS, ln)))
    if len(paras) < 20:
        return []
    per_1k = sum(paras) * 1000 / len(paras)
    if per_1k <= limit:
        return []
    return [(1, "抽象名词密度 %.0f/千段 (上限 %d); 手写基线 51, AI 379" % (per_1k, limit))]

# 段落内出现列表项/缩进续行/未闭合行内 code 时, 它不是"被折行的正文段落"。
LIST_ITEM_RE = re.compile(r"^\s*(?:[-*+]|\d+[.)])\s")

def check_hard_wrap(masked: str, _limit: int) -> list[tuple[int, str]]:
    """检测中文正文里被手工折行的段落。

    原因 (已实测): Markdown 的软换行 (段落内单个 \n) 在渲染时被折叠成**空格**。
    中文不用空格断词, 所以折行会在成品里凭空造出空格, 还会让行尾多出孤字。
    判据: 折点在句中间 (上一行不以句末标点收尾) 才算真折行。

    实测规模: 模型连续产出的 38 个 md 文件里带 81 处硬折行  是稳定的默认行为。
    """
    out: list[tuple[int, str]] = []
    lines = masked.splitlines()
    i = 0
    while i < len(lines):
        ln = lines[i].strip()
        if not ln or ln.startswith(("#", ">", "|", "-", "*", "+", "`", "~", "<")):
            i += 1
            continue
        if re.match(r"^\d+[.)]\s", ln):
            i += 1
            continue
        # 收集本段 (到空行为止)
        block = [ln]
        j = i + 1
        while j < len(lines) and lines[j].strip():
            block.append(lines[j].strip())
            j += 1
        if len(block) > 1:
            # 排除: 段内有列表项 / 有缩进续行 / 有未闭合行内 code
            has_list = any(LIST_ITEM_RE.match(x) for x in block)
            code_open = any(x.count("`") % 2 == 1 for x in block[:-1])
            if not has_list and not code_open:
                if any(not re.search(r"[。！？；:.!?;]$", x) for x in block[:-1]):
                    out.append((i + 1, f"{len(block)} 行被折行 (跨 {sum(len(x) for x in block)} 字), "
                                       f"应合并为一行"))
        i = j if j > i else i + 1
    return out

def check_table(masked: str, _limit: int) -> list[tuple[int, str]]:
    """按"表格"计数而不是按"表格行"计数, 否则一张大表会刷出几十条命中。"""
    out: list[tuple[int, str]] = []
    start = 0
    rows = 0
    for i, ln in enumerate(masked.splitlines(), start=1):
        if re.match(r"^\s*\|.*\|\s*$", ln):
            if rows == 0:
                start = i
            rows += 1
            continue
        if rows:
            out.append((start, f"一张 {rows} 行的表格"))
            rows = 0
    if rows:
        out.append((start, f"一张 {rows} 行的表格"))
    return out

CHECKERS = {
    "bold_bullet": check_bold_bullet,
    "bold_density": check_bold_density,
    "emoji": check_emoji,
    "long_para": check_long_para,
    "long_sentence": check_long_sentence,
    "abstract_density": check_abstract_density,
    "hard_wrap": check_hard_wrap,
    "table": check_table,
}
