# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""AI 味静态检测器 + 可增长表达语料库。

设计前提: "去 AI 味" 如果只写成一句要求, 模型会自评通过。所以判据必须是
**可机械检测的**, 并且阈值来自这个仓库的真实证据 (作者 33 篇 blog 全文只用过
2 次破折号 —— 而 AI 生成的 ai-docs 正文用了 300 多次)。

三条子命令:
  lint     按 profile 检查文件, 命中 E 级规则则 exit 1
  learn    把一条"好/坏表达"追加进项目级语料库 ai-docs/.hx-voice.toml
  samples  打印语料库里已积累的表达 (写作前读, 盲审后补)

profile 决定用哪套规则:
  atom     .hx-info.md  面向知识库的原子知识点 (最严: 不许铺垫/反问/第一人称/配图)
  article  index.md     面向人类的派生文章
  blog     blog/*.md    随笔 (最松, 只查 AI 套话)

规则与阈值是**数据**不是代码: RULES 表里改一行就能调, ai-docs/.hx-voice.toml
里加一条 [[bad]] 就能长出新规则, 都不需要改逻辑。
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

VOICE_DB_CANDIDATES = ("ai-docs/.hx-voice.toml", ".hx-voice.toml")

E, W = "E", "W"


@dataclass
class Rule:
    rid: str
    sev: str
    profiles: tuple[str, ...]
    why: str
    pattern: str | None = None
    flags: int = 0
    # 自定义检查器名 (pattern 为 None 时使用)
    checker: str | None = None
    limit: int = 0


# --- 套话词表 -------------------------------------------------------------
# 阈值来自双语料对比: 对 blog/ + docs/ (作者手写) 与 ai-docs/ (AI 生成) 分别
# 计数, 只有"人写语料里近乎不出现、AI 语料里出现"的词才进 HARD 表。
# 校准结果举例: `总之` 人写 18 次 / AI 0 次 -> 这是作者的词, 不进表;
#               `本质上` 人写 3 / AI 6 -> 证据不足, 放 SOFT 只告警。
# 新增词前先跑一遍对比统计, 不要凭感觉往里堆。
CLICHE_HARD = [
    "值得一提的是", "需要指出的是", "不难发现", "显而易见",
    "综上所述", "总而言之", "总的来说", "概括来说", "一言以蔽之",
    "换句话说", "从某种意义上", "在某种程度上", "归根结底",
    "标志着", "凸显了", "赋能", "前所未有", "不可或缺", "至关重要", "极大地",
    "显著提升", "深度整合", "全新升级", "深刻洞察", "宝贵经验",
    "在当今", "在如今", "随着.{0,12}的不断发展", "随着.{0,12}的日益",
    "让我们一起", "希望这", "希望对你有帮助", "以上就是",
    "未来可期", "前景光明", "大有可为", "拭目以待",
    "堪称", "无疑是", "俨然", "可谓是",
    "深入浅出", "干货满满", "保姆级", "一文读懂", "彻底搞懂",
]
# 人写语料里也有少量用例, 只告警不拦. 想升级成 E 就先补证据。
CLICHE_SOFT = ["值得注意的是", "也就是说", "本质上", "某种意义上"]

# 句长上限。依据: 三类语料的实测分布 (2026-09-26) --
#   作者手写 blog   1752 句: 中位 17 / 均值 22 / p90 43
#   作者手写 docs  32781 句: 中位 25 / 均值 30 / p90 58
#   ai-docs (AI)    3177 句: 中位 34 / 均值 39 / p90 71
#
# 手写与 AI 生成差 2 倍, 这是真信号。阈值取作者手写 blog 的 p90 = 43:
# 意味着 90% 的手写句子在这以下。
#
# 踩坑记录 (两轮, 都要记住):
#   1. 先拍 60 时 32 篇人写笔记 31 篇被误报 -- 阈值不能拍, 必须来自分布。
#   2. 切句只认全角句号时整段被当成一句, 得出中位 90 的假数据;
#      本仓库的中文正文用【半角句点】结尾 (标点规范把全角句号归一成 .)。
# 报 W 不报 E: 长句本身不是错, 只是提示这句读起来要喘一口气。
SENTENCE_SOFT_LIMIT = 43

# -- 抽象名词密度: 唯一 AI 显著超标的一项 -----------------------------
#
# 实测 (2026-09-26, 手写 801 段 vs AI 803 段, 每千段计数):
#   抽象名词  手写 51.2  |  AI 379.8   -> AI 是 7.4 倍 (唯一正向超标)
#   感叹句    手写 224.7 |  AI 7.5      -> AI 只有 1/30
#   第一人称  手写 389.5 |  AI 36.1     -> 1/11
#   疑问句    手写 147.3 |  AI 18.7     -> 1/8
#
# 「读起来像说明书」的直接原因就是第一项: 手写在讲一件具体的事, AI 在陈述一个机制。
# 其余三项 (没人称/没情绪/没问句) 是同一件事的另一面, 但无法靠多写几个我来修好 ——
# 那是选材问题不是措辞问题, 所以这里只检测最能自证的一项。
#
# 阈值取 200/千段: 手写是 51, 留 4 倍余量, 避免误伤本来就偏理论的内容。
#
# 实测区分度 (2026-09-26):
#   手写 blog  0 / 42 篇命中
#   ai-docs   29 / 32 篇命中
#
# 踩坑: 常量必须定义在 RULES 表**之前** —— 定义在后面时模块导入就 NameError。
# 同一坑在本文件踩了两次 (另一个是 SENTENCE_SOFT_LIMIT)。
ABSTRACT_NOUN_SOFT_PER_1K = 200

# 词表: **只收「在技术语境里几乎总是空壳」的那类。**
#
# 第一版词表包含了 方法/路径/结构/模型/框架/模式 —— 那是错的, 用真人文章对照时被证伪:
#   edsionte《Linux内核中通过文件描述符获取绝对路径》全文命中旧词表 700/千段,
#   逐条看上下文, 命中的全是「绝对路径」「文件系统数据结构」「方法一/方法二」——
#   **这些在该语境里都指具体的东西**, 不是空壳。
#   换成现在这张表后, 那篇是 0。
#
# 对照结果 (每千行正文, **口径: 去 frontmatter 与代码块, 正文行 >20 字**):
#   真人技术文章 5 篇:  17 / 9 / 0 / 18 / 11      <- 上限约 18
#   我写的 MCP 那篇:    765 -> 183 (换表后)        <- 仍高约 10 倍
#   ai-docs 32 篇:      旧表命中 30 篇 -> 新表命中 9 篇
#
# **引用这些数时必须连口径一起写** —— 它们对切分口径敏感:
#   换一种口径 (分母取全部非空行, 不去代码块) 会得到另一组值 (曾得到 0/23/13/9/18)。
#   两版都曾被写进文档, 造成过前后不一致, 所以这里锚死一种口径。
#
# 判据: 把它换成"具体发生了什么", 句子会变好还是变空?
#   会变好 (机制 -> 每次请求都重新握手)  -> 收进表
#   换不了 (绝对路径 / 方法一 / 数据结构) -> 不收, 因为它本来就是具体的
#
# **但这条判据有个边界, 必须知道**: 同一个词在不同文章里性质不同。
#   「能力」在 MCP 协议那篇里指 capabilities 字段 (具体术语), 在别的文章里可能是空壳。
#   词表永远抓不准这一层 —— 所以这个规则**只当提示, 不当判据**。
#   它报出来时正确的反应是"看一眼这个词在这里是术语还是空壳", 不是"必须删掉"。
#
# 更可靠的替代口径 (未实现, 留给以后): 算**同词重复率** —— 一个抽象名词如果在同一篇里
# 反复出现且每次都在不同语境, 多半是空壳; 如果是同一个术语在用 (如 capabilities),
# 反而说明它在讲具体的东西。
ABSTRACT_NOUNS = (
    r"机制|链路|形态|维度|范式|要素|场景|体系|流程|闭环|抓手"
    r"|赋能|生态|矩阵|组合拳|层面|视角|定位|方法论|语义层"
)


RULES: list[Rule] = [
    # ---- 三大句式指纹 (证据最强, 一律 E) ----
    Rule("em-dash", E, ("atom", "article", "blog"),
         "破折号是最强的 AI 指纹: 作者 33 篇 blog 全文只用过 2 次, 改用 半角括号 / ... / 单独一句",
         pattern=r"—"),
    Rule("not-but", E, ("atom", "article", "blog"),
         '"不是X而是Y" 是公认的 AI 口癖. 直接说 Y 是什么, 不要先否定一个没人说过的 X',
         pattern=r"不(?:是|只是|仅是|仅仅是|光是)[^。.!?\n]{0,30}?而(?:是|在于)"),
    Rule("not-but2", E, ("atom", "article", "blog"),
         '"不仅...更是" 同上, 属于同一族套话',
         pattern=r"不仅(?:仅)?[^。.!?\n]{0,30}?(?:更是|更在于|还是)"),
    Rule("cliche", E, ("atom", "article", "blog"),
         "套话: 删掉它句子信息量不变, 说明它只是在占位",
         pattern="|".join(CLICHE_HARD)),
    Rule("cliche-soft", W, ("atom", "article"),
         "疑似套话 (人写语料里也有少量用例): 确认它真的承载了信息再留",
         pattern="|".join(CLICHE_SOFT)),
    Rule("assistant-leak", E, ("atom", "article", "blog"),
         "助手口头禅漏进正文, 这是对话残留不是文章内容",
         pattern=r"希望(?:这|以上|本文)|如果(?:你|您)(?:还|有)(?:其他|任何)|随时(?:告诉我|问我)|我(?:已经|将)为(?:你|您)"),

    # ---- 排版指纹 ----
    # 数词故意从「三」起: 「一块假栈」「两个线程」里的「一/两」是普通量词,
    # 把它们算进来会把正常中文全判成凑数排比 (实测误报)。真正的凑数排比都是三及以上。
    Rule("hype-heading", W, ("article", "blog"),
         "凑数式排比小标题 (三大/四个/五块), 通常是为了排版整齐硬凑的",
         pattern=r"^#{2,6}[^\n]*?[三四五六七]\s*(?:大|个|层|条|块|点|方面|维度|理由|信号)[^\n]*$",
         flags=re.M),
    Rule("bold-bullet", W, ("article", "blog"),
         "列表项一律 `- **粗体标题**:` 开头是 AI 排版指纹, 人写的列表参差不齐",
         checker="bold_bullet", limit=50),
    Rule("bold-density", W, ("atom", "article", "blog"),
         "加粗密度过高: 全都重点等于没有重点",
         checker="bold_density", limit=25),
    Rule("emoji", W, ("article",),
         "装饰性 emoji: 作者在技术文里只偶发用 1 个 (随笔里随便用, 所以 blog profile 不查)",
         checker="emoji", limit=3),
    Rule("table", E, ("article",),
         "派生文章默认禁表格 (用 --allow-table 放开). 表格容易替代思考, 把该讲清的推理压成格子",
         checker="table"),
    Rule("long-para", W, ("atom", "article", "blog"),
         "段落过长: 作者的段落几乎一句一段, 大段文本是排版事故",
         checker="long_para"),
    Rule("abstract-density", W, ("article",),
         "抽象名词密度过高: 手写千段 51 个, AI 千段 379 个. 这是在陈述机制而不是在讲一件事",
         checker="abstract_density", limit=ABSTRACT_NOUN_SOFT_PER_1K),
    Rule("long-sentence", W, ("article",),
         "句长: 作者手写的中位 17 字, AI 生成的中位 34 字. 超过 43 字读起来要喘一口气",
         checker="long_sentence", limit=SENTENCE_SOFT_LIMIT),

    Rule("hard-wrap", E, ("atom", "article", "blog"),
         "中文正文被手工折行: Markdown 软换行在渲染时变成**空格**, 中文会凭空多出空格。"
         "一段话写成一行 (列表/代码块按各自语法换行不受此限)",
         checker="hard_wrap"),

    # ---- 过程泄漏 ----
    Rule("meta-leak", E, ("atom", "article"),
         "生成过程/工具链/本地路径泄漏进产物",
         pattern=r"本文(?:将|会|首先)?(?:介绍|讲解|探讨|带你)|AI\s*(?:辅助|生成)|\bTODO\b|/Users/|\.agents/|uv run|makeDoc\.py"),

    # ---- atom 专属: 知识库纯洁度 ----
    Rule("atom-image", E, ("atom",),
         ".hx-info.md 是喂给检索的纯文本, 图片在这里是噪声 (配图属于 index.md)",
         pattern=r"!\["),
    Rule("atom-person", E, ("atom",),
         "原子知识点不带人称: 检索命中的是知识, 不是谁在说话",
         pattern=r"(?<![A-Za-z])(?:我们|我的|我|咱|你可以|你需要|大家)(?![A-Za-z])"),
    Rule("atom-rhetorical", E, ("atom",),
         "设问/反问是给人类做铺垫的, 检索时它只会污染这一条知识点",
         pattern=r"[?？]\s*$", flags=re.M),
    Rule("atom-transition", E, ("atom",),
         "递进连接词 (首先/其次/因此...) 说明这条知识点依赖上下文, 违反自包含",
         pattern=r"^\s*(?:首先|其次|再次|然后|接下来|最后|另外|此外|而且|因此|所以|不过|但是|同时)[,，、]",
         flags=re.M),
    Rule("atom-hedge", E, ("atom",),
         "模糊限定词: 确定就断言, 不确定就写 [不确定], 不要留下 可能/也许/大概 这类对冲",
         pattern=r"(?:可能|也许|或许|大概|似乎|应该是|差不多|大致上)(?!\])"),

    # ---- article 专属: 共鸣段的硬约束 ----
    Rule("second-person-hook", E, ("article",),
         "作者的博客是第一人称自述, 不对读者寒暄. 禁止 你最近/相信你/如果你也 这类开场",
         pattern=r"(?:相信|如果|也许)(?:你|您)(?:也|还|一定|可能)|(?:你|您)(?:最近|可能已经|一定)"),
]


# --------------------------------------------------------------------------
# 掩码: 把 frontmatter / 代码块 / 行内代码 / URL / 链接目标 换成等长空格,
# 这样正则不会误伤代码, 而偏移量仍能换算回原始行号。
# --------------------------------------------------------------------------
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


# --- checker 实现 ----------------------------------------------------------
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


# 只算真正的表情/符号 emoji。箭头 (→ ↑ ⇒) 是作者常用的技术记号, 不算装饰。
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


# ── 句长: 从真人口播语料实测出来的判据 ──────────────────────────────
#
# 2026-09-26 用「渡劫C++协程」第 7 讲的 ASR 转写做量化对比 (215 句):
#   真人口播: 句长中位 29 字 / 均值 34 / p90 63 / 最长 120
#   我写的稿: 句长中位 50 字 / 均值 62 / p90 113 / 最长 385
#
# 差 40% 不是用词问题, 是文体问题 —— 书面语允许长句叠从句, 口播不允许,
# 因为听的人没法回看。长句是 AI 味里最稳定、最容易量的一个信号。
#
# 阈值 60: 真人 p90 是 63 (正常讲述偶尔会到), 而我的稿均值就 62。
# 报 W 不报 E —— 长句本身不是错, 只是提示这句读起来要喘一口气。
#
# (see .agents/notes/implemented/architecture/2026-09-26-sentence-length-gate.md — 阈值为什么必须来自分布而不是凭感觉定)


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
# 实测教训: 人写笔记里的折行段几乎全是这类误报, 折点落在语义边界上。
LIST_ITEM_RE = re.compile(r"^\s*(?:[-*+]|\d+[.)])\s")


def check_hard_wrap(masked: str, _limit: int) -> list[tuple[int, str]]:
    """检测中文正文里被手工折行的段落。

    原因 (已实测): Markdown 的软换行 (段落内单个 \n) 在渲染时被折叠成**空格**。
    中文不用空格断词, 所以折行会在成品里凭空造出空格, 还会让行尾多出孤字。
    判据: 折点在句中间 (上一行不以句末标点收尾) 才算真折行。

    实测规模: 模型连续产出的 38 个 md 文件里带 81 处硬折行 —— 是稳定的默认行为。
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

# 段落长度上限。取值依据: 27 篇站内人写笔记的 858 个正文段落实测分布 ——
#   p50=64 / p75=109 / p90=166 / p95=204 / p99=340 / 最大 1092
# article 原定 160 落在 p90, 意味着约 10% 的人写段落都会被报 (阈值偏严);
# 改取 p95=204 作提示线, 只抓真正异常的少数。
# atom 仍取 80: 知识稿的段落本就该短, 且 atom 是给检索用的, 不该有大段。
PARA_LIMIT = {"atom": 80, "article": 204, "blog": 260}


# --- 项目级语料库 ---------------------------------------------------------
def find_voice_db(start: Path | None = None) -> Path:
    cur = (start or Path.cwd()).resolve()
    while True:
        for rel in VOICE_DB_CANDIDATES:
            cand = cur / rel
            if cand.is_file():
                return cand
        if cur.parent == cur:
            break
        cur = cur.parent
    # 不存在时返回首选写入位置
    cur = (start or Path.cwd()).resolve()
    while True:
        if (cur / "ai-docs").is_dir():
            return cur / "ai-docs" / ".hx-voice.toml"
        if cur.parent == cur:
            return (start or Path.cwd()).resolve() / ".hx-voice.toml"
        cur = cur.parent


def _mini_toml_arrays(text: str) -> dict:
    """只认 [[bad]] / [[good]] 与其下的 key = "value"。

    存在的理由: 语料库是这个 skill 的记忆, 解析失败就等于把积累过的表达
    静默丢掉 —— 比报错更坏。tomllib 要 Python 3.11, 而 `python3` 可能是 3.9。
    """
    out: dict[str, list[dict]] = {"bad": [], "good": []}
    cur: dict | None = None
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        m = re.match(r"^\[\[(bad|good)\]\]$", s)
        if m:
            cur = {}
            out[m.group(1)].append(cur)
            continue
        if cur is None:
            continue
        kv = re.match(r'^([A-Za-z_]+)\s*=\s*"(.*)"\s*$', s)
        if kv:
            cur[kv.group(1)] = kv.group(2).replace('\\"', '"').replace("\\n", "\n").replace("\\\\", "\\")
    return out


def load_voice_db(path: Path) -> dict:
    if not path.is_file():
        return {"bad": [], "good": [], "meme": []}
    raw = path.read_text(encoding="utf-8")
    try:
        import tomllib
        data = tomllib.loads(raw)
        return {"bad": data.get("bad", []) or [], "good": data.get("good", []) or [], "meme": data.get("meme", []) or []}
    except ImportError:
        return _mini_toml_arrays(raw)
    except Exception as exc:  # noqa: BLE001
        print(f"warn: 语料库 TOML 有语法错误 ({exc}), 退化为宽松解析", file=sys.stderr)
        return _mini_toml_arrays(raw)


def _toml_str(s: str) -> str:
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'


def cmd_learn(args) -> int:
    path = Path(args.db) if args.db else find_voice_db()
    if getattr(args, "meme", None):
        kind, text = "meme", args.meme
    elif args.bad:
        kind, text = "bad", args.bad
    else:
        kind, text = "good", args.good
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.is_file():
        path.write_text(
            "# HXLoLi 表达语料库 (可增长)\n"
            "# 由 hx_voice.py learn 追加; 写作前读 `hx_voice.py samples`, 盲审后补 learn。\n"
            "# [[bad]] 带 pattern 时会被 lint 当成额外的 E 级规则。\n"
            "schema_version = 1\n",
            encoding="utf-8",
        )
    block = [f"\n[[{kind}]]", f"text = {_toml_str(text)}"]
    if args.why:
        block.append(f"why = {_toml_str(args.why)}")
    if args.pattern:
        try:
            re.compile(args.pattern)
        except re.error as exc:
            print(f"error: pattern 不是合法正则: {exc}", file=sys.stderr)
            return 2
        block.append(f"pattern = {_toml_str(args.pattern)}")
    if args.source:
        block.append(f"source = {_toml_str(args.source)}")
    with path.open("a", encoding="utf-8") as fh:
        fh.write("\n".join(block) + "\n")
    print(f"learned[{kind}] -> {path}")
    return 0


def cmd_samples(args) -> int:
    path = Path(args.db) if args.db else find_voice_db()
    db = load_voice_db(path)
    if args.json:
        print(json.dumps({"db": str(path), **db}, ensure_ascii=False, indent=2))
        return 0
    print(f"# 语料库: {path}")
    for kind, label in (
        ("good", "值得模仿"),
        ("bad", "必须避免"),
        ("meme", "梗 / 热词 (只用作者确认过的; 编错会变成新的 AI 味)"),
    ):
        items = db.get(kind, [])
        if not items:
            continue
        print(f"\n## {label} ({len(items)})")
        for it in items:
            why = f"   <- {it['why']}" if it.get("why") else ""
            print(f"- {it.get('text', '')}{why}")
    return 0


# --- lint ----------------------------------------------------------------
@dataclass
class Hit:
    rid: str
    sev: str
    line: int
    text: str
    why: str


@dataclass
class FileReport:
    path: str
    profile: str
    hits: list[Hit] = field(default_factory=list)

    @property
    def errors(self) -> int:
        return sum(1 for h in self.hits if h.sev == E)

    @property
    def warns(self) -> int:
        return sum(1 for h in self.hits if h.sev == W)


def lint_file(path: Path, profile: str, *, allow_table: bool, db: dict) -> FileReport:
    text = path.read_text(encoding="utf-8")
    masked = mask(text)
    rep = FileReport(path=str(path), profile=profile)

    for rule in RULES:
        if profile not in rule.profiles:
            continue
        if rule.rid == "table" and allow_table:
            continue
        if rule.checker:
            limit = PARA_LIMIT[profile] if rule.checker == "long_para" else rule.limit
            for lineno, detail in CHECKERS[rule.checker](masked, limit):
                rep.hits.append(Hit(rule.rid, rule.sev, lineno, detail, rule.why))
            continue
        for m in re.finditer(rule.pattern, masked, rule.flags):
            frag = m.group(0).strip()
            if not frag:
                continue
            rep.hits.append(Hit(rule.rid, rule.sev, line_of(text, m.start()),
                                frag[:60], rule.why))

    for item in db.get("bad", []):
        pat = item.get("pattern")
        if not pat:
            continue
        try:
            rx = re.compile(pat, re.M)
        except re.error:
            continue
        for m in rx.finditer(masked):
            rep.hits.append(Hit("learned", E, line_of(text, m.start()),
                                m.group(0).strip()[:60],
                                item.get("why") or f"语料库标记为坏表达: {item.get('text', '')}"))

    rep.hits.sort(key=lambda h: (h.line, h.rid))
    return rep


def guess_profile(path: Path) -> str:
    name = path.name
    if name == ".hx-info.md":
        return "atom"
    if "/blog/" in path.as_posix():
        return "blog"
    return "article"


def cmd_lint(args) -> int:
    db = load_voice_db(Path(args.db) if args.db else find_voice_db())
    reports: list[FileReport] = []
    for raw in args.paths:
        p = Path(raw)
        if not p.is_file():
            print(f"error: 文件不存在: {p}", file=sys.stderr)
            return 2
        prof = args.profile or guess_profile(p)
        if prof not in ("atom", "article", "blog"):
            print(f"error: 未知 profile: {prof}", file=sys.stderr)
            return 2
        reports.append(lint_file(p, prof, allow_table=args.allow_table, db=db))

    if args.json:
        print(json.dumps({
            "files": [{
                "path": r.path, "profile": r.profile,
                "errors": r.errors, "warnings": r.warns,
                "hits": [h.__dict__ for h in r.hits],
            } for r in reports],
            "ok": all(r.errors == 0 for r in reports),
        }, ensure_ascii=False, indent=2))
    else:
        # 按规则分组渲染。**一条规则一行, 不逐命中刷屏。**
        # 实测教训: 逐条打印时, 20 处 long-sentence 会占 40 行, 而其中 39 行的说明是同一句。
        # 那条说明只需要看一次 —— 重复 20 遍是纯占上下文。
        for r in reports:
            print(f"\n=== {r.path}  [profile={r.profile}]")
            if not r.hits:
                print("  (无命中)")
                continue
            by_rule: dict[str, list] = {}
            for h in r.hits:
                by_rule.setdefault(h.rid, []).append(h)
            # 组内按行号排序; 组间按 (严重度, 首个行号) 排, E 在前
            for rid, hits in sorted(
                by_rule.items(),
                key=lambda kv: (kv[1][0].sev != "E", kv[1][0].line),
            ):
                hits.sort(key=lambda h: h.line)
                sev = hits[0].sev
                lines = ", ".join(str(h.line) for h in hits[:24])
                more = "" if len(hits) <= 24 else f" ...(+{len(hits) - 24})"
                print(f"  {sev} [{rid}] {len(hits)} 处 @ 行 {lines}{more}")
                print(f"       {hits[0].why}")
                # 每种规则只额外展示前 2 条命中内容, 够定位就行
                for h in hits[:2]:
                    print(f"         {h.line}: {h.text[:70]}")
                if len(hits) > 2:
                    print("         ... 其余同类, 按上面的行号自查")
            print(f"  {r.errors} errors, {r.warns} warnings")
        total_e = sum(r.errors for r in reports)
        total_w = sum(r.warns for r in reports)
        status = "PASS" if total_e == 0 else "FAIL"
        print(f"\n{status}  {total_e} errors, {total_w} warnings")

    if args.soft:
        return 0
    return 1 if any(r.errors for r in reports) else 0


def unwrap_text(text: str) -> tuple[str, int]:
    """把中文正文里被手工折行的段落合并成一行。返回 (新文本, 合并处数)。

    合并时按作者既有的空格约定补空格: 标点 (逗号/句号/冒号/分号/问号/叹号/半角括号) 之后
    接中文时要有一个空格 (HXLoLi 标点规范: `工具, 建议` `注意: 这里`)。只在拉丁字母/数字相邻
    时才补是不够的 —— 那会产出 `线程,自己带一段` 这种与全篇不一致的写法。

    **必须跳过 frontmatter。** 它是一行一个 key 的 YAML, 按段落合并会把
    `hxid: "..."` 与 `title: "..."` 压成一行, 产出非法 YAML —— 实测踩过这个坑。
    """
    # frontmatter 原样保留, 不参与合并
    fm = ""
    m = re.match(r"\A---\n.*?\n---\n", text, flags=re.S)
    if m:
        fm, text = m.group(0), text[m.end():]
    lines = text.split("\n")
    out: list[str] = []
    i = 0
    merged = 0
    in_fence = False
    while i < len(lines):
        ln = lines[i]
        s = ln.strip()
        # 围栏行本身: 原样输出, 翻转状态
        if s.startswith('```') or s.startswith("~~~"):
            in_fence = not in_fence
            out.append(ln)
            i += 1
            continue
        # 围栏内部: 一个字都不动 (块内换行是内容的一部分)
        if in_fence:
            out.append(ln)
            i += 1
            continue
        if not s or s.startswith(("#", ">", "|", "-", "*", "+", "`", "~", "<")) \
                or re.match(r"^\d+[.)]\s", s):
            out.append(ln)
            i += 1
            continue
        block = [s]
        j = i + 1
        while j < len(lines) and lines[j].strip():
            block.append(lines[j].strip())
            j += 1
        if len(block) > 1:
            has_list = any(LIST_ITEM_RE.match(x) for x in block)
            code_open = any(x.count("`") % 2 == 1 for x in block[:-1])
            if not has_list and not code_open and \
                    any(not re.search(r"[。！？；:.!?;]$", x) for x in block[:-1]):
                joined = block[0]
                for nxt in block[1:]:
                    prev = joined[-1] if joined else ""
                    # 标点后接中文: 补空格 (作者约定)
                    punct = bool(re.match(r"[,.:;?!)]", prev))
                    # 拉丁字母/数字与拉丁字母/数字相邻: 补空格 (真实需要)
                    latin = bool(re.match(r"[A-Za-z0-9`]", prev)) and bool(re.match(r"[A-Za-z0-9(]", nxt[:1]))
                    joined += (" " if (punct or latin) else "") + nxt
                out.append(joined)
                merged += 1
                i = j
                continue
        out.append(ln)
        i += 1
    return fm + "\n".join(out), merged


def cmd_fix(args) -> int:
    total = 0
    for pat in args.paths:
        for p in sorted(Path().rglob(pat)) if any(c in pat for c in "*?[") else [Path(pat)]:
            if not p.is_file():
                print(f"跳过 (不是文件): {p}", file=sys.stderr)
                continue
            raw = p.read_text(encoding="utf-8")
            new, n = unwrap_text(raw)
            if not n:
                continue
            total += n
            if args.write:
                p.write_text(new, encoding="utf-8")
                print(f"  {p}: 合并 {n} 处 (已写入)")
            else:
                print(f"  {p}: 合并 {n} 处 (dry-run)")
    if not total:
        print("没有发现被折行的段落")
    elif not args.write:
        print(f"\n合计 {total} 处; 加 --write 落盘")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="AI 味静态检测 + 表达语料库")
    p.add_argument("--db", help="语料库路径, 默认向上找 ai-docs/.hx-voice.toml")
    sub = p.add_subparsers(dest="cmd", required=True)

    lt = sub.add_parser("lint", help="检查文件的 AI 味")
    lt.add_argument("paths", nargs="+")
    lt.add_argument("--profile", choices=["atom", "article", "blog"],
                    help="不传则按文件名推断 (.hx-info.md -> atom, blog/ -> blog, 其余 article)")
    lt.add_argument("--allow-table", action="store_true", help="放开 article 的禁表格规则")
    lt.add_argument("--soft", action="store_true", help="只报告, 总是 exit 0")
    lt.add_argument("--json", action="store_true")
    lt.set_defaults(func=cmd_lint)

    ln = sub.add_parser("learn", help="把一条表达记进语料库")
    g = ln.add_mutually_exclusive_group(required=True)
    g.add_argument("--bad", help="坏表达原文")
    g.add_argument("--good", help="好表达原文")
    g.add_argument("--meme", help="梗 / 热词 (只用作者确认过的; 机器编不出来, 编错会变成新的 AI 味)")
    ln.add_argument("--why", help="为什么好/为什么坏; 梗填「什么场合用」")
    ln.add_argument("--pattern", help="可选正则; 给了就会变成 lint 的 E 级规则")
    ln.add_argument("--source", help="出处 (文件或链接); 梗必填 (从哪听来的)")
    ln.set_defaults(func=cmd_learn)

    fx = sub.add_parser("fix", help="把中文正文里被手工折行的段落合并为一行")
    fx.add_argument("paths", nargs="+")
    fx.add_argument("--write", action="store_true", help="落盘 (默认 dry-run)")
    fx.set_defaults(func=cmd_fix)

    sp = sub.add_parser("samples", help="打印语料库")
    sp.add_argument("--json", action="store_true")
    sp.set_defaults(func=cmd_samples)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
