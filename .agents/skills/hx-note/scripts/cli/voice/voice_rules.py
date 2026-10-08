"""AI 味规则的**数据层**: 词表、阈值、规则表。

阈值全部来自本仓真实语料的实测分布, 不凭感觉定。改规则只改这里, 不动检测逻辑。
"""
from __future__ import annotations

import re
from dataclasses import dataclass

E, W = "E", "W"


@dataclass
class Rule:
    """一条规则: pattern 与 checker 二选一, 后者指向 CHECKERS 里的函数名。"""

    rid: str
    sev: str
    profiles: tuple[str, ...]
    why: str
    pattern: str | None = None
    flags: int = 0
    checker: str | None = None
    limit: int = 0


# 套话词表。校准法: 对 blog/ + docs/ (作者手写) 与 ai-docs/ (AI 生成) 分别计数,
# 只有"人写近乎不出现、AI 常出现"的词进 HARD 表。新增词前先跑一遍对比统计。
#
# 阈值: 人写篇数占比须 < 2% (作者 977 篇人写语料, 即 < 20 篇), 否则说明它是正常词。
#
# 实测否决案例 (记下来免得重复试):
#   其实   人写 124 次/82 篇 > AI 98 次/52 篇   -> 人写更多, 是普通口语词
#   进行   人写 1845 > AI 62                     -> 人写 30 倍, 是正常动词
#   几乎   人写 141/97 篇                        -> 正常副词
#   沉淀   人写 2 次/1 篇 vs AI 74 次/28 篇      -> 通过, 但作者那 1 篇是活用
#                                                  (「想着一定要沉淀下来!」), 故只挂 article,
#                                                  不挂 blog 与 atom
# 参考: 简明技术中文 (simplified-technical-chinese) 的词表每条自带「自动检查」列,
# 把"能自动查"和"只能人看"分开声明, 而不是假装全都能查。
# 校准约定与 profile 区分的依据见
# 
# Agent Notes: 套话词表按人写/AI 的双向频次校准, 并区分「只约束派生文章」的词
# 
# Agent Notes: 套话词表按人写/AI 的双向频次校准, 并区分「只约束派生文章」的词
# .agents/notes/implemented/process/2026-10-06-cliche-list-calibrated-both-ways.md
CLICHE_COUNTS = {
    # 词: (人写次, 人写篇, AI 次, AI 篇)
    "沉淀": (2, 1, 74, 28),
}

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

# 单独挂 article 的词: 通过频次校准, 但作者在随笔里会活用 (词义随语境变),
# 所以只约束派生文章, 不管 blog 与原子稿。
CLICHE_ARTICLE_ONLY = ["沉淀"]

CLICHE_SOFT = ["值得注意的是", "也就是说", "本质上", "某种意义上"]

# 句长上限 (作者手写 blog 的 p90)。见 notes/2026-09-26-sentence-length-gate.md。
SENTENCE_SOFT_LIMIT = 43

# 抽象名词密度上限: 唯一 AI 显著超标的一项 (手写 51 / AI 379, 每千段)。
ABSTRACT_NOUN_SOFT_PER_1K = 200

# 抽象名词词表: 只收"在技术语境里几乎总是空壳"的那类。报出来只当提示, 不当判据。
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
    Rule("cliche-article", E, ("article",),
         "套话 (通过频次校准, 但作者在随笔里会活用该词, 故只约束派生文章)",
         pattern="|".join(CLICHE_ARTICLE_ONLY)),
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

# 段落长度上限 (作者手写段落的 p95)。原子知识稿要求更短。
# Agent Notes: 沉淀流水线改为"两份产物 + 九个单一职责阶段"
# .agents/notes/implemented/process/2026-09-24-sediment-two-artifacts-nine-stages.md
PARA_LIMIT = {"atom": 80, "article": 204, "blog": 260}
