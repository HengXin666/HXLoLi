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


# 套话词表: 只收"人写近乎不出现, AI 常出现"的词, 词旁记 (人写次, 人写篇, AI 次, AI 篇)
# 合格阈值是人写篇数占比低于 2%, 凭印象加词会误伤作者手写语料, 故新增词前先跑双向频次对比
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
         "不用破折号. 想补充就用半角括号, 想停顿就用 ... 或另起一句 (作者 33 篇 blog 里只用过 2 次)",
         pattern=r"—"),
    Rule("not-but", E, ("atom", "article", "blog"),
         '不写「不是 X 而是 Y」. 直接说 Y 是什么; 只有读者真会以为是 X 时, 才单独写一句纠正',
         pattern=r"不(?:是|只是|仅是|仅仅是|光是)[^。.!?\n]{0,30}?而(?:是|在于)"),
    Rule("not-but2", E, ("atom", "article", "blog"),
         '不写「不仅...更是」. 两件事都重要就分两句说',
         pattern=r"不仅(?:仅)?[^。.!?\n]{0,30}?(?:更是|更在于|还是)"),
    Rule("cliche", E, ("atom", "article", "blog"),
         "不写套话 (值得一提的是 / 综上所述 / 至关重要 等). 判断办法: 删掉后句子意思不变, 就删",
         pattern="|".join(CLICHE_HARD)),
    Rule("cliche-article", E, ("article",),
         "文章里不用「沉淀」. 说清具体是记下了什么、存到了哪 (随笔里可以用, 这条只管 ai-docs)",
         pattern="|".join(CLICHE_ARTICLE_ONLY)),
    Rule("cliche-soft", W, ("atom", "article"),
         "「值得注意的是 / 也就是说 / 本质上」先删掉读一遍, 意思没少就别加回来",
         pattern="|".join(CLICHE_SOFT)),
    Rule("assistant-leak", E, ("atom", "article", "blog"),
         "不写「希望对你有帮助 / 有问题随时问我」这类对话口吻, 文章写完就停",
         pattern=r"希望(?:这|以上|本文)|如果(?:你|您)(?:还|有)(?:其他|任何)|随时(?:告诉我|问我)|我(?:已经|将)为(?:你|您)"),

    # ---- 排版指纹 ----
    # 数词故意从「三」起: 「一块假栈」「两个线程」里的「一/两」是普通量词,
    # 把它们算进来会把正常中文全判成凑数排比 (实测误报)。真正的凑数排比都是三及以上。
    Rule("hype-heading", W, ("article", "blog"),
         "小标题不写「三大 / 四个 / 五块」. 用这一节讲的那件事命名",
         pattern=r"^#{2,6}[^\n]*?[三四五六七]\s*(?:大|个|层|条|块|点|方面|维度|理由|信号)[^\n]*$",
         flags=re.M),
    Rule("bold-bullet", W, ("article", "blog"),
         "别每个列表项都用 `- **粗体**:` 开头. 只给真要点名的项加粗, 其余直接写",
         checker="bold_bullet", limit=50),
    Rule("bold-density", W, ("atom", "article", "blog"),
         "加粗只留给读者扫一眼就该看到的那几个词. 一段里加粗多了等于没加",
         checker="bold_density", limit=25),
    Rule("emoji", W, ("article",),
         "技术文里不放装饰 emoji, 最多偶尔 1 个 (随笔不管)",
         checker="emoji", limit=3),
    # 2026-11-08 用户硬红线: ai-docs 笔记禁用 md 表格, 无例外, --allow-table 对 article 不再生效。
    Rule("table", E, ("article",),
         "不用 markdown 表格. 对照写成短句或分点, 结构关系画成图",
         checker="table", limit=-1),
    # 下面三条会逼着每段塞图/表/代码, 是装饰图和编造示例的来源; 对照实验里预算版 6/6 输给无预算版。
    # 2026-11-08 停用 (profiles 置空), 保留定义以便回看。
    Rule("section-visual", E, (),
         "已停用: 不再要求每段都有视觉",
         checker="section_visual"),
    Rule("text-ratio", W, (),
         "已停用",
         checker="text_ratio", limit=4),
    Rule("prose-budget", W, (),
         "已停用: 对照实验里 30 行预算伤收获",
         checker="prose_budget", limit=30),
    # 2026-11-08 用户硬红线: ai-docs 禁止大段文本, article 升为 E。
    Rule("long-para-hard", E, ("article",),
         "一段只说一件事, 超过上限就拆. 拆的时候不靠补图补代码",
         checker="long_para"),
    Rule("long-para", W, ("atom", "blog"),
         "段落写短. 作者的 blog 几乎一句一段, 超过上限就拆",
         checker="long_para"),
    Rule("abstract-density", W, ("article",),
         "少用「机制 / 链路 / 闭环 / 维度」这类抽象名词, 换成具体的那件事 (作者手写每千段 51 个, AI 稿 379 个)",
         checker="abstract_density", limit=ABSTRACT_NOUN_SOFT_PER_1K),
    Rule("long-sentence", W, ("article",),
         "超过 43 字的句子拆开. 作者手写的句子中位 17 字",
         checker="long_sentence", limit=SENTENCE_SOFT_LIMIT),

    Rule("hard-wrap", E, ("atom", "article", "blog"),
         "一段话写成一行, 别手工折行: Markdown 会把软换行渲染成空格, 中文里会多出空格 (列表和代码块不受此限)",
         checker="hard_wrap"),

    # ---- 过程泄漏 ----
    Rule("meta-leak", E, ("atom", "article"),
         "正文里不出现「本文将介绍」「AI 生成」、TODO、本地路径、uv run 这类生成过程的痕迹",
         pattern=r"本文(?:将|会|首先)?(?:介绍|讲解|探讨|带你)|AI\s*(?:辅助|生成)|\bTODO\b|/Users/|\.agents/|uv run|makeDoc\.py"),

    # ---- atom 专属: 知识库纯洁度 ----
    Rule("atom-image", E, ("atom",),
         ".hx-info.md 里不放图, 图放 index.md (这份是给检索的纯文本)",
         pattern=r"!\["),
    Rule("atom-person", E, ("atom",),
         "知识点里不写 我 / 我们 / 你, 直接陈述事实",
         pattern=r"(?<![A-Za-z])(?:我们|我的|我|咱|你可以|你需要|大家)(?![A-Za-z])"),
    Rule("atom-rhetorical", E, ("atom",),
         "知识点里不用问句, 直接写答案",
         pattern=r"[?？]\s*$", flags=re.M),
    Rule("atom-transition", E, ("atom",),
         "知识点不以 首先 / 因此 / 另外 开头. 每条单独拿出来要能读懂",
         pattern=r"^\s*(?:首先|其次|再次|然后|接下来|最后|另外|此外|而且|因此|所以|不过|但是|同时)[,，、]",
         flags=re.M),
    Rule("atom-hedge", E, ("atom",),
         "确定就直接写, 不确定就标 [不确定], 别用 可能 / 也许 / 大概 含糊过去",
         pattern=r"(?:可能|也许|或许|大概|似乎|应该是|差不多|大致上)(?!\])"),

    # ---- article 专属: 共鸣段的硬约束 ----
    Rule("second-person-hook", E, ("article",),
         "用第一人称讲自己的事, 不跟读者寒暄 (不写 你最近 / 相信你 / 如果你也)",
         pattern=r"(?:相信|如果|也许)(?:你|您)(?:也|还|一定|可能)|(?:你|您)(?:最近|可能已经|一定)"),
]

# 段落长度上限 (作者手写段落的 p95)。原子知识稿要求更短。
# Agent Notes: 沉淀流水线改为"两份产物 + 九个单一职责阶段"
# .agents/notes/implemented/process/2026-09-24-sediment-two-artifacts-nine-stages.md
# article 2026-11-08 由 204 降为 150: 作者 blog 正文段 (去代码/列表/引用) 实测 p90=100, p95=147。
PARA_LIMIT = {"atom": 80, "article": 150, "blog": 260}


# ---- 写前规则清单 (单一来源) ----
# 2026-11-08: 写作规则必须在动笔前就端给执行者, 闸门只核对同一份清单。
# 见 .agents/notes/implemented/architecture/2026-11-08-rules-feed-forward-gates-confirm.md
# lint 管不到、由 doctor 或人工判的内容规则也登记在这里, 这样 status 打出来的就是全集。
CONTENT_RULES: dict[str, list[str]] = {
    "article": [
        "只放真跑过或能指出出处的代码, 每块的出处写进 .hx-mitemite.md「## 代码出处」. 没有真代码就不放, 不为示意编一段",
        "图只画结构: 架构图 / 流程图 / 真实截图. 封面、概念示意、类比图不画. 架构图用生图模型画, 画完逐个核对节点和箭头",
        "只写 .hx-info.md 里有的论断. 要加新事实, 先回写 .hx-info.md 并附依据",
    ],
    "atom": [],
}


def writing_rules(profile: str) -> list[str]:
    """动笔前要看到的全部硬规则: lint 的 E 级规则 + CONTENT_RULES。"""
    out = []
    for r in RULES:
        if r.sev == E and profile in r.profiles:
            why = r.why
            if r.checker == "long_para":
                why = f"{why} (上限 {PARA_LIMIT[profile]} 字)"
            out.append(why)
    return out + CONTENT_RULES.get(profile, [])
