#!/usr/bin/env python3
"""kb: 卡片 + 原件 + 概念索引.

用法:
  python3 scripts/kb/kb.py lint [卡片路径 ...]   不给路径就检查 kb/cards 全部
  python3 scripts/kb/kb.py index                 生成 kb/index.json
  python3 scripts/kb/kb.py find <概念或别名>     按概念查卡 (含子概念), 默认排除 stale
  python3 scripts/kb/kb.py use <卡片 id>         打印动作, 记一次调用到 kb/.usage.jsonl

规则见 .hark/plan.md (v3) §3. 本文件实现: C1-C10, M1, P1, S1 (warn). M2 / U1 待补.
"""
from __future__ import annotations

import datetime as dt
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
KB = REPO / "kb"
CARDS = KB / "cards"
RAW = KB / "raw"
SELF = RAW / "self"
CONCEPTS = KB / "concepts.yaml"
INDEX = KB / "index.json"
USAGE = KB / ".usage.jsonl"

TYPES = {"cmd", "check", "decide", "measure", "pitfall", "question"}
ACTIONS = {"cmd", "check", "question"}
TIERS = {"A", "B", "C"}
TTL_DAYS = {"A": 90, "B": 180}          # 可用 frontmatter ttl_days 覆盖 (行情/费率写 30)
MAX_LINES = 15
MAX_CONCLUSION = 60
MAX_NOVEL = 0.4                         # C10: 结论里的汉字二元组, 摘录里没有的比例上限

BANNED = ["总之", "综上", "总而言之", "值得注意", "值得一提", "未来", "展望", "不难看出",
          "本质上", "赋能", "深刻", "我们可以看到", "众所周知", "毋庸置疑", "显而易见"]
SELF_WORDS = ["我", "作者", "本人", "最近", "之前", "以前", "习惯", "启发", "体会", "感悟"]
SENT_END = re.compile(r"[。！？!?；;]|\.(?=\s|$)")
NUM = re.compile(r"\d+(?:\.\d+)?")
ACTION_ITEM = re.compile(r"^\s*-\s*(\[ \]\s+\S|`[^`]+`|[?？]\s*\S)")
TOKEN_RES = [
    re.compile(r"`([^`]+)`"),                                              # 反引号
    re.compile(r"[A-Za-z_][\w]*(?:\.[A-Za-z_][\w]*)+"),                   # a.b.c
    re.compile(r"\b[A-Za-z]+_[\w]+"),                                     # snake_case
    re.compile(r"\b[a-z]+[A-Z]\w*|\b[A-Z][a-z]+[A-Z]\w*"),                # camelCase / CamelCase
    re.compile(r"\b[A-Za-z]+(?:-[A-Za-z]+)+\b"),                           # 连字符词
    re.compile(r"「([^」]+)」|\"([^\"]+)\"|“([^”]+)”"),                    # 引号内原文
]
SRC_RE = re.compile(r"^(?P<path>[^#]+)(?:#L(?P<a>\d+)(?:-L?(?P<b>\d+))?)?$")


@dataclass
class Card:
    path: Path
    meta: dict
    body: list[str]            # frontmatter 之后的行
    body_start: int            # body 第一行在文件中的行号 (1 起)
    conclusion: str = ""
    conclusion_line: int = 0
    quotes: list[tuple[int, str]] = field(default_factory=list)
    action_lines: list[tuple[int, str]] = field(default_factory=list)
    has_action_header: bool = False


def zi(s: str) -> int:
    """按 "字" 计长: 每个汉字算 1, 每个连续的英文/数字/符号词算 1, 空白不算."""
    return len(re.findall(r"[\u3400-\u9fff]|[A-Za-z0-9_.()\[\]%/+-]+|[^\sA-Za-z0-9_.()\[\]%/+-]", s))


def cjk_bigrams(s: str) -> set[str]:
    out = set()
    for run in re.findall(r"[\u3400-\u9fff]+", s):
        out |= {run[i:i + 2] for i in range(len(run) - 1)}
    return out


def tokens(s: str) -> set[str]:
    out = set()
    for r in TOKEN_RES:
        for m in r.finditer(s):
            g = next((x for x in m.groups() if x), None) if m.groups() else m.group(0)
            if g and len(g.strip()) >= 2:
                out.add(g.strip())
    return out


def norm(s: str) -> str:
    return re.sub(r"\s+", " ", s).strip()


def load_concepts() -> dict:
    data = yaml.safe_load(CONCEPTS.read_text(encoding="utf-8")) or {}
    return {k: (v or {}) for k, v in data.items()}


def parse_card(path: Path) -> Card:
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines or lines[0].strip() != "---":
        raise ValueError("缺 frontmatter")
    end = next((i for i in range(1, len(lines)) if lines[i].strip() == "---"), None)
    if end is None:
        raise ValueError("frontmatter 没有闭合")
    meta = yaml.safe_load("\n".join(lines[1:end])) or {}
    card = Card(path, meta, lines[end + 1:], end + 2)
    in_action = False
    for off, line in enumerate(card.body):
        no = card.body_start + off
        s = line.strip()
        if not s:
            continue
        if s.startswith("动作"):
            in_action, card.has_action_header = True, True
            continue
        if in_action:
            card.action_lines.append((no, line))
        elif s.startswith(">"):
            card.quotes.append((no, s[1:].strip()))
        elif not card.conclusion:
            card.conclusion, card.conclusion_line = s, no
        else:
            # 结论之后、动作之前的其他自由文字也算结论区, 用于 C3 计句
            card.conclusion += " " + s
    return card


def resolve_src(ref: str) -> tuple[Path | None, int, int, str]:
    """返回 (文件, 起行, 止行, 错误). 路径相对 kb/ 或仓库根."""
    m = SRC_RE.match(ref.strip())
    if not m:
        return None, 0, 0, f"src 格式不对: {ref}"
    p = m.group("path")
    cand = [KB / p, REPO / p]
    f = next((c for c in cand if c.is_file()), None)
    if f is None:
        return None, 0, 0, f"src 文件不存在: {p}"
    try:
        f.resolve().relative_to(RAW.resolve())
    except ValueError:
        return None, 0, 0, f"src 必须在 kb/raw/ 下: {p}"
    a = int(m.group("a") or 0)
    b = int(m.group("b") or a)
    return f, a, b, ""


def src_text(f: Path, a: int, b: int) -> str:
    lines = f.read_text(encoding="utf-8", errors="replace").splitlines()
    if a:
        lines = lines[a - 1:b]
    return norm("\n".join(lines))


def lint_card(card: Card, concepts: dict, today: dt.date) -> list[tuple[str, int, str, str]]:
    """返回 [(级别, 行号, 规则, 说明)]."""
    out = []
    m = card.meta

    def err(rule, msg, line=1):
        out.append(("error", line, rule, msg))

    def warn(rule, msg, line=1):
        out.append(("warn", line, rule, msg))

    rel = card.path.resolve().relative_to(CARDS.resolve()).with_suffix("").as_posix()
    if m.get("id") != rel:
        err("ID", f"id 应为 {rel}, 实为 {m.get('id')}")

    ctype = m.get("type")
    if ctype not in TYPES:                                            # C6
        err("C6", f"type 不在白名单 {sorted(TYPES)}: {ctype}")
    if m.get("tier") not in TIERS:
        err("TIER", f"tier 必须是 A/B/C: {m.get('tier')}")

    # C2 可索引
    cs = m.get("concepts") or []
    if not cs:
        err("C2", "concepts 为空")
    for c in cs:
        if c not in concepts:
            err("C2", f"概念未登记在 concepts.yaml: {c}")

    # C1 必须有动作
    action = m.get("action")
    if action not in ACTIONS:
        err("C1", f"action 必须是 {sorted(ACTIONS)}: {action}")
    items = [(n, l) for n, l in card.action_lines if ACTION_ITEM.match(l)]
    if not card.has_action_header or not items:
        err("C1", "正文缺 `动作:` 段或其下没有 `- [ ] 项` / `- `命令`` / `- ? 问题`")

    # C3 一卡一结论
    if not card.conclusion:
        err("C3", "没有结论句")
    else:
        n_sent = len(SENT_END.findall(card.conclusion.rstrip("。.!！?？")))
        if n_sent > 0:
            err("C3", f"结论只许一句, 实有 {n_sent + 1} 句", card.conclusion_line)
        if zi(card.conclusion) > MAX_CONCLUSION:
            err("C3", f"结论 {zi(card.conclusion)} 字, 上限 {MAX_CONCLUSION} (汉字和英文词各算 1)", card.conclusion_line)
    n_lines = sum(1 for l in card.body if l.strip())
    if n_lines > MAX_LINES:
        err("C3", f"正文 {n_lines} 行, 上限 {MAX_LINES}")

    # src
    srcs = m.get("src") or []
    if isinstance(srcs, str):
        srcs = [srcs]
    resolved = []
    for s in srcs:
        f, a, b, e = resolve_src(s)
        if e:
            err("SRC", e)
        else:
            resolved.append((s, f, a, b))
    needs_src = ctype != "question"
    if needs_src and not srcs:
        err("SRC", "非 question 卡必须有 src")

    # C4 逐字摘录
    if needs_src and not card.quotes:
        err("C4", "缺 `>` 原文摘录")
    texts = [src_text(f, a, b) for _, f, a, b in resolved]
    for no, q in card.quotes:
        if resolved and not any(norm(q) in t for t in texts):
            err("C4", f"摘录不是所引原件的逐字子串: {q[:40]}", no)

    # C5 数字命中原文
    if card.conclusion and resolved:
        for num in NUM.findall(card.conclusion):
            if not any(re.search(rf"(?<![\d.]){re.escape(num)}(?![\d])", t) for t in texts):
                err("C5", f"结论里的数字 {num} 在所引原件中找不到", card.conclusion_line)

    # C9 专名与引文必须出自原件: 反引号内容, 点号路径, snake/camel 标识符, 连字符词, 引号内原文
    if card.conclusion and resolved:
        for tok in sorted(tokens(card.conclusion)):
            if not any(norm(tok) in t for t in texts):
                err("C9", f"结论里的 {tok} 在所引原件中找不到", card.conclusion_line)

    # C10 不许改写: 结论的汉字二元组, 摘录里没有的比例 ≤ MAX_NOVEL
    if card.conclusion and card.quotes:
        cb = cjk_bigrams(card.conclusion)
        if len(cb) >= 4:
            qb = cjk_bigrams(" ".join(q for _, q in card.quotes))
            novel = len(cb - qb) / len(cb)
            if novel > MAX_NOVEL:
                err("C10", f"结论 {novel:.0%} 的字词摘录里没有 (上限 {MAX_NOVEL:.0%}), 照摘录写, 不要改写", card.conclusion_line)

    # C7 禁用句式 (结论 + 动作)
    zone = [(card.conclusion_line, card.conclusion)] + card.action_lines
    for no, text in zone:
        for w in BANNED:
            if w in text:
                err("C7", f"禁用词: {w}", no)

    # C8 命令真跑
    if ctype == "cmd" and not m.get("tested") and action == "cmd":
        err("C8", "cmd 卡没有 tested: <日期> <环境>, action 只能是 check")

    # M1 关于 "我" 只能取不能生
    for no, text in zone:
        hit = [w for w in SELF_WORDS if w in text]
        if hit:
            if not resolved or any(not f.resolve().is_relative_to(SELF.resolve()) for _, f, _, _ in resolved):
                err("M1", f"含自指词 {hit}, src 必须全部在 kb/raw/self/", no)

    # P1 方案先定位
    if m.get("request") == "plan" and m.get("tier") == "A" and ctype != "question":
        pos = m.get("positioning")
        f, _, _, e = resolve_src(pos) if pos else (None, 0, 0, "缺 positioning")
        if e or not f.resolve().is_relative_to(SELF.resolve()):
            err("P1", f"A 档方案卡必须有指向 kb/raw/self/ 的 positioning ({e or '不在 raw/self'}); 否则只能产出 question 卡")

    # S1 过期
    tier = m.get("tier")
    ttl = m.get("ttl_days") or TTL_DAYS.get(tier)
    as_of = m.get("as_of")
    if tier in TTL_DAYS and not as_of:
        err("S1", "A/B 档必须有 as_of")
    elif ttl and as_of:
        d = as_of if isinstance(as_of, dt.date) else dt.date.fromisoformat(str(as_of))
        if (today - d).days > int(ttl):
            warn("S1", f"stale: as_of {d}, 超过 {ttl} 天")
    return out


def all_cards() -> list[Path]:
    return sorted(CARDS.rglob("*.md"))


def cmd_lint(args: list[str]) -> int:
    concepts = load_concepts()
    today = dt.date.today()
    paths = [Path(a).resolve() for a in args] or all_cards()
    n_err = n_warn = 0
    for p in paths:
        shown = p.relative_to(REPO) if p.is_relative_to(REPO) else p
        try:
            card = parse_card(p)
        except Exception as e:  # noqa: BLE001
            print(f"{shown}:1 error PARSE {e}")
            n_err += 1
            continue
        for lvl, no, rule, msg in lint_card(card, concepts, today):
            print(f"{shown}:{no} {lvl} {rule} {msg}")
            n_err += lvl == "error"
            n_warn += lvl == "warn"
    print(f"-- {len(paths)} 张卡, {n_err} error, {n_warn} warn")
    return 1 if n_err else 0


def expand(concepts: dict, key: str) -> set[str]:
    out, todo = {key}, [key]
    while todo:
        k = todo.pop()
        for c, v in concepts.items():
            if v.get("parent") == k and c not in out:
                out.add(c)
                todo.append(c)
    return out


def build_index() -> dict:
    concepts = load_concepts()
    today = dt.date.today()
    cards, by_concept = {}, {}
    for p in all_cards():
        try:
            c = parse_card(p)
        except Exception:  # noqa: BLE001
            continue
        stale = any(r == "S1" for _, _, r, _ in lint_card(c, concepts, today))
        cid = c.meta.get("id")
        cards[cid] = {
            "conclusion": c.conclusion, "type": c.meta.get("type"), "tier": c.meta.get("tier"),
            "concepts": c.meta.get("concepts") or [], "stale": stale,
            "path": p.relative_to(REPO).as_posix(),
        }
        for k in c.meta.get("concepts") or []:
            by_concept.setdefault(k, []).append(cid)
    alias = {}
    for k, v in concepts.items():
        alias[k.lower()] = k
        for a in v.get("aliases") or []:
            alias[str(a).lower()] = k
    return {"generated": today.isoformat(), "alias": alias, "concepts": by_concept, "cards": cards}


def cmd_index(_: list[str]) -> int:
    idx = build_index()
    INDEX.write_text(json.dumps(idx, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print(f"index: {len(idx['cards'])} 张卡, {len(idx['concepts'])} 个概念 -> {INDEX.relative_to(REPO)}")
    return 0


def cmd_find(args: list[str]) -> int:
    if not args:
        print("用法: kb find <概念或别名> [--all]")
        return 2
    show_stale = "--all" in args
    q = " ".join(a for a in args if a != "--all")
    idx = build_index()
    concepts = load_concepts()
    key = idx["alias"].get(q.lower())
    ids: list[str] = []
    if key:
        for k in sorted(expand(concepts, key)):
            ids += [i for i in idx["concepts"].get(k, []) if i not in ids]
    else:
        ids = [i for i, c in idx["cards"].items() if q.lower() in c["conclusion"].lower()]
        print(f"(没有概念或别名叫 {q}, 按结论全文匹配)")
    n = 0
    for i in ids:
        c = idx["cards"][i]
        if c["stale"] and not show_stale:
            continue
        flag = " [stale]" if c["stale"] else ""
        print(f"{i}  [{c['type']}/{c['tier']}]{flag}  {c['conclusion']}")
        n += 1
    if not n:
        print("没有卡. 去原件或网上查, 查到再抽卡.")
    return 0 if n else 1


def cmd_use(args: list[str]) -> int:
    if not args:
        print("用法: kb use <卡片 id>")
        return 2
    cid = args[0].removesuffix(".md")
    p = CARDS / f"{cid}.md"
    if not p.is_file():
        print(f"没有这张卡: {cid}")
        return 1
    c = parse_card(p)
    print(c.conclusion)
    for _, l in c.action_lines:
        print(l)
    with USAGE.open("a", encoding="utf-8") as f:
        f.write(json.dumps({"id": cid, "at": dt.datetime.now().isoformat(timespec="seconds")}, ensure_ascii=False) + "\n")
    return 0


def main(argv: list[str]) -> int:
    """kb 命令入口: lint / index / find / use.

    决策: .agents/notes/implemented/architecture/2026-11-09-kb-cards-raw-concept-index.md
    """
    cmds = {"lint": cmd_lint, "index": cmd_index, "find": cmd_find, "use": cmd_use}
    if not argv or argv[0] not in cmds:
        print(__doc__)
        return 2
    return cmds[argv[0]](argv[1:])


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
