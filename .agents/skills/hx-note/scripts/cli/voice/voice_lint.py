"""lint 与 fix 两个子命令的实现: 逐文件检查、折行合并。"""
from __future__ import annotations

import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

from lib.textpaths import expand_markdown
from voice.voice_checks import CHECKERS, LIST_ITEM_RE, line_of, mask
from voice.voice_corpus import find_voice_db, load_voice_db
from voice.voice_rules import E, PARA_LIMIT, RULES, W


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
    """按路径猜 profile。递归收集来的路径可能是相对的, 所以按**路径段**判断。

    用 `"blog" in path.parts` 而不是老的 `"/blog/" in as_posix()`: 后者的前导斜杠
    要求路径是绝对路径, 于是 `blog/2026/05/07/01_x.md` 会被猜成 article 
    实测这个错配会把最松的一套 (blog) 换成另一套规则, 报出一堆不是问题的东西。
    """
    name = path.name
    if name == ".hx-info.md":
        return "atom"
    if "blog" in path.parts:
        return "blog"
    return "article"

def cmd_lint(args) -> int:
    db = load_voice_db(Path(args.db) if args.db else find_voice_db())
    files, missing = expand_markdown(args.paths)
    for raw in missing:
        print(f"error: 文件不存在: {raw}", file=sys.stderr)
        return 2
    if not files:
        print("error: 没有找到任何 markdown 文件", file=sys.stderr)
        return 2

    reports: list[FileReport] = []
    for p in files:
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
        # 那条说明只需要看一次  重复 20 遍是纯占上下文。
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
    时才补是不够的  那会产出 `线程,自己带一段` 这种与全篇不一致的写法。

    **必须跳过 frontmatter。** 它是一行一个 key 的 YAML, 按段落合并会把
    `hxid: "..."` 与 `title: "..."` 压成一行, 产出非法 YAML  实测踩过这个坑。
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
    files, missing = expand_markdown(args.paths)
    for raw in missing:
        print(f"跳过 (找不到): {raw}", file=sys.stderr)
    total = 0
    touched = 0
    for p in files:
        raw = p.read_text(encoding="utf-8")
        new, n = unwrap_text(raw)
        if not n:
            continue
        total += n
        touched += 1
        if args.write:
            p.write_text(new, encoding="utf-8")
            print(f"  {p}: 合并 {n} 处 (已写入)")
        else:
            print(f"  {p}: 合并 {n} 处 (dry-run)")
    if not total:
        print(f"没有发现被折行的段落 (扫过 {len(files)} 个文件)")
    else:
        head = f"合计 {total} 处, 涉及 {touched} 个文件"
        print("\n" + (head + "; 已落盘" if args.write else head + "; 加 --write 落盘"))
    return 0
