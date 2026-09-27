# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""从仓库里的既有痕迹构建 [用户画像], 缓存到 ai-docs/.hx-persona.md。

为什么要缓存: 画像每篇重算一次纯属浪费, 而且结果不稳定会让"引入/展望"段的
口吻在不同笔记之间漂移。默认 TTL 14 天。

为什么 provider 要可插拔: 数据源的可用性是环境相关的 —— 这个仓库当前甚至
不是一个 git 仓库 (`git log` 直接失败)。所以每个 provider 必须能独立降级,
缺一个就在"数据缺口"里写明缺了什么, 而不是让整次构建失败。

新增数据源 = 写一个 `collect_xxx(ctx) -> Section` 并登记进 PROVIDERS, 不改其它任何东西。
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from dataclasses import dataclass, field
from datetime import date, datetime, timedelta
from pathlib import Path

DEFAULT_OUT = "ai-docs/.hx-persona.md"
EXTRA_FILE = "ai-docs/.hx-persona.extra.md"
DEFAULT_TTL_DAYS = 14
DEFAULT_MONTHS = 3
DEFAULT_MIN_ITEMS = 6

# 作者的口吻标记。命中越多越可能是一句"只有他会这么写"的原句。
VOICE_MARKERS = [
    "qwq", "awa", "=-=", "\\o/", "喵", "(雾)", "~~", "八嘎", "白嫖", "吃灰",
    "拉胯", "卢瑟", "老登", "哥们", "调教", "镇楼", "艹", "完事了", "就完事",
]


@dataclass
class Section:
    name: str
    ok: bool
    lines: list[str] = field(default_factory=list)
    note: str = ""


@dataclass
class Ctx:
    root: Path
    since: date
    months: int


# --------------------------------------------------------------------------
def repo_root(start: Path | None = None) -> Path:
    cur = (start or Path.cwd()).resolve()
    while True:
        if (cur / "ai-docs").is_dir() and (cur / "blog").is_dir():
            return cur
        if cur.parent == cur:
            return (start or Path.cwd()).resolve()
        cur = cur.parent


def frontmatter(text: str) -> tuple[dict[str, str], str]:
    m = re.match(r"\A---[ \t]*\r?\n(.*?)\r?\n---[ \t]*\r?\n?(.*)\Z", text, re.S)
    if not m:
        return {}, text
    fm: dict[str, str] = {}
    key = None
    for line in m.group(1).splitlines():
        if re.match(r"^\s*-\s+", line) and key:
            fm[key] = (fm.get(key, "") + "," + line.split("-", 1)[1].strip()).strip(",")
            continue
        kv = re.match(r"^([A-Za-z_][\w-]*)\s*:\s*(.*)$", line)
        if kv:
            key = kv.group(1)
            fm[key] = kv.group(2).strip().strip("\"'")
    return fm, m.group(2)


def split_tags(raw: str) -> list[str]:
    """frontmatter 的 tags 有两种写法: 行内 `["a", "b"]` 与多行 `- a`。都要认。"""
    raw = (raw or "").strip().strip("[]")
    return [t.strip().strip("\"'[] ") for t in raw.split(",") if t.strip().strip("\"'[] ")]


def parse_day(raw: str) -> date | None:
    raw = (raw or "").strip().strip("\"'")
    m = re.match(r"(\d{4})[-/](\d{1,2})[-/](\d{1,2})", raw)
    if not m:
        return None
    try:
        return date(int(m.group(1)), int(m.group(2)), int(m.group(3)))
    except ValueError:
        return None


def months_ago(d: date, months: int) -> date:
    y, m = d.year, d.month - months
    while m <= 0:
        m += 12
        y -= 1
    return date(y, m, min(d.day, 28))


def voice_score(line: str) -> int:
    s = sum(3 for mk in VOICE_MARKERS if mk in line)
    if re.search(r"(?<![A-Za-z])我(?![A-Za-z])", line):
        s += 2
    if 12 <= len(line) <= 70:
        s += 2
    if line.count("(") and line.count(")"):
        s += 1
    if line.rstrip().endswith("~"):
        s += 1
    return s


# --- providers -------------------------------------------------------------
def collect_blog(ctx: Ctx) -> Section:
    """blog/ 是最可靠的数据源: 有 frontmatter 日期, 而且是作者亲手写的。"""
    files = sorted((ctx.root / "blog").rglob("*.md"))
    if not files:
        return Section("blog", False, note="blog/ 下没有 md")
    items = []
    for f in files:
        fm, body = frontmatter(f.read_text(encoding="utf-8", errors="replace"))
        d = parse_day(fm.get("date", "")) or parse_day("-".join(f.parts[-4:-1]))
        if d is None:
            continue
        items.append((d, fm.get("title") or f.stem, fm.get("tags", ""), body, f))
    items.sort(key=lambda x: x[0], reverse=True)
    picked = [it for it in items if it[0] >= ctx.since]
    widened = False
    if len(picked) < DEFAULT_MIN_ITEMS:
        picked = items[:DEFAULT_MIN_ITEMS]
        widened = True
    lines = [f"- `{it[0]}` {it[1]}" + (f"  tags: {it[2]}" if it[2] else "") for it in picked]
    note = ""
    if widened and picked:
        note = (f"近 {ctx.months} 个月只有 {sum(1 for i in items if i[0] >= ctx.since)} 篇, "
                f"已自动放宽到最近 {len(picked)} 篇 (最早 {picked[-1][0]})")
    return Section("blog", True, lines, note)


REPO_RE = re.compile(r"github\.com/([\w.-]+)/([\w.-]+)")
# 代码示例里的占位 URL (github.com/user/repo.git) 不是项目, 要滤掉
PLACEHOLDER_OWNERS = {"user", "users", "org", "owner", "example", "yourname", "USERNAME"}


def collect_projects(ctx: Ctx) -> Section:
    """从 blog 正文里的 GitHub 链接反推"最近在做什么项目"。"""
    hits: dict[str, tuple[date, str]] = {}
    for f in sorted((ctx.root / "blog").rglob("*.md")):
        fm, body = frontmatter(f.read_text(encoding="utf-8", errors="replace"))
        d = parse_day(fm.get("date", "")) or parse_day("-".join(f.parts[-4:-1]))
        if d is None:
            continue
        for line in body.splitlines():
            for m in REPO_RE.finditer(line):
                if m.group(1).lower() in PLACEHOLDER_OWNERS:
                    continue
                repo = m.group(2).rstrip(")").rstrip("/")
                if repo.endswith(".git"):
                    continue
                desc = re.sub(r"^#+\s*", "", line).strip()
                desc = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", desc)[:90]
                prev = hits.get(repo)
                if prev is None or d > prev[0]:
                    hits[repo] = (d, desc)
    if not hits:
        return Section("projects", False, note="blog 正文里没有 GitHub 链接")
    ordered = sorted(hits.items(), key=lambda kv: kv[1][0], reverse=True)
    lines = [f"- **{repo}** (`{d}`) {desc}" for repo, (d, desc) in ordered[:12]]
    return Section("projects", True, lines)


def collect_ai_docs(ctx: Ctx) -> Section:
    """ai-docs 的 tag 频次 = "最近在调研什么方向"。"""
    root = ctx.root / "ai-docs"
    if not root.is_dir():
        return Section("ai-docs", False, note="没有 ai-docs/")
    freq: dict[str, int] = {}
    recent: list[tuple[date, str, str]] = []
    for f in sorted(root.rglob("index.md")):
        if any(p.startswith(".") for p in f.relative_to(root).parts):
            continue
        fm, _ = frontmatter(f.read_text(encoding="utf-8", errors="replace"))
        tags = split_tags(fm.get("tags", ""))
        d = parse_day(fm.get("created_at", ""))
        for t in tags:
            freq[t] = freq.get(t, 0) + 1
        if d:
            recent.append((d, fm.get("title", f.parent.name), ", ".join(tags)))
    if not freq and not recent:
        return Section("ai-docs", False, note="ai-docs 下没有可读的 index.md")
    recent.sort(reverse=True)
    top = sorted(freq.items(), key=lambda kv: -kv[1])[:12]
    lines = ["- 高频 tag: " + ", ".join(f"{k}×{v}" for k, v in top)]
    lines += [f"- `{d}` {title}" + (f"  [{tags}]" if tags else "")
              for d, title, tags in recent[:8]]
    return Section("ai-docs", True, lines)


def collect_voice(ctx: Ctx) -> Section:
    """摘作者的原句。画像只写"在做什么项目"是不够的 —— 派生文章要模仿的是节奏。"""
    cands: list[tuple[int, str, str]] = []
    for f in sorted((ctx.root / "blog").rglob("*.md")):
        _, body = frontmatter(f.read_text(encoding="utf-8", errors="replace"))
        in_code = False
        for line in body.splitlines():
            if line.lstrip().startswith("```"):
                in_code = not in_code
                continue
            s = line.strip()
            if in_code or not s or s.startswith(("#", "|", "!", "<!--")):
                continue
            s = s.lstrip("> ").lstrip("-*+ ").strip()
            if len(s) < 10:
                continue
            sc = voice_score(s)
            if sc >= 5:
                cands.append((sc, s[:110], str(f.relative_to(ctx.root))))
    if not cands:
        return Section("voice", False, note="没有抽到带口吻标记的句子")
    cands.sort(key=lambda x: -x[0])
    seen: set[str] = set()
    lines = []
    for _, s, src in cands:
        if src in seen and len(lines) > 4:
            continue
        seen.add(src)
        lines.append(f"- {s}\n  <- `{src}`")
        if len(lines) >= 10:
            break
    return Section("voice", True, lines)


def collect_git(ctx: Ctx) -> Section:
    try:
        out = subprocess.run(
            ["git", "log", f"--since={ctx.since.isoformat()}", "--pretty=%ad %s",
             "--date=short", "-n", "40"],
            cwd=ctx.root, capture_output=True, text=True, timeout=20,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return Section("git", False, note=f"git 不可用: {exc}")
    if out.returncode != 0:
        first = (out.stderr or "").strip().splitlines()
        return Section("git", False, note=f"git log 失败: {first[0] if first else '未知原因'}")
    lines = [f"- {ln}" for ln in out.stdout.strip().splitlines() if ln.strip()]
    if not lines:
        return Section("git", False, note=f"{ctx.since} 之后没有提交")
    return Section("git", True, lines[:20])


def collect_github(ctx: Ctx) -> Section:
    try:
        probe = subprocess.run(["gh", "auth", "status"], capture_output=True,
                               text=True, timeout=20)
    except (OSError, subprocess.SubprocessError) as exc:
        return Section("github", False, note=f"gh CLI 不可用: {exc}; 需要时由人类补充")
    if probe.returncode != 0:
        return Section("github", False, note="gh 未登录; 需要时由人类补充")
    try:
        out = subprocess.run(
            ["gh", "api", "/user/repos?sort=pushed&per_page=15",
             "--jq", '.[] | "\\(.pushed_at[0:10]) \\(.name) \\(.description // "")"'],
            capture_output=True, text=True, timeout=40,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return Section("github", False, note=f"gh api 调用失败: {exc}")
    if out.returncode != 0:
        return Section("github", False, note="gh api 调用失败")
    lines = [f"- {ln}" for ln in out.stdout.strip().splitlines() if ln.strip()]
    return Section("github", bool(lines), lines, "" if lines else "gh 返回空")


PROVIDERS = [
    ("最近写了什么 (blog)", collect_blog),
    ("最近在做什么项目", collect_projects),
    ("最近在调研什么方向 (ai-docs)", collect_ai_docs),
    ("口吻样本 (照抄这些句子的节奏, 不要照抄内容)", collect_voice),
    ("最近改了什么 (git)", collect_git),
    ("GitHub 活跃仓库", collect_github),
]


# --------------------------------------------------------------------------
HEADER_RE = re.compile(r"built_at=(\d{4}-\d{2}-\d{2})")


def is_stale(path: Path, ttl_days: int) -> tuple[bool, str]:
    if not path.is_file():
        return True, "缓存不存在"
    m = HEADER_RE.search(path.read_text(encoding="utf-8", errors="replace")[:400])
    if not m:
        return True, "缓存头缺少 built_at"
    built = parse_day(m.group(1))
    if built is None:
        return True, "built_at 不可解析"
    age = (date.today() - built).days
    if age > ttl_days:
        return True, f"缓存已 {age} 天 (TTL {ttl_days})"
    return False, f"缓存 {age} 天内有效 (built_at={built})"


def build(ctx: Ctx, ttl: int) -> str:
    secs = [(title, fn(ctx)) for title, fn in PROVIDERS]
    gaps = [f"- **{s.name}**: {s.note or '无数据'}" for _, s in secs if not s.ok]

    out = [
        f"<!-- generated by hx_persona.py  built_at={date.today().isoformat()}  "
        f"ttl={ttl}d  window={ctx.months}m -->",
        "<!-- 不要手改本文件, 下次 build 会覆盖. 手写补充请写 "
        f"{EXTRA_FILE}, 它会被原样并入且优先于自动结论. -->",
        "",
        "# 用户画像",
        "",
        f"窗口: `{ctx.since}` ~ `{date.today()}` (近 {ctx.months} 个月)",
        "",
    ]
    idx = 0
    for title, s in secs:
        if not s.ok:
            continue
        out += [f"## 0x{idx:02X} {title}", ""]
        if s.note:
            out += [f"> {s.note}", ""]
        out += s.lines + [""]
        idx += 1

    out += [f"## 0x{idx:02X} 数据缺口 (这些方向的判断不要硬编)", ""]
    out += (gaps or ["- 无"]) + [""]
    idx += 1

    extra = ctx.root / EXTRA_FILE
    if extra.is_file():
        out += [f"## 0x{idx:02X} 人类补充 (优先于以上全部自动结论)", "",
                extra.read_text(encoding="utf-8").strip(), ""]

    out += [
        "---",
        "",
        "用法约束:",
        "",
        "- 画像用来**选切入点**, 不是用来夸读者. 禁止写成 `你最近在做 X, 而这篇讲 Y`.",
        "- 作者的文章是第一人称自述. 引入段要写成 `我为什么会碰到这个问题`, 不是对读者寒暄.",
        "- 数据缺口里的方向一律不许臆测; 需要时问人类.",
        "",
    ]
    return "\n".join(out)


def cmd_build(args) -> int:
    root = repo_root(Path(args.root) if args.root else None)
    ctx = Ctx(root=root, since=months_ago(date.today(), args.months), months=args.months)
    out_path = root / (args.out or DEFAULT_OUT)
    stale, why = is_stale(out_path, args.ttl)
    if not stale and not args.refresh:
        print(f"skip: {why}; 加 --refresh 强制重建", file=sys.stderr)
        print(out_path)
        return 0
    text = build(ctx, args.ttl)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # --- 写入前的守卫 ---
    # (see .agents/notes/implemented/architecture/2026-09-26-persona-moved-to-private-repo.md — 画像为什么不许落在公开仓)
    #
    # 画像含"最近在做什么项目"与仓库商业描述, **不能进公开仓**。它真正的家在
    # HXLoLi-imouto (私有仓), 通过 scripts/setup-private.mjs 以符号链接映射到
    # ai-docs/.hx-persona.md。
    #
    # 但如果有人没跑过 setup-private, 那个路径上什么都没有 —— 直接写就会在**公开仓**
    # 里新建一个真文件, 并可能被提交。所以这里拦一道:
    #   · 目标是符号链接 -> 正常写 (内容落到私有仓);
    #   · 目标不存在   -> 拒绝, 并告诉该怎么办。
    if not out_path.is_symlink() and not out_path.is_file():
        print(
            "error: 画像应写入私有仓, 但映射还没建立。\n"
            "  预期路径是符号链接: ai-docs/.hx-persona.md -> ../HXLoLi-imouto/ai-docs/.hx-persona.md\n"
            "  先跑: node scripts/setup-private.mjs\n"
            "  确实要写到别处 (如临时文件), 用 --out 显式指定。",
            file=sys.stderr,
        )
        return 2
    if not out_path.is_symlink() and args.out is None:
        print(
            "error: 目标不是符号链接 —— 会在公开仓里新建真文件, 拒绝写入。\n"
            "  先跑 node scripts/setup-private.mjs 建立映射, 或用 --out 显式指定路径。",
            file=sys.stderr,
        )
        return 2

    out_path.write_text(text, encoding="utf-8")
    print(f"built: {out_path} ({why})", file=sys.stderr)
    print(out_path)
    return 0


def cmd_show(args) -> int:
    root = repo_root(Path(args.root) if args.root else None)
    out_path = root / (args.out or DEFAULT_OUT)
    stale, why = is_stale(out_path, args.ttl)
    if stale:
        print(f"error: {why}; 先跑 `hx_persona.py build`", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps({"path": str(out_path), "status": why,
                          "content": out_path.read_text(encoding="utf-8")},
                         ensure_ascii=False, indent=2))
    else:
        print(out_path.read_text(encoding="utf-8"))
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="构建/读取 HXLoLi 用户画像")
    p.add_argument("--root", help="仓库根, 默认向上找同时含 ai-docs/ 与 blog/ 的目录")
    p.add_argument("--out", help=f"输出路径, 默认 {DEFAULT_OUT}")
    p.add_argument("--ttl", type=int, default=DEFAULT_TTL_DAYS, help="缓存有效天数")
    sub = p.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("build", help="构建画像 (缓存未过期则跳过)")
    b.add_argument("--months", type=int, default=DEFAULT_MONTHS, help="回溯窗口月数")
    b.add_argument("--refresh", action="store_true", help="忽略 TTL 强制重建")
    b.set_defaults(func=cmd_build)

    s = sub.add_parser("show", help="打印缓存的画像 (过期则报错)")
    s.add_argument("--json", action="store_true")
    s.set_defaults(func=cmd_show)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
