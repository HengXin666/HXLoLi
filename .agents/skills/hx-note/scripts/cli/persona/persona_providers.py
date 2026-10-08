"""数据源: 每个 provider 独立降级, 缺谁就在"数据缺口"里写明。"""
from __future__ import annotations

import re
import subprocess
from datetime import date

from persona.persona_core import (DEFAULT_MIN_ITEMS, Ctx, Section, frontmatter,
                                  parse_day, split_tags, voice_score)

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
    """摘作者的原句。画像只写"在做什么项目"是不够的  派生文章要模仿的是节奏。"""
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
