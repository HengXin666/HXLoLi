"""阶段 9 的交付闸门: 文件齐备、AI 味、标点、tag、引用可达。"""
from __future__ import annotations

import re
import subprocess
from pathlib import Path

from flow.flow_core import (PUNCT_CLI, TAGS_CLI, VOICE_CLI, Flow,
                           _orphan_sidecars, divergence_report, repo_root)


def _run(cmd: list[str], cwd: Path) -> tuple[bool, str]:
    try:
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=180)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"无法执行: {exc}"
    out = (r.stdout + r.stderr).strip()
    return r.returncode == 0, out[-600:]


# 绝对式引用: ".agents/skills/<skill>/..."  本意就是指向仓库里的一个真实文件。
ABS_RE = re.compile(r"[.]agents/skills/[\w./-]+[.](?:py|md|ts|mjs|js|html|toml|json)")
# 相对式引用: 形如 impl/rules.md / shared/voice/impl/blind-audit.md / ../shared/x.md
# 为什么必须也认它: 绝对式检查上线后, 剩余失效引用**全部**是这种写法 (实测一次改名后
# 3 处 references/xxx.md 静默失效, 而 doctor 照样报绿)。只认绝对式等于只堵了一半。
# 前置边界不可省: 没有它, `renderers/shared/x.mjs` 会被截成 `shared/x.mjs` 而误报
# (实测在 hx-archify 的第三方声明里就是这么误伤的)。
REL_RE = re.compile(
    r"(?<![\w/.-])"
    r"(?:[.][.]/)*"
    r"(?:steps|entries|shared|templates|assets|scripts|impl|references)"
    r"/[\w./-]+[.](?:py|md|ts|mjs|js|html|toml|json)")

# 反引号围起来的行内片段。**只豁免其中含 markdown 链接语法的那些** 
# 那种写法是"给人点的链接", 而且常在规范文档里当反例示范 (实测: hx-make-skill
# 用 `[references/x.md](references/x.md)` 演示错误写法)。其余行内 code 照查, 因为
# 本仓的真实引用本来就用反引号包 (如 `impl/rules.md`), 全豁免等于把检查关掉。
INLINE_CODE_RE = re.compile(r"`[^`\n]*`")


def _link_example_spans(line: str) -> list[tuple[int, int]]:
    return [(m.start(), m.end()) for m in INLINE_CODE_RE.finditer(line)
            if "](" in m.group(0)]


def _resolve_skill_ref(md: Path, skills: Path, ref: str) -> Path:
    """相对式引用按三个基准依次解析: 文件所在目录 -> skill 根 -> 仓库根。

    三个都试是刻意的, 因为三种写法在本仓都真实存在:
      - `impl/rules.md`            相对同级
      - `steps/2-collect/impl/x.md` 相对 skill 根
      - `scripts/generateAiDocsSidebar.js` 相对站点根 (skill 描述站点时用, 见 shared/pipeline.md)
    """
    bases = [md.parent, _skill_root_of(md, skills), skills.parent.parent]
    for base in bases:
        if base is None:
            continue
        if (base / ref).resolve().is_file():
            return base / ref
    return md.parent / ref


def _skill_root_of(md: Path, skills: Path) -> Path | None:
    """md 所属 skill 的根目录 (skills/ 下的第一级)。"""
    try:
        rel = md.relative_to(skills)
    except ValueError:
        return None
    return skills / rel.parts[0]


def _broken_skill_paths(root: Path) -> list[str]:
    """扫 .agents/skills 下的 md, 找出指向不存在文件的引用 (绝对式与相对式都认)。

    存在的理由: skill 的参考文档会被反复搬动 (重组目录、合并 skill、改文件名),
    而写死在文档里的路径不会跟着动。这类腐化必须由工具抓, 肉眼看不出来。
    """
    out: list[str] = []
    skills = root / ".agents" / "skills"
    if not skills.is_dir():
        return out
    for md in skills.rglob("*.md"):
        # 第三方依赖与产物目录里的 md 不是本仓的引用来源 (hx-archify 带 node_modules)
        if {"__pycache__", "node_modules", "dist", "build"} & set(md.parts):
            continue
        try:
            text = md.read_text(encoding="utf-8")
        except OSError:
            continue
        in_fence = False
        for i, line in enumerate(text.splitlines(), 1):
            if line.lstrip().startswith("```"):
                in_fence = not in_fence
                continue
            # 围栏里的示例代码不是引用 (规范文档需要用它们示范错误写法)
            if in_fence:
                continue
            exempt = _link_example_spans(line)

            def _skip(start: int) -> bool:
                return any(a <= start < b for a, b in exempt)

            for m in ABS_RE.finditer(line):
                if _skip(m.start()):
                    continue
                if not (root / m.group(0)).exists():
                    out.append(f"{md.relative_to(skills)}:{i} -> {m.group(0)}")
            for m in REL_RE.finditer(line):
                if _skip(m.start()):
                    continue
                ref = m.group(0)
                if not _resolve_skill_ref(md, skills, ref).is_file():
                    out.append(f"{md.relative_to(skills)}:{i} -> {ref}")
    # 同一行的链接文字与链接目标会各命中一次, 去重后再报
    return list(dict.fromkeys(out))


def cmd_doctor(args) -> int:
    root = repo_root(Path(args.root) if args.root else None)
    f = Flow(root, args.slug).load()
    target = f.target
    if target is None:
        raise SystemExit("error: 还没走过阶段 5 place, 没有正式目录可检查")

    index = target / "index.md"
    info = target / ".hx-info.md"
    # 被调脚本的绝对路径来自 flow_core 的唯一定义  写成 root/".agents/skills"
    # 会拼出不存在的路径, 四项检查全部静默失败 (实测踩过: 脚本搬目录后没跟着改)。
    checks: list[tuple[str, bool, str]] = []

    def add(name: str, ok: bool, detail: str = "") -> None:
        checks.append((name, ok, detail))

    add("index.md 存在", index.is_file(), str(index))
    add(".hx-info.md 存在", info.is_file(), str(info))

    if index.is_file():
        txt = index.read_text(encoding="utf-8")
        need = ["hxid", "title", "created_at", "model", "skill", "authors", "tags"]
        lack = [k for k in need if not re.search(rf"^{k}\s*:", txt, re.M)]
        add("frontmatter 字段齐备", not lack, f"缺 {lack}" if lack else "")
        add("正文无 TODO 残留", "TODO" not in txt, "还有 TODO 占位")
        add("正文无过程痕迹", not re.search(r"/Users/|\.agents/|uv run|\.hx-staging", txt),
            "出现了本地路径或命令")
        ok, out = _run(["uv", "run", str(VOICE_CLI),
                        "lint", str(index)], root)
        add("index.md AI 味检查", ok, out)
        ok, out = _run(["uv", "run", str(PUNCT_CLI),
                        "--check", str(index)], root)
        add("index.md 标点规范", ok, out)
        ok, out = _run(["uv", "run", str(TAGS_CLI),
                        "check", str(index)], root)
        add("tag 是注册表里的规范名", ok, out)

    if info.is_file():
        ok, out = _run(["uv", "run", str(VOICE_CLI),
                        "lint", str(info)], root)
        add(".hx-info.md 原子度检查", ok, out)
        if index.is_file():
            a = re.search(r'^hxid:\s*"?([\w-]+)"?', index.read_text(encoding="utf-8"), re.M)
            b = re.search(r'^hxid:\s*"?([\w-]+)"?', info.read_text(encoding="utf-8"), re.M)
            add("两份文件 hxid 一致", bool(a and b and a.group(1) == b.group(1)),
                f"index={a.group(1) if a else None} info={b.group(1) if b else None}")

    sidebar = root / "sidebarsAiDocs.ts"
    if sidebar.is_file():
        # sidebar 脚本会剥掉目录的 `NNN-` 序号前缀再生成 doc id,
        # 所以这里必须用剥掉前缀的名字去搜, 否则永远搜不到 (踩过).
        leaf = re.sub(r"^\d+[-_]", "", target.name)
        add("已注册进 sidebarsAiDocs.ts", leaf in sidebar.read_text(encoding="utf-8"),
            f"没搜到 {leaf!r}, 跑 `node scripts/generateAiDocsSidebar.js`")

    for rel in ("review/voice-audit.md", "review/fidelity-audit.md"):
        add(f"{rel} 存在", (f.dir / rel).is_file(), str(f.dir / rel))

    # --- 可复用物: 笔记里必须有一个"能拿到手"的东西 ---
    #
    # 存在的理由 (实测): 只靠文字要求, 产出会退化成纯散文  读者读完懂了原理,
    # 却拿不到任何能照做的东西, 这篇笔记的参考价值就是零。所以把它变成闸门。
    # 判据是"至少有一件", 不是"必须有多少件": 讲解型笔记本来就该短。
    if index.is_file():
        txt = index.read_text(encoding="utf-8")
        # 侧车: 与 index.md 同目录, 且**真被正文引用**的文件。
        # 只放在目录里不算  实测踩过: 一个无人引用的 .html 侧车会被构建插件
        # 发布到站点上, 成为谁也到不了的死文件。
        referenced = re.findall(r"\]\(([^)\s#][^)\s]*)\)", txt)
        sidecars = [r for r in referenced
                    if re.search(r"\.(tsx|html|drawio\.svg|svg|png|jpe?g|webp)$", r, re.I)
                    and (target / r).is_file()]
        # 代码块: 带分组名的围栏 (平台会渲染成可 tab 切换的对照块, 见 hxloli-md.md)
        code_blocks = re.findall(r"^```[a-zA-Z0-9_+-]+\s+\[", txt, re.M)
        add("笔记里有可复用物 (被引用的侧车或带分组名的代码块)",
            bool(sidecars) or bool(code_blocks),
            "侧车: " + (", ".join(sidecars) if sidecars else "无")
            + f"; 带分组名的代码块: {len(code_blocks)} 处"
            + chr(10) + "读者读完只想「哦原来如此」的纯讲解笔记可以跳过; 其余必须给出一件"
                       "能拿到手的东西 (见 steps/6-derive/impl/reusable-spec.md)")

    # --- 技能文档里的路径是否还指向真东西 ---
    #
    # 为什么放进 doctor: skill 的 reference 会被反复搬动 (重组目录、合并 skill、改文件名),
    # 而写死在文档里的路径不会跟着动。实测踩过  一次批量改名后, 18 处路径全部失效,
    # 而 skill 照样"看起来正常", 直到真去执行才发现脚本不存在。
    # 这类腐化必须由工具抓, 肉眼看不出来。
    broken = _broken_skill_paths(root)
    add("技能文档里的路径全部可达", not broken,
        chr(10).join(broken[:8]) if broken else "")

    orphans = _orphan_sidecars(target)
    add("侧车全部被正文引用", not orphans,
        "没人引用的侧车会被构建发布成死文件: " + ", ".join(orphans))

    # 发散记录: 只报不拦。
    # 为什么不在 doctor 拦: 这个检查在 done atom 时已经硬拦过一次, 到这里再拦只会
    # 卡住 done 时还没加这条闸门的历史流程 (实测: 已有的两篇都停在 3/9 之前)。
    n_div, div_detail = divergence_report(f.dir / "source/divergence.md")
    add(f"发散记录 (>=4 条带判据的产出; 当前 {n_div} 条, 仅报告不拦截)",
        True, div_detail)

    print()
    bad = 0
    for name, ok, detail in checks:
        print(f"  [{'x' if ok else ' '}] {name}")
        if not ok:
            bad += 1
            if detail:
                for line in detail.splitlines()[:6]:
                    print(f"        {line}")
    print()
    print(f"{'PASS' if bad == 0 else 'FAIL'}  {len(checks) - bad}/{len(checks)} 项通过")
    if bad:
        print("\n未通过的项必须修完或在回复里写明为什么跳过。")
    return 1 if bad else 0
