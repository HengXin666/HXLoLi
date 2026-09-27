# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""沉淀流水线的状态机。

存在的理由 (不是为了好看):
把一次沉淀切成 9 个阶段, 每个阶段只做一件事。但"切开"如果只写在文档里, 模型照样会
在一个阶段里顺手把下一个阶段也做了 —— 于是路径决策的上下文里混进了跟用户的闲聊,
写文章的上下文里混进了目录编号的推理。所以边界必须由脚本强制:

1. `status` 每次只告诉你**当前一个**阶段该做什么、该加载哪个 skill。
2. `done` 会检查该阶段的产物是否真的落盘了, 没落盘就拒绝推进。
3. 阶段间的传递全部走文件, 所以任何阶段都能在一个**新鲜的 agent** 里从磁盘恢复,
   不依赖对话历史还在。

暂存区: ai-docs/.hx-staging/<slug>/  (点开头, 不被 Docusaurus / sidebar / tag 索引 /
quality-gate 收录)。阶段 5 `place` 之后, 笔记产物搬进正式目录, 过程产物留在暂存区。
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from dataclasses import dataclass
from datetime import date
from pathlib import Path

STAGING = "ai-docs/.hx-staging"
FLOW = "flow.json"
# 这些文件属于笔记, place 之后住在正式目录; 其余属于过程, 永远留在暂存区。
NOTE_FILES = {".hx-info.md", ".hx-mitemite.md", "index.md"}


@dataclass
class Stage:
    num: int
    key: str
    what: str
    skill: str
    artifacts: tuple[str, ...]
    exit_check: str


STAGES: tuple[Stage, ...] = (
    Stage(1, "intake", "读懂这次请求: 素材是什么、用户到底要什么。只做定位, 不做任何沉淀",
          "hx-note (本 skill)", (),
          "能用一句话复述用户意图, 且已确认素材可获取"),
    Stage(2, "collect", "采素材: 转写/抓取/截图, 产出可引用的原始材料与 provenance",
          "hx-note 阶段 2 (视频先过 transcribe)",
          ("source/material.md", "source/provenance.md"),
          "material.md 里每个要点都能指回 provenance 里的来源"),
    Stage(3, "atom", "写 .hx-info.md: 纯原子知识点, 面向知识库, 无铺垫无配图",
          "hx-note 阶段 3 (atom-rules.md)", (".hx-info.md",),
          "hx_voice.py lint --profile atom 零 E 级命中"),
    Stage(4, "align", "人类审核 .hx-info.md + 意图同步。只谈内容, 不谈放哪里",
          "hx-note 阶段 4 (queue-and-frontier.md)", (".hx-mitemite.md",),
          "frontier 为空, 且人类明确确认 .hx-info.md 通过"),
    Stage(5, "place", "定路径 + 建目录 + 初始化 index.md 模板 (跑 hx_flow.py place)",
          "hx-note 阶段 5 (hxloli-md.md)", (),
          "target_dir 已记录, index.md 模板已生成, .hx-info.md 已搬进去"),
    Stage(6, "derive", "按模板派生 index.md: 定制引入 + 图文正文 + 定制展望",
          "hx-note 阶段 6 (layering.md + templates/)", ("index.md",),
          "hx_voice.py lint --profile article 零 E 级命中"),
    Stage(7, "review", "人类评审 + AI 味盲审 (sub-agent, 只给 index.md)",
          "hx-note 阶段 7 (blind-audit.md)", ("review/voice-audit.md",),
          "盲审结论已逐条落实或写明不改的理由; 新发现的表达已 learn 进语料库"),
    Stage(8, "fidelity", "保真盲审 (sub-agent, 给 .hx-info.md + index.md)",
          "hx-note 阶段 8 (blind-audit.md)", ("review/fidelity-audit.md",),
          "报告判定 PASS; 新增/歪曲两节为空"),
    Stage(9, "land", "落地注册: sidebar / 标点 / tag / hxid 闸门 (跑 hx_flow.py doctor)",
          "hx-note", (),
          "doctor 全绿"),
)

BY_KEY = {s.key: s for s in STAGES}
BY_NUM = {s.num: s for s in STAGES}


def repo_root(start: Path | None = None) -> Path:
    cur = (start or Path.cwd()).resolve()
    while True:
        if (cur / "ai-docs").is_dir() and (cur / "docusaurus.config.ts").is_file():
            return cur
        if cur.parent == cur:
            return (start or Path.cwd()).resolve()
        cur = cur.parent


def slugify(raw: str) -> str:
    s = re.sub(r"[^\w\u4e00-\u9fff-]+", "-", raw.strip()).strip("-")
    return (s or "untitled")[:60]


class Flow:
    def __init__(self, root: Path, slug: str):
        self.root = root
        self.slug = slug
        self.dir = root / STAGING / slug
        self.path = self.dir / FLOW
        self.data: dict = {}

    # --- 持久化 ---
    def load(self) -> Flow:
        if not self.path.is_file():
            raise SystemExit(f"error: 没有找到 {self.path}; 先跑 `hx_flow.py init --slug {self.slug} ...`")
        self.data = json.loads(self.path.read_text(encoding="utf-8"))
        return self

    def save(self) -> None:
        self.dir.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(self.data, ensure_ascii=False, indent=2) + "\n",
                             encoding="utf-8")

    # --- 位置解析 ---
    @property
    def target(self) -> Path | None:
        t = self.data.get("target_dir")
        return (self.root / t) if t else None

    def locate(self, rel: str) -> Path:
        """产物可能在暂存区也可能已搬进正式目录, 按实际存在的那个算。"""
        cands: list[Path] = []
        if self.target and rel in NOTE_FILES:
            cands.append(self.target / rel)
        cands.append(self.dir / rel)
        for c in cands:
            if c.exists():
                return c
        return cands[0]

    # --- 阶段 ---
    def done_set(self) -> set[str]:
        return set(self.data.get("done", []))

    def current(self) -> Stage | None:
        done = self.done_set()
        for s in STAGES:
            if s.key not in done:
                return s
        return None

    def missing(self, stage: Stage) -> list[str]:
        return [rel for rel in stage.artifacts if not self.locate(rel).exists()]


# --------------------------------------------------------------------------
def cmd_init(args) -> int:
    root = repo_root(Path(args.root) if args.root else None)
    slug = slugify(args.slug or args.title or "")
    f = Flow(root, slug)
    if f.path.is_file() and not args.force:
        raise SystemExit(f"error: {f.path} 已存在; 加 --force 重置 (会清空 done 记录)")
    f.data = {
        "slug": slug,
        "title": args.title or "",
        "source": args.source or "",
        "kind": args.kind,
        "created_at": date.today().isoformat(),
        "target_dir": "",
        "done": [],
        "notes": {},
    }
    (f.dir / "source").mkdir(parents=True, exist_ok=True)
    (f.dir / "review").mkdir(parents=True, exist_ok=True)
    f.save()
    print(f"created: {f.path}")
    print_status(f)
    return 0


# ----------------------------------------------------------------
# 语料强制注入
#
# 问题: 语料库 (好句/坏句/梗) 一直被记下来, 但从没出现在写作现场的上下文里。
# 实测过: 攒了 37 条, 而写引入时一条都想不起来 —— 没有任何机制让它出现在眼前。
#
# 解法: 挂在 status 的输出里。它是每一步都必须调的入口, 所以语料**每一轮都在场**。
# 成本可承受: 正文合计约 1000 字符 (中文 1 字 1 token), 相对一次写作的上下文可以忽略。
#
# 关键是: 这不是『提示模型去读』(依赖自觉), 而是『把东西摆在眼前』(不依赖)。
# (see .agents/notes/implemented/architecture/2026-09-26-voice-corpus-forced-injection.md — 为什么语料必须强制在场)
def voice_corpus_lines(root: Path) -> list[str]:
    db = root / 'ai-docs' / '.hx-voice.toml'
    if not db.is_file():
        return []
    try:
        raw = db.read_text(encoding='utf-8')
    except OSError:
        return []
    # 按行解析, 不上正则 —— 这种小格式用逐行判断更清楚, 也不会在转义上翻车。
    out: list[str] = []
    cur_kind: str | None = None
    bucket: dict[str, list[str]] = {'good': [], 'bad': [], 'meme': []}
    for line in raw.splitlines():
        stripped = line.strip()
        if stripped.startswith('[[') and stripped.endswith(']]'):
            name = stripped[2:-2]
            cur_kind = name if name in bucket else None
            continue
        if cur_kind and stripped.startswith('text'):
            _, _, value = stripped.partition('=')
            value = value.strip().strip('"')
            if value:
                bucket[cur_kind].append(value)
    for kind, label in (('good', '值得模仿'), ('bad', '别这么写'), ('meme', '梗')):
        items = bucket[kind]
        if not items:
            continue
        out.append('  ' + label + ' (' + str(len(items)) + '):')
        for t in items[:12]:
            out.append('    · ' + t[:76])
    return out

def print_status(f: Flow) -> None:
    cur = f.current()
    done = f.done_set()
    print()
    # 语料每次都在场。放在最开头是刻意的 —— 后面的分支会提前 return,
    # 而语料必须在**每一个**阶段都出现, 不能因为走到某个分支就没了。
    corpus = voice_corpus_lines(f.root)
    if corpus:
        print('┌─ 表达语料 (改稿前扫一眼; 详见 ai-docs/.hx-voice.toml)')
        for line in corpus:
            print('│' + line)
        print('└' + '─' * 62)
    print(f"# 沉淀流程  slug={f.slug}  kind={f.data.get('kind')}")
    if f.data.get("source"):
        print(f"  素材: {f.data['source']}")
    print(f"  暂存: {f.dir}")
    print(f"  笔记: {f.data.get('target_dir') or '(阶段 5 才确定)'}")
    print()
    for s in STAGES:
        mark = "x" if s.key in done else (">" if cur and s.key == cur.key else " ")
        print(f"  [{mark}] {s.num}. {s.key:<9} {s.what[:52]}")
    print()
    if cur is None:
        print("全部阶段已完成。交付前再跑一次 `hx_flow.py doctor`。")
        return
    miss = f.missing(cur)
    print("=" * 64)
    print(f"当前只做这一件事 -> 阶段 {cur.num} `{cur.key}`")
    print("=" * 64)
    print(f"  做什么  : {cur.what}")
    print(f"  加载    : {cur.skill}")
    print(f"  过关条件: {cur.exit_check}")
    if cur.artifacts:
        print("  产物    :")
        for rel in cur.artifacts:
            p = f.locate(rel)
            print(f"      [{'x' if p.exists() else ' '}] {p}")
    if miss:
        print(f"  还缺    : {', '.join(miss)}")
    print()
    print(f"  完成后  : hx_flow.py done {cur.key} --slug {f.slug}")
    print()
    print("  不要顺手做下一个阶段。下一阶段要换 skill、换关注点, 混在一起就是这次重构要消灭的问题。")


def cmd_status(args) -> int:
    root = repo_root(Path(args.root) if args.root else None)
    f = Flow(root, args.slug).load()
    if args.json:
        cur = f.current()
        print(json.dumps({
            "slug": f.slug, "target_dir": f.data.get("target_dir"),
            "done": sorted(f.done_set()),
            "current": None if cur is None else {
                "num": cur.num, "key": cur.key, "what": cur.what,
                "skill": cur.skill, "exit_check": cur.exit_check,
                "artifacts": [str(f.locate(r)) for r in cur.artifacts],
                "missing": f.missing(cur),
            },
        }, ensure_ascii=False, indent=2))
        return 0
    print_status(f)
    return 0


def cmd_done(args) -> int:
    root = repo_root(Path(args.root) if args.root else None)
    f = Flow(root, args.slug).load()
    key = args.stage if args.stage in BY_KEY else None
    if key is None and args.stage.isdigit() and int(args.stage) in BY_NUM:
        key = BY_NUM[int(args.stage)].key
    if key is None:
        raise SystemExit(f"error: 未知阶段 {args.stage!r}; 可用: "
                         + ", ".join(f"{s.num}/{s.key}" for s in STAGES))
    stage = BY_KEY[key]

    earlier = [s.key for s in STAGES if s.num < stage.num and s.key not in f.done_set()]
    if earlier and not args.force:
        raise SystemExit(f"error: 阶段 {earlier} 还没完成, 不允许跳阶段 (真要跳加 --force 并写 --note 说明原因)")

    miss = f.missing(stage)
    if miss and not args.force:
        raise SystemExit("error: 产物缺失, 拒绝推进:\n  "
                         + "\n  ".join(str(f.locate(m)) for m in miss)
                         + "\n(确实不需要这些产物时加 --force 并用 --note 写明原因)")

    f.data.setdefault("done", [])
    if key not in f.data["done"]:
        f.data["done"].append(key)
    if args.note:
        f.data.setdefault("notes", {})[key] = args.note
    if miss and args.force:
        f.data.setdefault("notes", {})[key + ":skipped"] = f"缺 {miss}; {args.note or '未说明原因'}"
    f.save()
    print(f"done: {stage.num} {key}")
    print_status(f)
    return 0


def cmd_place(args) -> int:
    """阶段 5: 唯一允许创建正式目录的地方。"""
    root = repo_root(Path(args.root) if args.root else None)
    f = Flow(root, args.slug).load()
    if "align" not in f.done_set() and not args.force:
        raise SystemExit("error: 阶段 4 align 还没完成 —— 路径要在人类确认 .hx-info.md 之后才定")

    target = (root / args.to).resolve()
    if not str(target).startswith(str((root / "ai-docs").resolve())):
        raise SystemExit(f"error: --to 必须落在 ai-docs/ 下, 收到 {target}")
    target.mkdir(parents=True, exist_ok=True)

    index = target / "index.md"
    if not index.exists():
        make_doc = root / ".agents/skills/hx-note/scripts/makeDoc.py"
        if not make_doc.is_file():
            raise SystemExit(f"error: 找不到 {make_doc}")
        cmd = ["uv", "run", str(make_doc), "--output", str(index),
               "--skill", "hx-note"]
        if args.title or f.data.get("title"):
            cmd += ["--title", args.title or f.data["title"]]
        for t in (args.tag or []):
            cmd += ["--tag", t]
        if args.model:
            cmd += ["--model", args.model]
        r = subprocess.run(cmd, cwd=root, text=True)
        if r.returncode != 0:
            raise SystemExit("error: makeDoc.py 失败, 不继续搬迁")
    else:
        print(f"note: {index} 已存在, 跳过 makeDoc.py (将在其基础上编辑)")

    # 搬迁笔记类产物; 过程产物 (source/ review/ flow.json) 留在暂存区
    for rel in (".hx-info.md", ".hx-mitemite.md"):
        src = f.dir / rel
        dst = target / rel
        if src.exists() and not dst.exists():
            dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
            src.unlink()
            print(f"moved: {rel} -> {dst}")

    # hxid 对齐: .hx-info.md 必须沿用 index.md 的 hxid (同一篇笔记的两个面)
    info = target / ".hx-info.md"
    if info.exists() and index.exists():
        m = re.search(r'^hxid:\s*"?([\w-]+)"?', index.read_text(encoding="utf-8"), re.M)
        if m:
            txt = info.read_text(encoding="utf-8")
            new = re.sub(r'^hxid:\s*.*$', f'hxid: "{m.group(1)}"', txt, count=1, flags=re.M)
            if new != txt:
                info.write_text(new, encoding="utf-8")
                print(f"aligned: .hx-info.md hxid -> {m.group(1)}")

    f.data["target_dir"] = str(target.relative_to(root))
    f.data.setdefault("done", [])
    if "place" not in f.data["done"]:
        f.data["done"].append("place")
    f.save()
    print_status(f)
    return 0


# --- doctor ---------------------------------------------------------------
def _run(cmd: list[str], cwd: Path) -> tuple[bool, str]:
    try:
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=180)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"无法执行: {exc}"
    out = (r.stdout + r.stderr).strip()
    return r.returncode == 0, out[-600:]


def _broken_skill_paths(root: Path) -> list[str]:
    """扫 .agents/skills 下的 md, 找出指向不存在文件的 .agents/skills/... 路径。

    只认形如 ".agents/skills/<skill>/..." 的绝对式引用 —— 这类写法本意就是"指向仓库里的
    一个真实文件", 失效即是 bug。相对路径不管 (它们可能是引用同目录的兄弟文件)。
    """
    pat = re.compile(r"[.]agents/skills/[\w./-]+[.](?:py|md|ts|mjs|js|html|toml|json)")
    out: list[str] = []
    skills = root / ".agents" / "skills"
    if not skills.is_dir():
        return out
    for md in skills.rglob("*.md"):
        if "__pycache__" in md.parts:
            continue
        try:
            text = md.read_text(encoding="utf-8")
        except OSError:
            continue
        for i, line in enumerate(text.splitlines(), 1):
            for m in pat.finditer(line):
                target = m.group(0)
                if not (root / target).exists():
                    out.append(f"{md.relative_to(skills)}:{i} -> {target}")
    return out


def cmd_doctor(args) -> int:
    root = repo_root(Path(args.root) if args.root else None)
    f = Flow(root, args.slug).load()
    target = f.target
    if target is None:
        raise SystemExit("error: 还没走过阶段 5 place, 没有正式目录可检查")

    index = target / "index.md"
    info = target / ".hx-info.md"
    # 本 skill 自己的脚本目录。**不要写成 root/".agents/skills"** —— 那会拼出
    # .agents/skills/hx_voice.py 这种不存在的路径, 四项检查全部静默失败 (实测踩过:
    # hx-note 合并时脚本从 <旧skill>/scripts/ 搬到了 hx-note/scripts/, 这行没跟着改)。
    sk = Path(__file__).resolve().parent
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
        ok, out = _run(["uv", "run", str(sk / "hx_voice.py"),
                        "lint", str(index)], root)
        add("index.md AI 味检查", ok, out)
        ok, out = _run(["uv", "run", str(sk / "format_cn_punct.py"),
                        "--check", str(index)], root)
        add("index.md 标点规范", ok, out)
        ok, out = _run(["uv", "run", str(sk / "hxloli_tags.py"),
                        "check", str(index)], root)
        add("tag 是注册表里的规范名", ok, out)

    if info.is_file():
        ok, out = _run(["uv", "run", str(sk / "hx_voice.py"),
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

    # --- 技能文档里的路径是否还指向真东西 ---
    #
    # 为什么放进 doctor: skill 的 reference 会被反复搬动 (重组目录、合并 skill、改文件名),
    # 而写死在文档里的路径不会跟着动。实测踩过 —— 一次批量改名后, 18 处路径全部失效,
    # 而 skill 照样"看起来正常", 直到真去执行才发现脚本不存在。
    # 这类腐化必须由工具抓, 肉眼看不出来。
    broken = _broken_skill_paths(root)
    add("技能文档里的路径全部可达", not broken,
        chr(10).join(broken[:8]) if broken else "")

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


def cmd_list(args) -> int:
    root = repo_root(Path(args.root) if args.root else None)
    base = root / STAGING
    if not base.is_dir():
        print("(暂存区为空)")
        return 0
    for d in sorted(base.iterdir()):
        fp = d / FLOW
        if not fp.is_file():
            continue
        data = json.loads(fp.read_text(encoding="utf-8"))
        done = data.get("done", [])
        cur = next((s for s in STAGES if s.key not in set(done)), None)
        print(f"{d.name:<40} {len(done)}/9  "
              f"{'完成' if cur is None else f'阶段 {cur.num} {cur.key}'}  "
              f"{data.get('target_dir') or ''}")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="ai-docs 沉淀流水线状态机")
    p.add_argument("--root", help="仓库根, 默认向上找含 ai-docs/ 与 docusaurus.config.ts 的目录")
    sub = p.add_subparsers(dest="cmd", required=True)

    i = sub.add_parser("init", help="开一条新流程")
    i.add_argument("--slug", help="暂存目录名; 不给则从 --title 推")
    i.add_argument("--title", help="拟定标题")
    i.add_argument("--source", help="素材 URL 或路径")
    i.add_argument("--kind", default="article",
                   choices=["video", "article", "custom", "local", "research"])
    i.add_argument("--force", action="store_true")
    i.set_defaults(func=cmd_init)

    s = sub.add_parser("status", help="当前该做哪一件事")
    s.add_argument("--slug", required=True)
    s.add_argument("--json", action="store_true")
    s.set_defaults(func=cmd_status)

    d = sub.add_parser("done", help="标记某阶段完成 (产物缺失会被拒)")
    d.add_argument("stage", help="阶段号或 key")
    d.add_argument("--slug", required=True)
    d.add_argument("--note", help="这一阶段的结论/取舍, 会写进 flow.json")
    d.add_argument("--force", action="store_true", help="绕过产物检查, 必须配 --note")
    d.set_defaults(func=cmd_done)

    pl = sub.add_parser("place", help="阶段 5: 建正式目录 + 初始化 index.md + 搬迁产物")
    pl.add_argument("--slug", required=True)
    pl.add_argument("--to", required=True, help="ai-docs 下的目标目录 (相对仓库根)")
    pl.add_argument("--title")
    pl.add_argument("--tag", action="append")
    pl.add_argument("--model")
    pl.add_argument("--force", action="store_true")
    pl.set_defaults(func=cmd_place)

    dc = sub.add_parser("doctor", help="阶段 9: 跑全部交付闸门")
    dc.add_argument("--slug", required=True)
    dc.set_defaults(func=cmd_doctor)

    ls = sub.add_parser("list", help="列出暂存区里所有流程")
    ls.set_defaults(func=cmd_list)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())