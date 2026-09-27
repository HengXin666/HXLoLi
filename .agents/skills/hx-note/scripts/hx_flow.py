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
GOAL = "source/goal.md"
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
          "hx-note (本 skill)", ("source/goal.md",),
          "goal.md 里 >=1 条可判定的验收条款 (带检查动作); 且已确认素材可获取"),
    Stage(2, "collect", "采素材: 转写/抓取/截图, 产出可引用的原始材料与 provenance",
          "hx-note 阶段 2 (视频先过 transcribe)",
          ("source/material.md", "source/provenance.md"),
          "material.md 里每个要点都能指回 provenance 里的来源"),
    Stage(3, "atom", "先跑发散算子造出素材没有的结论, 再写 .hx-info.md",
          "hx-note 阶段 3 (steps/3-atom/impl/divergence.md + rules.md)",
          (".hx-info.md", "source/divergence.md"),
          "source/divergence.md 含 >=4 条带判据的产出; 且 hx_voice.py lint --profile atom 零 E 级命中"),
    Stage(4, "align", "逼问式对齐: 一次一题, 锐化模糊词, 对齐结果落 goal.md 术语表",
          "hx-note 阶段 4 (steps/4-align/impl/grill.md 先读, 再看 frontier.md)",
          (".hx-mitemite.md", "source/goal.md"),
          "每题人类都答过; goal.md 术语表已锐化; frontier 为空"),
    Stage(5, "place", "定路径 + 建目录 + 初始化 index.md 模板 (跑 hx_flow.py place)",
          "hx-note 阶段 5 (shared/hxloli-md.md)", (),
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
    if cur.key == "align":
        items = read_answer_card(f.locate(".hx-mitemite.md"))
        pending = [it for it in items if not it["answered"]]
        print(f"  待人类作答: {len(pending)}/{len(items)} 题未答")
        print(f"  端给人类  : hx_flow.py brief --slug {f.slug}   (**先把问题贴出来, 再停下等回答**)")
        print("  注意      : 把推荐答案写进问题里不算作答; 未答会被 done 闸门拒绝")
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

    # 阶段 3 的额外硬闸门: 发散记录必须真的产出过东西。
    # 放在 done 而不是 doctor: 这是关键路径, 跳过它整条流水线就退化成复述,
    # 而 doctor 是交付前的最后一道, 到那时内容已经写完, 改回来代价太大。
    # 阶段 1 的硬闸门: 需求必须落成可判定的条款。
    # Stage(1).artifacts 原先是**空的** —— 实测结果是需求写不写流程完全不管,
    # 于是同一个任务里出现了 4 份各不相同的"用户要什么", 而没人回对过最初那句话。
    if key == "intake" and not args.force:
        why = goal_gate(f.locate(GOAL))
        if why:
            raise SystemExit("error: " + why
                             + "\n  (素材确实没有可判定的目标时加 --force 并写明原因)")

    # 阶段 4 的硬闸门: 人类必须真的答过。
    # 只要求"文件存在"是不够的 —— 实测踩过: 把推荐答案写进 Q、A 留空、
    # 直接 done align, 闸门照样放行, 于是"人类审核"这一步在事实上被跳过了。
    if key == "align" and not args.force:
        card = f.locate(".hx-mitemite.md")
        items = read_answer_card(card)
        pending = [it for it in items if not it["answered"]]
        if not items:
            raise SystemExit(
                "error: 答题卡里没有任何问题块 (或格式不合法), 阶段 4 无从审核。\n"
                "  先把本轮 frontier 的问题写进 %s\n"
                "  格式: steps/4-align/impl/frontier.md" % card)
        if pending:
            seqs = ", ".join(it["seq"] for it in pending)
            raise SystemExit(
                "error: %d/%d 题还没被回答, 拒绝推进: %s\n"
                "  **必须把问题端给人类, 并等人类作答。** 推荐答案写进问题里不算作答。\n"
                "  端给人类: hx_flow.py brief --slug %s\n"
                "  人类作答后这次 done 才会放行。确属人类明确授权按推荐执行的, "
                % (len(pending), len(items), seqs, f.slug)
                + "加 --force 并在 --note 里写明是谁、什么时候授权的。")

    if key == "atom" and not args.force:
        div = f.locate("source/divergence.md")
        n, detail = divergence_report(div)
        if not div.is_file():
            raise SystemExit(
                "error: 缺 source/divergence.md —— 阶段 3 必须先跑发散算子再写 .hx-info.md。\n"
                "  做法: steps/3-atom/impl/divergence.md (四个算子); 格式: assets/divergence-template.md\n"
                "  素材是纯资讯类、确实无可发散结构时加 --force 并在 --note 里写明原因")
        if n < 4:
            raise SystemExit(
                "error: source/divergence.md 里只有 %d 条带判据的产出, 要求 >= 4。\n"
                "  %s\n"
                "  每条的'结论'格必须以 %s 开头, 并在'判据/去向'格给出可查的动作。"
                % (n, detail or "先把四个算子各跑一次", DIVERGENT_MARK)
                + "\n  确实无可发散结构时加 --force 并在 --note 里写明原因")

    f.data.setdefault("done", [])
    if key not in f.data["done"]:
        f.data["done"].append(key)
    if args.note:
        f.data.setdefault("notes", {})[key] = args.note
    if miss and args.force:
        f.data.setdefault("notes", {})[key + ":skipped"] = f"缺 {miss}; {args.note or '未说明原因'}"
    # 记下这次 align 是不是在"人类未答"的情况下靠 --force 过的。
    # place 靠这个标记决定要不要再拦一次 —— 只看 done_set 是不够的,
    # 因为 --force 也会把 align 写进 done_set。
    if key == "align" and args.force:
        pending = [it for it in read_answer_card(f.locate(".hx-mitemite.md"))
                   if not it["answered"]]
        f.data["align_forced"] = True
        if pending:
            print(f"warn: 阶段 4 带 --force 通过, 但有 {len(pending)} 题未答 —— place 时会再拦一次")
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

    # align 若被 --force 标记完成 (人类未作答), 这里再拦一次。
    # 两处都拦的理由: done 是"AI 自己宣布过了", place 是"真的开始动正式目录" ——
    # 后者是把未审内容铺到站点上的最后一道口子。
    if not args.force and "align" in f.done_set() and f.data.get("align_forced"):
        items = read_answer_card(f.locate(".hx-mitemite.md"))
        pending = [it for it in items if not it["answered"]]
        # 空卡也算未通过: "一题都没有" 与 "全答完了" 是两种完全不同的状态,
        # 前者说明压根没审。实测踩过: 只判 pending 时, 空卡会静默放行 (0 条 pending)。
        if pending or not items:
            detail = (", ".join(it["seq"] for it in pending) if pending
                      else "答题卡为空, 没有任何问题被审核过")
            raise SystemExit(
                "error: 阶段 4 是带 --force 跳过的 (人类未作答), 拒绝落正式目录。\n"
                "  未答/未审: %s\n"
                "  先跑: hx_flow.py brief --slug %s" % (detail, f.slug))

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


# --- 答题卡: 读数与待答判定 ------------------------------------------------
#
# 为什么需要这一组函数 (实测三个缺口):
#   1. 答题卡是点开头文件, 且住在点开头目录 ai-docs/.hx-staging/<slug>/ 下 ——
#      普通 ls 看不到, dev-edit-server.mjs 也跳过点开头项, 所以暂存区**没有 UI**。
#      人类唯一的入口是对话, 而没有任何东西强制把问题端出来。
#   2. Stage(4).artifacts 只要求 .hx-mitemite.md **存在**, 不检查 A 填没填 ——
#      于是"AI 把推荐答案写进 Q, 留空 A, 直接 done align"能过闸门。
#   3. 后果实测: 全库 16 份答题卡里 11 份的 A 从未被回答过, 包括已发布的笔记。
#
# 所以这里补两样: 一个把问题端到眼前的读数 (brief), 一个"未答不许推进"的闸门。
# 块头有三种实测格式, 都要认 (人手工追加的块不带哈希):
#   ## 0x00 3b596b54 begin {   <- 脚本生成
#   ## 0x05 (新) begin {       <- 人手工追加
#   ## (新) 0x06 begin {       <- 人手工追加的另一种写法
# 块头有四种实测格式, 全部要认 (人工手写的变体是常态, 不是异常):
#   ## 0x00 3b596b54 begin {   <- 脚本生成 (hashtag 校验块)
#   ## 0x05 (新) begin {       <- 人手工追加
#   ## (新) 0x06 begin {       <- 人手工追加的另一种写法
#   ## 0x00 Q1 读者是谁        <- 完全手写的卡 (无 begin/哈希)
# 判据是「## 后跟一个 0xNN 开头的块」而不是某个固定尾串 —— 卡是给人改的,
# 要求人按脚本格式写等于把格式错误变成解析失败, 进而把"未答"误判成"没有问题"。
BLOCK_RE = re.compile(r"^##\s+(?:\(新\)\s*)?(0x[0-9A-Fa-f]{2})\b(.*)$", re.M)
# 答案体: 从 **A**: 到收尾的 `}` 或下一个块为止
A_BODY_RE = re.compile(r"\*\*A\*\*:\s*(.*?)(?=\n\}\s*$|\n##\s|\Z)", re.S | re.M)


def read_answer_card(path: Path) -> list[dict]:
    """解析答题卡的每个问题块, 返回 [{seq, q, a, answered}]。

    解析失败不抛异常: 答题卡是给人改的文件, 手改会破坏格式。宁可返回空列表
    (上层会提示"格式异常"), 也不要让一条格式问题把整条流水线卡死。
    """
    if not path.is_file():
        return []
    text = path.read_text(encoding="utf-8")
    marks = list(BLOCK_RE.finditer(text))
    out: list[dict] = []
    for i, m in enumerate(marks):
        end = marks[i + 1].start() if i + 1 < len(marks) else len(text)
        body = text[m.end():end]
        # 问题: 块头标题 + 到 **A**: 为止的正文
        head = m.group(2).strip()
        q = body.split("**A**:", 1)[0]
        q = q.replace("**Q**:", "").strip()
        # 手写的卡把问题写在标题行 (如 `0x00 Q1 读者是谁`), 也要收进来
        head_tail = re.sub(r"^(?:Q\d+\s*[:：]?\s*)", "", head).strip()
        if head_tail and head_tail.lower() not in ("begin {", "begin", "{"):
            q = (head_tail + "\n" + q).strip()
        # 答案: **A**: 之后到收尾花括号为止。
        # **必须剥掉收尾的 `}`** —— 实测踩过: 不剥时惰性匹配会把 `}` 当成答案内容,
        # 于是全库 11 份从未被回答的答题卡一律被判成"已答", 闸门形同虚设。
        am = A_BODY_RE.search(body)
        a = am.group(1) if am else ""
        a = a.strip().strip("}").strip()
        # 只有标点/空白不算答过
        answered = len(re.sub(r"[\s\-—.。,:：;；\]\[]", "", a)) > 0
        out.append({"seq": m.group(1), "q": q, "a": a, "answered": answered})
    return out



# --- 目标契约: 验收条款与需求覆盖 ----------------------------------------
#
# 存在的理由 (实测): 需求此前只活在对话历史与若干随手记的文件里。实测同一个任务里
# 出现过 4 份各不相同的需求表述 (intake.md / 答题卡 / intent-ir.md / STATE.md),
# 全部手工维护、无一致性检查, 而 Stage(1).artifacts 当时是**空的** —— 需求写不写
# 流程完全不管。于是上下文一压缩, 需求只剩摘要, 没人拿最终产物回对过最初那句话。
#
# 这里补三样: 验收条款的机械判据、需求到知识点的反向覆盖、条款漂移的留痕。
GOAL_ROW_RE = re.compile(r'^\|\s*(G\d+)\s*\|', re.M)
# 验收栏里必须出现可执行动作, 否则不算条款
ACCEPT_ACTION_RE = re.compile(
    r'查|跑|测|数|看|grep|npm|node|python|uv |命令|文件|目录|截图|渲染|行数|退出码|对比')
PLACEHOLDER_CELL_RE = re.compile(r'^<.*>$|^[\s.…\-—]*$')


def _section_rows(text: str, title: str) -> int:
    '''数某一节里的有效表格行 (排除表头分隔行与占位行)。'''
    if title not in text:
        return 0
    seg = text.split(title, 1)[1].split(chr(10) + '## ', 1)[0]
    n = 0
    for line in seg.splitlines():
        s = line.strip()
        if not s.startswith('|'):
            continue
        if set(s.strip('|')) <= set('-: '):
            continue
        cells = [c.strip() for c in s.strip('|').split('|')]
        if cells and not PLACEHOLDER_CELL_RE.match(cells[0]):
            n += 1
    return n


def read_goal(path: Path) -> dict:
    '''解析 goal.md: 返回 items / terms / drifted / problem。

    验收栏的机械要求是出现可执行动作 —— 写不出检查动作的条款是愿望, 不是条款。
    '''
    if not path.is_file():
        return {'items': [], 'terms': 0, 'drifted': 0, 'problem': '文件不存在'}
    text = path.read_text(encoding='utf-8')
    items: list[dict] = []
    for line in text.splitlines():
        if not GOAL_ROW_RE.match(line):
            continue
        cells = [c.strip() for c in line.strip().strip('|').split('|')]
        if len(cells) < 4 or set(''.join(cells)) <= set('-: '):
            continue
        gid, clause, check = cells[0], cells[1], cells[2]
        ok = (not PLACEHOLDER_CELL_RE.match(clause)
              and not PLACEHOLDER_CELL_RE.match(check)
              and bool(ACCEPT_ACTION_RE.search(check)))
        items.append({'gid': gid, 'clause': clause, 'check': check,
                      'ok': ok, 'status': cells[3] if len(cells) > 3 else ''})
    return {'items': items,
            'terms': _section_rows(text, '术语表'),
            'drifted': _section_rows(text, '漂移记录'),
            'problem': ''}


def goal_gate(path: Path) -> str:
    '''阶段 1 的闸门: 返回空串表示通过, 否则返回拒绝原因。'''
    if not path.is_file():
        return ('缺 %s —— 阶段 1 必须先把用户要什么落成可判定的条款。'
                '模板与格式见 assets/goal-template.md' % GOAL)
    g = read_goal(path)
    if not g['items']:
        return ('%s 里没有任何 G 编号的验收条款。'
                '每条形如: | G1 | <可判定的说法> | <怎么算满足> | 待验 |' % GOAL)
    bad = [it['gid'] for it in g['items'] if not it['ok']]
    if bad:
        return ('%d 条条款的验收栏写不出可执行动作, 它们只是愿望: %s。'
                '判据: 那一栏要出现 查/跑/测/数/看 这类动作词, 或具体文件名与命令。'
                '改法: 补出检查动作, 或把它移到背景一节 (背景不验收)。'
                % (len(bad), ', '.join(bad)))
    return ''


def cmd_cover(args) -> int:
    '''把每条验收条款被多少条知识点支撑列出来。

    这是**需求偏移的机械暴露**: 零支撑的条款说明这条需求从没被实现过。
    做法: 在 .hx-info.md 里找 满足标记 (形如 减号 满足: G3)。
    '''
    root = repo_root(Path(args.root) if args.root else None)
    f = Flow(root, args.slug).load()
    goal = read_goal(f.locate(GOAL))
    info = f.locate('.hx-info.md')
    text = info.read_text(encoding='utf-8') if info.is_file() else ''
    marks = re.findall(r'满足\s*[:：]\s*([Gg]\d+(?:\s*[,，]\s*[Gg]\d+)*)', text)
    hits: dict[str, int] = {}
    for grp in marks:
        for gid in re.split(r'[,，]', grp):
            gid = gid.strip().upper()
            if gid:
                hits[gid] = hits.get(gid, 0) + 1
    print('# 需求覆盖  slug=%s' % f.slug)
    print()
    if not goal['items']:
        print('goal.md 里没有验收条款 —— 先跑阶段 1。见 assets/goal-template.md')
        return 1
    orphan = []
    for it in goal['items']:
        n = hits.get(it['gid'], 0)
        flag = '  <-- 零支撑: 这条需求从没被实现过' if n == 0 else ''
        print('  %-4s %3d 条知识点  %s%s' % (it['gid'], n, it['clause'][:56], flag))
        if n == 0:
            orphan.append(it['gid'])
    known = {it['gid'] for it in goal['items']}
    unknown = sorted(set(hits) - known)
    if unknown:
        print()
        print('以下标记指向不存在的条款 (编号写错, 或条款已被删):')
        for gid in unknown:
            print('    %s (%d 处)' % (gid, hits[gid]))
    print()
    if orphan:
        print('FAIL: %d 条需求零支撑: %s' % (len(orphan), ', '.join(orphan)))
        print('  要么补知识点, 要么把它们标成已放弃 (原因写清)。')
        return 1
    print('PASS: 每条需求都有知识点支撑。')
    return 0



def cmd_brief(args) -> int:
    """把"要你审什么、东西在哪、怎么答"一次打全。

    存在的理由: 人类的抱怨是"不知道他要我审核的东西在哪"。问题不在人类不看,
    在于从来没有一个地方把"问题 + 被审对象 + 作答方法"放在同一屏里。
    """
    root = repo_root(Path(args.root) if args.root else None)
    f = Flow(root, args.slug).load()
    card = f.locate(".hx-mitemite.md")
    info = f.locate(".hx-info.md")
    target = f.target
    index = (target / "index.md") if target else None

    print(f"# 待你审核: {f.data.get('title') or f.slug}")
    print()
    print("## 被审的东西在哪 (别去找, 直接打开这几个)")
    for label, p in (("目标与验收契约", f.locate(GOAL)), ("知识点事实源", info),
                     ("面向读者的正文", index), ("答题卡 (你的答案写这里)", card)):
        if p is None:
            print(f"  - {label}: (还没生成)")
            continue
        mark = "x" if p.is_file() else " "
        print(f"  [{mark}] {label}: {p}")
    print()
    print("说明: 暂存区是点开头目录, 文件也是点开头. 普通 ls 看不见它们,")
    print("      所以上面给的是完整路径, 直接复制到编辑器打开即可。")
    print()

    items = read_answer_card(card)
    if not items:
        print("## 还没有问题")
        print()
        print("本阶段还没出题。执行者跑完步骤 3 会把问题写进上面那张答题卡。")
        return 0

    pending = [it for it in items if not it["answered"]]
    answered = [it for it in items if it["answered"]]
    print(f"## 你要回答的 ({len(pending)} 题未答 / 共 {len(items)} 题)")
    print()
    if not pending:
        print("  全部已答, 可以直接推进。")
    for it in pending:
        print(f"### {it['seq']}")
        print(it["q"] or "(问题正文为空)")
        print()
    if answered:
        print(f"## 已答的 ({len(answered)} 题, 仅列序号)")
        print("  " + ", ".join(card_seq(it) for it in answered))
        print()

    print("## 怎么答 (两种, 任选)")
    print()
    print("方式一 (推荐, 直接改文件): 用编辑器打开上面那张答题卡,")
    print("      在每个答案标记的下一行写你的答案。推荐答案就是问题里以 ➡️ 开头的那行,")
    print("      你同意就抄过去, 不同意就写你自己的。")
    print()
    print("方式二 (命令行):")
    print(f"      cd {f.dir}")
    print("      uv run %s <序号> <你的答案>" % (Path(__file__).resolve().parent / 'hx_mitemite_res.py'))
    print()
    print("答完告诉执行者继续。未答的题会让流程停在阶段 4, 不会再被自动跳过。")
    return 0


def card_seq(it: dict) -> str:
    return it["seq"] + ("=[已答]" if it["answered"] else "=[未答]")


# --- doctor ---------------------------------------------------------------
def _run(cmd: list[str], cwd: Path) -> tuple[bool, str]:
    try:
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=180)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"无法执行: {exc}"
    out = (r.stdout + r.stderr).strip()
    return r.returncode == 0, out[-600:]


# 绝对式引用: ".agents/skills/<skill>/..." —— 本意就是指向仓库里的一个真实文件。
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

# 反引号围起来的行内片段。**只豁免其中含 markdown 链接语法的那些** ——
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

    # --- 可复用物: 笔记里必须有一个"能拿到手"的东西 ---
    #
    # 存在的理由 (实测): 只靠文字要求, 产出会退化成纯散文 —— 读者读完懂了原理,
    # 却拿不到任何能照做的东西, 这篇笔记的参考价值就是零。所以把它变成闸门。
    # 判据是"至少有一件", 不是"必须有多少件": 讲解型笔记本来就该短。
    if index.is_file():
        txt = index.read_text(encoding="utf-8")
        # 侧车: 与 index.md 同目录, 且**真被正文引用**的文件。
        # 只放在目录里不算 —— 实测踩过: 一个无人引用的 .html 侧车会被构建插件
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
    # 而写死在文档里的路径不会跟着动。实测踩过 —— 一次批量改名后, 18 处路径全部失效,
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


# 发散记录里"素材没有"的标记。产物必须带的字面标记, 用来区分"复述"与"发散"。
DIVERGENT_MARK = "[素材没有]"
# 占位符: 模板留着没填的空格会被判为无效判据。
# 单个格子判, **不拼接后再判** —— 拼接会让 "<...>" 与 "..." 两个占位符合成
# "<...> ..." 而整体不再是纯占位符, 于是漏过 (实测踩过)。
PLACEHOLDER_RE = re.compile(r"^[<.…\-—\s]*$|^<[^>]*>$")
# 感想词: 出现这些且没有任何可查动作 (= 没有"查/跑/测/grep/看"这类动词) 时不算判据
VAGUE_RE = re.compile(r"^(?![^|]*(?:查|跑|测|试|grep|http|npm|node|python|文件|目录|命令|退出码))"
                      r"[^|]*(?:用心|极致|值得|体现|一致|优雅|追求|重视|厉害|讲究)[^|]*$")


def divergence_report(path: Path) -> tuple[int, str]:
    """数出发散记录里"带判据的产出"条数, 返回 (条数, 问题说明)。

    判据是机械的: 每一行"素材没有"的产出行, 必须同时给出去向/判据那一格的非占位内容。
    为什么卡这个: 不卡的话, 模型会用 4 行"这套设计体现了对细节的追求"把配额填满 ——
    那正是这一步要消灭的东西 (不可验证的感想)。判据格非空 = 它至少给了一个可查的动作。
    """
    if not path.is_file():
        return 0, "文件不存在"
    text = path.read_text(encoding="utf-8")
    good = 0
    bad_rows: list[str] = []
    for line in text.splitlines():
        if DIVERGENT_MARK not in line or not line.lstrip().startswith("|"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        # 表格形如 | # | 结论 | 判据 | 去向 |, 去掉分隔行
        if len(cells) < 4 or set("".join(cells)) <= set("-: "):
            continue
        # 判据格: 从第 3 格起, 逐格看有没有一格是"真的非占位内容"
        judged = [c for c in cells[2:]
                  if c and not PLACEHOLDER_RE.match(c) and not VAGUE_RE.match(c)
                  and len(re.sub(r"\W", "", c)) >= 4]
        if judged:
            good += 1
        else:
            bad_rows.append(line.strip()[:60])
    detail = ""
    if bad_rows:
        detail = "以下产出行没有可查的判据 (占位符或空): " + "; ".join(bad_rows[:3])
    return good, detail


def _orphan_sidecars(target: Path) -> list[str]:
    """找出与 index.md 同目录、但正文里一次也没引用的侧车。

    这类文件不会被任何读者看到, 却会被站点插件拷进产物 (实测踩过: 一个 8.6KB 的
    .html 侧车在磁盘上、被 build 发布、而 grep 全仓零引用)。
    """
    if not target.is_dir():
        return []
    index = target / "index.md"
    text = index.read_text(encoding="utf-8") if index.is_file() else ""
    out: list[str] = []
    for p in sorted(target.iterdir()):
        if not p.is_file() or p.name.startswith("."):
            continue
        if p.suffix.lower() not in (".html", ".tsx", ".svg", ".png", ".jpg", ".jpeg", ".webp"):
            continue
        if p.name not in text:
            out.append(p.name)
    return out


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
                   choices=["video", "article", "custom", "local", "research", "repo"])
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

    cv = sub.add_parser("cover", help="需求覆盖: 每条验收条款被多少条知识点支撑")
    cv.add_argument("--slug", required=True)
    cv.set_defaults(func=cmd_cover)

    bf = sub.add_parser("brief", help="把待审的题、被审对象、作答方法一次打全 (给人看的)")
    bf.add_argument("--slug", required=True)
    bf.set_defaults(func=cmd_brief)

    dc = sub.add_parser("doctor", help="阶段 9: 跑全部交付闸门")
    dc.add_argument("--slug", required=True)
    dc.set_defaults(func=cmd_doctor)

    ls = sub.add_parser("list", help="列出暂存区里所有流程")
    ls.set_defaults(func=cmd_list)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())