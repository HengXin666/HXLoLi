"""流水线共享层: 常量、阶段表、Flow 对象、发散与覆盖报告。

入口在 flow/hx_flow.py; 多个子命令读同一份阶段表, 所以单独成模块。
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

# 跨脚本路径的唯一定义点: 写成 Path(__file__).parent 拼兄弟脚本名会随搬目录静默失效
# (see )
# Agent Notes: hx-note 的 scripts 收敛到自己的体量与文本门禁
# 
# Agent Notes: hx-note 的 scripts 收敛到自己的体量与文本门禁
# .agents/notes/implemented/process/2026-10-05-hx-note-scripts-converge-to-own-gates.md
SCRIPTS_DIR = Path(__file__).resolve().parents[2]  # scripts/
CLI_DIR = SCRIPTS_DIR / "cli"
MAKEDOC_CLI = CLI_DIR / "authoring" / "makeDoc.py"
MITEMITE_RES_CLI = CLI_DIR / "mitemite" / "hx_mitemite_res.py"
PUNCT_CLI = CLI_DIR / "textfmt" / "format_cn_punct.py"
TAGS_CLI = CLI_DIR / "taxonomy" / "hxloli_tags.py"
VOICE_CLI = CLI_DIR / "voice" / "hx_voice.py"


STAGING = "ai-docs/.hx-staging"
GOAL = "source/goal.md"
FLOW = "flow.json"
# 这些文件属于笔记, place 之后住在正式目录; 其余属于过程, 永远留在暂存区。
# Agent Notes: 沉淀流水线改为"两份产物 + 九个单一职责阶段"
# .agents/notes/implemented/process/2026-09-24-sediment-two-artifacts-nine-stages.md
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


# 发散记录里"素材没有"的标记。产物必须带的字面标记, 用来区分"复述"与"发散"。
DIVERGENT_MARK = "[素材没有]"
# 占位符: 模板留着没填的空格会被判为无效判据。
# 单个格子判, **不拼接后再判**  拼接会让 "<...>" 与 "..." 两个占位符合成
# "<...> ..." 而整体不再是纯占位符, 于是漏过 (实测踩过)。
PLACEHOLDER_RE = re.compile(r"^[<.…\-—\s]*$|^<[^>]*>$")
# 感想词: 出现这些且没有任何可查动作 (= 没有"查/跑/测/grep/看"这类动词) 时不算判据
VAGUE_RE = re.compile(r"^(?![^|]*(?:查|跑|测|试|grep|http|npm|node|python|文件|目录|命令|退出码))"
                      r"[^|]*(?:用心|极致|值得|体现|一致|优雅|追求|重视|厉害|讲究)[^|]*$")


def divergence_report(path: Path) -> tuple[int, str]:
    """数出发散记录里"带判据的产出"条数, 返回 (条数, 问题说明)。

    判据是机械的: 每一行"素材没有"的产出行, 必须同时给出去向/判据那一格的非占位内容。
    为什么卡这个: 不卡的话, 模型会用 4 行"这套设计体现了对细节的追求"把配额填满 
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
