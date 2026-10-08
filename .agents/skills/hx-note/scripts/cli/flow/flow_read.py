"""读数与判定: 状态展示、暂存区列表、答题卡解析、目标契约闸门。"""
from __future__ import annotations

import json
import re
from pathlib import Path

from flow.flow_core import FLOW, GOAL, STAGING, STAGES, Flow, repo_root


# 语料强制注入: 挂在 status 的输出里, 因为它是每一步都必须调的入口,
# 于是语料每一轮都在场  不依赖模型自觉去读。
# (see )
# Agent Notes: 表达语料强制注入 status 输出, 不依赖模型自觉去读
# 
def voice_corpus_lines(root: Path) -> list[str]:
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-26-voice-corpus-forced-injection.md
    """
    db = root / 'ai-docs' / '.hx-voice.toml'
    if not db.is_file():
        return []
    try:
        raw = db.read_text(encoding='utf-8')
    except OSError:
        return []
    # 按行解析, 不上正则  这种小格式用逐行判断更清楚, 也不会在转义上翻车。
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
    # 语料每次都在场。放在最开头是刻意的  后面的分支会提前 return,
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


# --- 答题卡: 读数与待答判定 ------------------------------------------------
# 补两样: 把问题端到眼前的读数 (brief), 与"未答不许推进"的闸门 (done/place)。
# 块头格式必须宽容  卡是给人改的, 卡死格式会把"未答"误判成"没有问题"。
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
        # **必须剥掉收尾的 `}`**  实测踩过: 不剥时惰性匹配会把 `}` 当成答案内容,
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
# 全部手工维护、无一致性检查, 而 Stage(1).artifacts 当时是**空的**  需求写不写
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

    验收栏的机械要求是出现可执行动作  写不出检查动作的条款是愿望, 不是条款。
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
        return ('缺 %s  阶段 1 必须先把用户要什么落成可判定的条款。'
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


def card_seq(it: dict) -> str:
    return it["seq"] + ("=[已答]" if it["answered"] else "=[未答]")


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
