"""阶段 5 落地与给人看的两个读数: place / cover / brief。"""
from __future__ import annotations

import re
import subprocess
from pathlib import Path

from flow.flow_core import GOAL, MAKEDOC_CLI, MITEMITE_RES_CLI, Flow, repo_root
from flow.flow_read import card_seq, print_status, read_answer_card, read_goal


def cmd_place(args) -> int:
    """阶段 5: 唯一允许创建正式目录的地方。"""
    root = repo_root(Path(args.root) if args.root else None)
    f = Flow(root, args.slug).load()
    if "align" not in f.done_set() and not args.force:
        raise SystemExit("error: 阶段 4 align 还没完成  路径要在人类确认 .hx-info.md 之后才定")

    # align 若被 --force 标记完成 (人类未作答), 这里再拦一次。
    # 两处都拦的理由: done 是"AI 自己宣布过了", place 是"真的开始动正式目录" 
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
        make_doc = MAKEDOC_CLI
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
        print('goal.md 里没有验收条款  先跑阶段 1。见 assets/goal-template.md')
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
    print("      uv run %s <序号> <你的答案>" % MITEMITE_RES_CLI)
    print()
    print("答完告诉执行者继续。未答的题会让流程停在阶段 4, 不会再被自动跳过。")
    return 0
