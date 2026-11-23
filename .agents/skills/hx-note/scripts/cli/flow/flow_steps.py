"""推进阶段的子命令: init (开流程) 与 done (标记完成并过闸门)。"""
from __future__ import annotations

import re
from datetime import date
from pathlib import Path

from flow.flow_core import (BY_KEY, BY_NUM, DIVERGENT_MARK, GOAL, STAGES, Flow,
                           divergence_report, repo_root, slugify)
from flow.flow_read import goal_gate, print_status, read_answer_card
from lib.sources_core import KINDS, breadth_report



PERSONA = "ai-docs/.hx-persona.md"
NO_LINK = "画像无关联:"


def persona_anchors(text: str) -> set[str]:
    """Agent Notes
    .agents/notes/implemented/architecture/2026-11-08-migration-anchored-to-persona.md
    """
    names = set(re.findall(r"^- \*\*([^*]{2,40})\*\*", text, re.M))
    names |= {t.split("  tags:")[0].strip()
              for t in re.findall(r"^- `\d{4}-\d\d-\d\d` ([^\[\n]{4,60})", text, re.M)}
    for tags in re.findall(r"^- 高频 tag: (.+)$", text, re.M):
        names |= {re.sub(r"×\d+$", "", t).strip() for t in tags.split(",")}
    return {n for n in names if len(n) >= 2}


def migration_gate(div: Path, persona: Path) -> str:
    """算子三至少一行落在画像条目上, 或明写 `画像无关联: 原因`. 空串表示通过"""
    text = div.read_text(encoding="utf-8")
    m = re.search(r"^## 算子三.*?(?=^## |\Z)", text, re.M | re.S)
    sec = m.group(0) if m else ""
    hit = re.search(re.escape(NO_LINK) + r"\s*(\S.{6,})", sec)
    if hit:
        return ""
    if not persona.is_file():
        return ("缺作者画像 " + PERSONA + "; 先跑 hx_persona.py build, "
                "再把算子三的迁移目标落到画像里的具体项目或方向上")
    names = persona_anchors(persona.read_text(encoding="utf-8"))
    if any(n in sec for n in names):
        return ""
    return ("算子三没有一行迁移到画像里的条目. 迁移目标写成画像 0x01/0x02 里的具体名字, "
            "并给检查动作; 确实无关就写一行 `" + NO_LINK + " <为什么>`")


def storyboard_gate(path: Path) -> str:
    """分镜每行 `- 0xNN | 标题 | 要证明的一句话 | 图 | 出处`; 只要求那句话非空。空串表示通过。
    图不再必填 (2026-11-08 撤掉「每段主视觉」, 见 note rules-feed-forward-gates-confirm)。
    旧格式 `| 0xNN | ... |` 仍认, 那时第 4 格是要证明的话。"""
    if not path.is_file():
        return "缺 source/storyboard.md  步骤 6 先出分镜再写正文 (assets/storyboard-template.md)"
    rows = []
    for ln in path.read_text(encoding="utf-8").splitlines():
        t = ln.strip()
        if t.startswith("- 0x"):
            c = [x.strip() for x in t[2:].split("|")]
            rows.append((c[0], c[2] if len(c) > 2 else ""))
        elif t.startswith("| 0x"):
            c = [x.strip() for x in t.strip("|").split("|")]
            rows.append((c[0], c[3] if len(c) > 3 else ""))
    if len(rows) < 3:
        return "分镜只有 %d 段, 要求 3 到 6 段" % len(rows)
    bad = [k for k, claim in rows if not claim or claim.startswith("<")]
    return ("这些段没写要证明的那句话: " + ", ".join(bad)) if bad else ""


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
        # 新流程必须带盲审判定头; 没有这个键的是旧流程, 缺首行只报告不拦。
        "verdict_required": True,
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
    # Stage(1).artifacts 原先是**空的**  实测结果是需求写不写流程完全不管,
    # 于是同一个任务里出现了 4 份各不相同的"用户要什么", 而没人回对过最初那句话。
    if key == "intake" and not args.force:
        why = goal_gate(f.locate(GOAL))
        if why:
            raise SystemExit("error: " + why
                             + "\n  (素材确实没有可判定的目标时加 --force 并写明原因)")

    # 阶段 4 的硬闸门: 人类必须真的答过。
    # 只要求"文件存在"是不够的  实测踩过: 把推荐答案写进 Q、A 留空、
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

    if key == "collect" and not args.force and f.data.get("kind") in KINDS:
        br = breadth_report(f.dir / "source")
        if br.problems:
            raise SystemExit("error: 来源广度不达标, 拒绝推进:\n  " + "\n  ".join(br.problems)
                             + "\n  等级与格式见 steps/2-collect/impl/research.md")

    if key == "derive" and not args.force:
        why = storyboard_gate(f.locate("source/storyboard.md"))
        if why:
            raise SystemExit("error: " + why)

    if key == "atom" and not args.force:
        div = f.locate("source/divergence.md")
        n, detail = divergence_report(div)
        if not div.is_file():
            raise SystemExit(
                "error: 缺 source/divergence.md  阶段 3 必须先跑发散算子再写 .hx-info.md。\n"
                "  做法: steps/3-atom/impl/divergence.md (四个算子); 格式: assets/divergence-template.md\n"
                "  素材是纯资讯类、确实无可发散结构时加 --force 并在 --note 里写明原因")
        if n < 4:
            raise SystemExit(
                "error: source/divergence.md 里只有 %d 条带判据的产出, 要求 >= 4。\n"
                "  %s\n"
                "  每条的'结论'格必须以 %s 开头, 并在'判据/去向'格给出可查的动作。"
                % (n, detail or "先把四个算子各跑一次", DIVERGENT_MARK)
                + "\n  确实无可发散结构时加 --force 并在 --note 里写明原因")
        why = migration_gate(div, root / PERSONA)
        if why:
            raise SystemExit("error: " + why + "\n  做法: steps/3-atom/impl/divergence.md 算子三")

    f.data.setdefault("done", [])
    if key not in f.data["done"]:
        f.data["done"].append(key)
    if args.note:
        f.data.setdefault("notes", {})[key] = args.note
    if miss and args.force:
        f.data.setdefault("notes", {})[key + ":skipped"] = f"缺 {miss}; {args.note or '未说明原因'}"
    # 记下这次 align 是不是在"人类未答"的情况下靠 --force 过的。
    # place 靠这个标记决定要不要再拦一次  只看 done_set 是不够的,
    # 因为 --force 也会把 align 写进 done_set。
    if key == "align" and args.force:
        pending = [it for it in read_answer_card(f.locate(".hx-mitemite.md"))
                   if not it["answered"]]
        f.data["align_forced"] = True
        if pending:
            print(f"warn: 阶段 4 带 --force 通过, 但有 {len(pending)} 题未答  place 时会再拦一次")
    f.save()
    print(f"done: {stage.num} {key}")
    print_status(f)
    return 0


def cmd_sources(args) -> int:
    """单独跑来源广度检查, 不推进阶段。"""
    root = repo_root(Path(args.root) if args.root else None)
    f = Flow(root, args.slug).load()
    b = breadth_report(f.dir / "source")
    print("来源 %d 个 (T1 %d 个), 要点 %d 条, 反例 %d 条" % (len(b.tiers), b.t1, b.points, b.counter))
    for line in b.problems:
        print("  - " + line)
    print("PASS" if not b.problems else "FAIL")
    return 0 if not b.problems else 1
