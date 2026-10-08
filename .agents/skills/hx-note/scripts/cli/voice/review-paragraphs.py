#!/usr/bin/env python3
"""note-triage 之外的第二个 jev 用途: **逐段**审 AI 味。

与 hx_voice.py lint 的分工:
  lint        -> 机械判据 (词表/句长/加粗), 全文一扫, 可复现, 零成本
  jev 逐段审  -> 机械判据抓不到的那种「格言化」, 但**只在它报警时才可信**

## 实测 (2026-09-26, 32 段对照)

问法: 「这段话像一句格言或箴言, 抽掉语境后依然成立吗?」

| | 手写 blog | ai-docs (AI 生成) |
|---|---|---|
| 中位 | **0.11** | **0.34** |
| 极差 | 0.08 - 0.23 | 0.12 - 0.71 |
| >0.4 占比 | **0%** | 38% |

**结论: 高精度、低召回。**
  · 它报 >0.4 时, 大概率真有问题  手写 16 段从未超过 0.23, 下界是干净的
  · 但它不报不代表没问题  62% 的 AI 段落它也放过

## 所以怎么用

**把它当「挑出该看哪几段」的筛子, 不当判据。**
拿到段落列表后逐条人读, 决定改不改。它的价值是把 217 段缩到十几段, 让人有地方下眼。

反例 (踩过): 早先试过直接问「这段像 AI 还是像人」, 手写 0.43-0.57 / 机器 0.47-0.54,
**完全重叠, 零信息**。换问法才有信号  这是第二个「换 key 无用、换问法有用」的实例。

用法:
    uv run python review-paragraphs.py <md 路径> [--threshold 0.4] [--json]
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path

QUESTION = '这段话像一句格言或箴言, 抽掉语境后依然成立吗?'
DEFAULT_THRESHOLD = 0.4


def env_get(key: str) -> str | None:
    return os.environ.get(key)


def paragraphs(md: Path) -> list[tuple[int, str]]:
    """取出待审段落 (行号, 文本)。跳过 frontmatter / 代码块 / 表格 / 标题。"""
    s = md.read_text(encoding='utf-8')
    s = re.sub(r'^---.*?^---', '', s, flags=re.S | re.M)
    lines = s.splitlines()
    out: list[tuple[int, str]] = []
    in_fence = False
    for i, raw in enumerate(lines, 1):
        t = raw.strip()
        if t.startswith('```'):
            in_fence = not in_fence
            continue
        if in_fence or not t or t.startswith(('#', '>', '|', '!')):
            continue
        if len(t) > 20:
            out.append((i, t))
    return out


def find_client() -> Path | None:
    for cand in (env_get('HX_JEV_CLIENT_DIR'),):
        if cand and (Path(cand).expanduser() / 'typesafe_client.py').is_file():
            return Path(cand).expanduser()
    return None


def find_accounts() -> Path | None:
    cand = env_get('HX_JEV_ACCOUNTS')
    if cand and Path(cand).expanduser().is_file():
        return Path(cand).expanduser()
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('path')
    ap.add_argument('--threshold', type=float, default=DEFAULT_THRESHOLD)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    md = Path(a.path)
    if not md.is_file():
        print('error: 文件不存在: ' + str(md), file=sys.stderr)
        return 2
    paras = paragraphs(md)
    if not paras:
        print('没有可审段落')
        return 0

    client = find_client()
    accounts = find_accounts()
    if client is None or accounts is None:
        print('review-paragraphs: 缺 jev 依赖 (.env 未配), 跳过 (辅助工具, 不阻塞)')
        return 0
    sys.path.insert(0, str(client))
    try:
        rows = json.loads(accounts.read_text(encoding='utf-8'))
        key = next(r['api_key'] for r in rows if r.get('usable') and r.get('api_key'))
        from typesafe_client import TypeSafe  # type: ignore
        ts = TypeSafe(key)
    except Exception as error:
        print('review-paragraphs: 初始化失败, 跳过 (' + str(error)[:50] + ')')
        return 0

    scored: list[tuple[int, str, float]] = []
    for lineno, text in paras:
        try:
            p, _ = ts.yes_no(text[:400], QUESTION)
            scored.append((lineno, text, p))
        except Exception:
            continue

    flagged = [x for x in scored if x[2] > a.threshold]
    flagged.sort(key=lambda x: -x[2])

    if a.json:
        print(json.dumps({'total': len(scored), 'flagged': len(flagged),
                          'items': [{'line': l, 'p': round(p, 3), 'text': t} for l, t, p in flagged]},
                         ensure_ascii=False, indent=1))
    else:
        print('# 逐段测格言化: %d 段里有 %d 段超 %.1f' % (len(scored), len(flagged), a.threshold))
        print('# 这是筛子不是判据  只在它报警时才可信 (手写从未超过 0.23); 不报不代表没问题')
        print()
        for lineno, text, p in flagged:
            print('  %.2f  行 %-4d %s' % (p, lineno, text[:62]))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())