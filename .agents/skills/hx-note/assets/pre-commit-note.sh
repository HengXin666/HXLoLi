#!/bin/sh
# Normalize article text, check its prose, then report the exact staged v2 graph.
# This human commit hook reports findings and always allows the commit.

set -u

ROOT=$(git rev-parse --show-toplevel 2>/dev/null) || exit 0
cd "$ROOT" || exit 0

# 站点根可能在两个地方 (实测踩过):
#   · HXLoLi 自己就是 git 仓库 -> ROOT 就是站点根
#   · 在 HXLoLis 总库下 -> ROOT 是总库, 站点在 ROOT/HXLoLi
if [ -d "$ROOT/ai-docs" ]; then
    SITE="$ROOT"
elif [ -d "$ROOT/HXLoLi/ai-docs" ]; then
    SITE="$ROOT/HXLoLi"
else
    exit 0
fi
cd "$SITE" || exit 0

# 路径可能带 HXLoLi/ 前缀 (在总库提交时), 统一去掉
strip() { echo "$1" | sed 's|^HXLoLi/||'; }

# Only staged Markdown enters text processing; the v2 scan also runs without Markdown changes.
# 另外: `git add .` 会 stage 全库, 所以这里对**未改动的文件**也要跳过 
# 否则每次提交都把全库 lint 一遍 (实测 3.4s, 而且会报一堆历史遗留)。
# **必须 -z**: 默认输出会把中文路径转成八进制转义并加引号 (core.quotepath),
# 于是 'grep \.md$' 匹配不到  实测踩过, 表现为钩子"静默不工作"。
CHANGED=$(git -c core.quotepath=false diff --cached --name-only --diff-filter=ACMR -z \
          | tr '\0' '\n' | grep '\.md$' || true)

# Keep Markdown paths whose staged contents differ from HEAD.
REAL=""
for f in $CHANGED; do
    if ! git diff --cached --quiet -- "$f" 2>/dev/null; then
        REAL="$REAL $f"
    fi
done
CHANGED=$REAL

PY=".agents/skills/hx-note/scripts/cli"
fail=0

# ---- 第 1 段: 自动修 ----
# 标点归一化把全角句号转成半角, 这是站内规范, 没有争议 -> 直接修
FIXED=""
for f in $CHANGED; do
    case "$f" in
        HXLoLi/ai-docs/*|ai-docs/*) ;;
        *) continue ;;
    esac
    # 路径可能是 HXLoLi/ 前缀或裸 ai-docs/, 统一成相对 SITE
    rel=$(echo "$f" | sed 's|^HXLoLi/||')
    [ -f "$rel" ] || continue
    if ! uv run "$PY/textfmt/format_cn_punct.py" --check "$rel" >/dev/null 2>&1; then
        uv run "$PY/textfmt/format_cn_punct.py" "$rel" >/dev/null 2>&1 && {
            git add "$f" 2>/dev/null || true
            FIXED="$FIXED $rel"
        }
    fi
    # 折行合并 (fix 会跳过 frontmatter 与代码块, 见 hx_voice.py 的注释)
    uv run "$PY/voice/hx_voice.py" fix "$rel" --write >/dev/null 2>&1 && {
        git add "$f" 2>/dev/null || true
    }
done
[ -n "$FIXED" ] && echo "[note-gate] 已自动修标点:$FIXED"

# ---- 第 2 段: 只读校验 ----
# 2a. 语料红线 (只对 article/atom, 且只报 E 级  W 是提示不是错)
for f in $CHANGED; do
    rel=$(echo "$f" | sed 's|^HXLoLi/||')
    case "$rel" in
        ai-docs/*.md|ai-docs/*/*.md|ai-docs/*/*/*.md|ai-docs/*/*/*/*.md) ;;
        *) continue ;;
    esac
    [ -f "$rel" ] || continue
    # 过程产物不查: 答题卡是给人类看的审核面板, 暂存区是中间产物。
    # **实测踩过**: 不排除时 .hx-mitemite.md 会被按 article 标准报一堆 E, 纯噪音。
    #
    # **必须先给 prof 一个默认值**: 空值时 hx_voice.py 会按文件名猜 profile,
    # 于是 index.md 被当成 atom 查, 报出一堆根本不是问题的 atom 规则。
    # 实测症状: 单独跑 PASS, 钩子里却报 atom-image / atom-hedge。
    prof=article
    case "$rel" in
        *.hx-info.md) prof=atom ;;
        *.hx-mitemite.md) continue ;;
        .hx-staging/*|*/.hx-staging/*) continue ;;
        */.*) continue ;;
    esac
    if ! uv run "$PY/voice/hx_voice.py" lint "$rel" --profile "$prof" >/tmp/hx_lint.txt 2>&1; then
        echo "[note-gate] 语料检查未过: $rel"
        grep -E '^  E' /tmp/hx_lint.txt | head -6
        fail=1
    fi
done

# Check v2 once, including commits that contain only code changes.
GATE_LOG=$(mktemp "${TMPDIR:-/tmp}/hx-agent-notes-pre-commit.XXXXXX") || {
    echo "[note-gate] 无法创建诊断文件, 本次双链未经验证"
    exit 0
}
GATE_REPORT="$GATE_LOG.json"
if [ -f scripts/redlines/agent_notes.py ]; then
    uv run scripts/redlines/agent_notes.py --all --staged --json "$GATE_REPORT" >"$GATE_LOG" 2>&1
else
    uv run --with-requirements .agents/skills/hx-agent-notes/scripts/redline/requirements.txt \
        python .agents/skills/hx-agent-notes/scripts/redline/verify.py \
        --all --staged --json "$GATE_REPORT" >"$GATE_LOG" 2>&1
fi
GATE_STATUS=$?

if [ -f "$GATE_REPORT" ] && python3 - "$GATE_REPORT" <<'PY'
import json
import sys

with open(sys.argv[1], encoding='utf-8') as report:
    data = json.load(report)
issues = data['issues']
errors = [i for i in issues if i['severity'] != 'review']
reviews = [i for i in issues if i['severity'] == 'review']
print(f"[note-gate] v2 暂存区: {len(errors)} 项错误, {len(reviews)} 项待审核")
for label, findings, limit in [('ERROR', errors, 6), ('REVIEW', reviews, 3)]:
    for issue in findings[:limit]:
        print(f"  {label} {issue['path']}:{issue['line']} [{issue['rule']}] {issue['message']}")
    if len(findings) > limit:
        print(f"  {label} 另有 {len(findings) - limit} 项, 见完整诊断")
if issues:
    print(f"[note-gate] 完整诊断: {sys.argv[1]}")
PY
then
    :
else
    echo "[note-gate] v2 扫描或报告读取失败, 本次双链未经验证"
    tail -8 "$GATE_LOG"
    echo "[note-gate] 工具日志: $GATE_LOG"
    fail=1
fi
if [ "$GATE_STATUS" != "0" ]; then
    fail=1
elif [ "$fail" = "0" ]; then
    rm -f "$GATE_LOG" "$GATE_REPORT"
fi

if [ "$fail" != "0" ]; then
    echo "[note-gate] 提示模式, 提交继续; 有待审核项或错误, 不代表双链通过"
fi
exit 0
