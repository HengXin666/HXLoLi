#!/bin/sh
# HXLoLi 笔记门禁 (pre-commit): 三段式  自动修 / 只读校验 / 报了就给出口。
# 立场是替人把该做的做掉而不是拦住提交 (BYPASS 提示只在真失败时打印)。
# 出口: HX_SKIP=1 git commit ... 或 git commit --no-verify ...

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

if [ "${HX_SKIP:-0}" = "1" ]; then
    echo "[note-gate] HX_SKIP=1, 跳过"
    exit 0
fi

# 只看这次真的改到的 md, 没改就不跑 (省时间, 也避免无关失败)。
# 另外: `git add .` 会 stage 全库, 所以这里对**未改动的文件**也要跳过 
# 否则每次提交都把全库 lint 一遍 (实测 3.4s, 而且会报一堆历史遗留)。
# **必须 -z**: 默认输出会把中文路径转成八进制转义并加引号 (core.quotepath),
# 于是 'grep \.md$' 匹配不到  实测踩过, 表现为钩子"静默不工作"。
CHANGED=$(git -c core.quotepath=false diff --cached --name-only --diff-filter=ACMR -z \
          | tr '\0' '\n' | grep '\.md$' || true)
[ -z "$CHANGED" ] && exit 0

# 只保留**内容真的变了**的 (git add . 会把全库塞进来)  用 diff 的 -M 与 --diff-filter 都挡不住,
# 得逐个问 git: 这个路径在暂存区与 HEAD 之间有没有内容差异。
REAL=""
for f in $CHANGED; do
    if ! git diff --cached --quiet -- "$f" 2>/dev/null; then
        REAL="$REAL $f"
    fi
done
CHANGED=$REAL
[ -z "$CHANGED" ] && exit 0

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

# 2b. notes 门禁 (快, ~260ms)
for s in verify-format verify-backlinks verify-tree; do
    if ! node ".agents/skills/hx-agent-notes/scripts/cli/$s.ts" >/tmp/hx_gate.txt 2>&1; then
        echo "[note-gate] $s 未过"
        grep -v 'Reparsing\|Warning\|eliminate\|trace\|MODULE_TYPELESS' /tmp/hx_gate.txt | tail -4
        fail=1
    fi
done
if ! node .agents/skills/hx-agent-notes/scripts/cli/verify-coverage.ts --staged >/tmp/hx_cov.txt 2>&1; then
    echo "[note-gate] verify-coverage 未过"
    grep -v 'Reparsing\|Warning\|eliminate\|trace\|MODULE_TYPELESS' /tmp/hx_cov.txt | tail -4
    fail=1
fi

# ---- 第 3 段: 报了就给出路 ----
if [ "$fail" != "0" ]; then
    echo ""
    echo "  以上是提示, 不是死路。两条出口:"
    echo "    HX_SKIP=1 git commit ...      只跳过这个笔记门禁"
    echo "    git commit --no-verify ...    跳过全部钩子"
    echo ""
    # 默认放行: 立场是「不挡人类」. 想改成硬拦, 把下面两行换成 exit 1
    echo "  [note-gate] 默认放行 (设计前提: 不阻挡提交)"
    exit 0
fi
exit 0
