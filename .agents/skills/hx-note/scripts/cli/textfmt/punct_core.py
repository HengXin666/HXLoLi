"""中文笔记标点归一化: 全角 -> 英文标点, 并在中文后补一个空格。

转换: ，。：；？！（） -> , . : ; ? ! ( )
保留: 、 顿号, 《》「」“” 引号/书名号, — 破折号 (这些在中文正文中按需使用).
保护: frontmatter、围栏代码块、行内代码、URL 原样保留.

示例:
    你好，世界。 -> 你好, 世界.
    红、绿、蓝 -> 红、绿、蓝
    《标题：副题》 -> 《标题: 副题》
"""
from __future__ import annotations


# Agent Notes: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织; 沉淀必须产出可复用物
# 
# 
def _cjk_set() -> set[str]:
    # 用运行时 chr 构造 CJK 码位集合, 避免源码内联转义.
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
    """
    chars: set[str] = set()
    for lo, hi in ((0x4E00, 0x9FFF), (0x3400, 0x4DBF), (0xF900, 0xFAFF)):
        chars.update(chr(c) for c in range(lo, hi + 1))
    return chars

_CJK: set[str] = _cjk_set()

_FW2HW: dict[str, str] = {
    chr(0xFF0C): chr(0x2C),   # ， -> ,
    chr(0x3002): chr(0x2E),   # 。 -> .
    chr(0xFF1A): chr(0x3A),   # ： -> :
    chr(0xFF1B): chr(0x3B),   # ； -> ;
    chr(0xFF1F): chr(0x3F),   # ？ -> ?
    chr(0xFF01): chr(0x21),   # ！ -> !
    chr(0xFF08): chr(0x28),   # （ -> (
    chr(0xFF09): chr(0x29),   # ） -> )
}

_NEED_SPACE_AFTER: set[str] = {',', '.', ':', ';', '!', '?'}

_CJK_PUNCT: set[str] = {
    chr(0x300A), chr(0x300B),  # 《 》
    chr(0x300C), chr(0x300D),  # 「 」
    chr(0x300E), chr(0x300F),  # 『 』
    chr(0x2018), chr(0x2019),  # ‘ ’
    chr(0x201C), chr(0x201D),  # “ ”
}

def _is_cjk(ch: str) -> bool:
    return ch in _CJK or ch in _CJK_PUNCT

def _fence_can_close(text: str, i: int, mlen: int, n: int) -> bool:
    # 标记后仅剩空白到行尾/EOF 才算真正关闭
    j = i + mlen
    while j < n and text[j] in ' \t':
        j += 1
    return j >= n or text[j] == chr(10)

def normalize(text: str) -> str:
    """扫描文本, 保护 frontmatter/代码块/行内代码后做标点转换."""
    out: list[str] = []
    n = len(text)
    i = 0
    # frontmatter 保护: 开头 --- 到下一个 --- 行
    if text.startswith('---'):
        nl = text.find(chr(10), 3)
        end = text.find(chr(10) + '---', nl + 1) if nl != -1 else -1
        if end != -1:
            out.append(text[: end + 4])
            i = end + 4
    # fence 与行内码保护: 用状态机逐字符扫
    fence = False
    inline = False
    tok: list[str] = []
    fence_marker = ''

    def flush_tok() -> None:
        # 保护片段原样入 out
        out.append(''.join(tok))
        tok.clear()

    while i < n:
        ch = text[i]
        if fence:
            if text.startswith(fence_marker, i) and _fence_can_close(text, i, len(fence_marker), n):
                # 关闭: 整行(含标记)进 tok, 原样保留后退出围栏态
                k = i
                while k < n and text[k] != chr(10):
                    tok.append(text[k])
                    k += 1
                if k < n:
                    tok.append(chr(10))
                    k += 1
                flush_tok()
                fence = False
                fence_marker = ''
                i = k
                continue
            tok.append(ch)
            i += 1
            continue
        if inline:
            tok.append(ch)
            if ch == '`':
                flush_tok()
                inline = False
            i += 1
            continue
        # 非保护态
        if ch == '`':
            # 判断是否围栏(连续3个及以上)还是行内
            cnt = 0
            while i + cnt < n and text[i + cnt] == '`':
                cnt += 1
            if cnt >= 3:
                fence = True
                fence_marker = '`' * cnt
                for _ in range(cnt):
                    tok.append('`')
                i += cnt
                # 若同行还有语言标注, 一并保护直到行尾
                while i < n and text[i] != chr(10):
                    tok.append(text[i])
                    i += 1
                if i < n:
                    tok.append(chr(10))
                    i += 1
                continue
            inline = True
            tok.append(ch)
            i += 1
            continue
        # 普通字符: 可能转换
        repl = _FW2HW.get(ch)
        if repl is None:
            out.append(ch)
            i += 1
            continue
        prev = text[i - 1] if i > 0 else ''
        nxt = text[i + 1] if i + 1 < n else ''
        if repl in _NEED_SPACE_AFTER:
            # 中文语境下英文标点后补一个空格 (除非已经是空格/换行/行尾)
            if nxt and not nxt.isspace() and not prev.isspace():
                out.append(repl + ' ')
            else:
                out.append(repl)
        else:
            out.append(repl)
        i += 1
    if tok:
        out.append(''.join(tok))
    return ''.join(out)
