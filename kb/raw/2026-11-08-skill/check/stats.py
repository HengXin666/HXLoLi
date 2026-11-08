#!/usr/bin/env python3
"""统计 index.md: 字数 / 纯文字行 / 每段视觉 / 最长连续纯文字 / 最大表格数据行.
纯文字行口径与 hx_voice.py 的 _tally 一致 (非空、非围栏内、非表格行、非 # 标题、非图片);
区别: 这里按任意 `## ` 切段 (hx_voice 只认 `## 0xNN`, 本次标题不用 0xNN, 故自己数).
"""
import re
import sys


def main(path):
    text = open(path, encoding="utf-8").read()
    body = re.sub(r"\A---\n.*?\n---\n", "", text, flags=re.S)
    plain = re.sub(r"```.*?```", "", body, flags=re.S)
    cjk = len(re.findall(r"[\u4e00-\u9fff]", body))
    words = len(re.findall(r"[A-Za-z0-9_][A-Za-z0-9_.\-/]*", body))
    nonspace = len(re.sub(r"\s", "", body))
    sections, intro = [], []
    cur = None
    fence = in_table = False
    run = 0
    for ln in body.splitlines():
        st = ln.strip()
        if st.startswith("## ") and not fence:
            cur = {"title": st[3:], "prose": 0, "vis": 0, "maxrun": 0, "maxrows": 0}
            sections.append(cur)
            run = 0
            continue
        tgt = cur if cur is not None else None
        if st.startswith("```"):
            if not fence and tgt:
                tgt["vis"] += 1
            fence = not fence
            run = 0
            continue
        is_row = bool(re.match(r"^\|.*\|$", st))
        if is_row:
            if not in_table:
                rows = 0
                if tgt:
                    tgt["vis"] += 1
            rows += 1
            if tgt:
                tgt["maxrows"] = max(tgt["maxrows"], rows - 2)
            in_table = True
            run = 0
            continue
        in_table = False
        if fence or st.startswith("#"):
            continue
        if not st:
            continue  # 空行不打断连续纯文字 (保守口径)
        if st.startswith("!["):
            if tgt:
                tgt["vis"] += 1
            run = 0
            continue
        if tgt is None:
            intro.append(st)
            continue
        tgt["prose"] += 1
        run += 1
        tgt["maxrun"] = max(tgt["maxrun"], run)
    intro_chars = len(re.sub(r"\s", "", "".join(intro)))
    total_prose = sum(s["prose"] for s in sections) + len(intro)
    print(f"file: {path}")
    print(f"非空白字符(不含frontmatter, 含代码): {nonspace}; 汉字: {cjk}; 英文/数字词: {words}; 汉字+词: {cjk + words}")
    print(f"去代码块后非空白字符: {len(re.sub(chr(92)+'s', '', plain))}")
    print(f"引入: {len(intro)} 行, {intro_chars} 字符(非空白)")
    print(f"正文段数: {len(sections)}; 纯文字行(含引入): {total_prose}; 不含引入: {total_prose - len(intro)}")
    for s in sections:
        print(f"  - {s['title']}: 纯文字 {s['prose']} 行, 视觉 {s['vis']}, 最长连续纯文字 {s['maxrun']} 行, 最大表格数据行 {s['maxrows']}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)
