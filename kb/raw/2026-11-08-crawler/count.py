"""把 `## 标题` 临时换成 `## 0xNN 标题` 后交给 hx_voice lint (它只认 0xNN 切段), 并统计字数与纯文字行."""
import re, sys, subprocess, pathlib
src = pathlib.Path(sys.argv[1]); text = src.read_text(encoding="utf-8")
lines = text.splitlines(); out=[]; k=0; fm=0
for ln in lines:
    if ln.startswith("## "):
        out.append(f"## 0x{k:02X} " + ln[3:]); k+=1
    else: out.append(ln)
tmp = pathlib.Path("/tmp")/(src.parent.name+"_lint.md"); tmp.write_text("\n".join(out)+"\n", encoding="utf-8")
# 字数: 去 frontmatter, 中文字符 + 英文单词
body = re.sub(r"^---\n.*?\n---\n", "", text, flags=re.S)
cjk = len(re.findall(r"[\u4e00-\u9fff]", body)); words = len(re.findall(r"[A-Za-z0-9_]+", body))
# 纯文字行: 不含代码块/表格/图/标题/frontmatter/空行
prose=0; fence=False; intro=0; insec=False; maxrun=0; run=0
for ln in body.splitlines():
    st=ln.strip()
    if st.startswith("```"): fence=not fence; run=0; continue
    if fence: continue
    if st.startswith("## "): insec=True; run=0; continue
    if not st or st.startswith("#") or re.match(r"^\|.*\|$", st) or st.startswith("!["):
        if not st: pass
        else: run=0
        continue
    prose+=1; run+=1; maxrun=max(maxrun,run)
    if not insec: intro+=1
print(f"{src}: CJK字={cjk} 英文词/数字={words} 合计={cjk+words}; 纯文字行(含引入)={prose} 其中引入={intro}; 段数={k}; 最长连续纯文字={maxrun}")
r = subprocess.run(["uv","run",".agents/skills/hx-note/scripts/cli/voice/hx_voice.py","lint",str(tmp),"--profile","article"], cwd="/workspace/work/hx/skill/HXLoLi", capture_output=True, text=True)
print(r.stdout[-3500:], r.stderr[-500:])
