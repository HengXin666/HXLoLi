import re,sys
t=open(sys.argv[1],encoding='utf-8').read()
fm=False;fence=False
for i,raw in enumerate(t.splitlines(),1):
    s=raw.strip()
    if s.startswith('```'): fence=not fence; continue
    if fence or not s or s.startswith(('#','>','|','---')): continue
    s=re.sub(r"`[^`]*`","C",s)
    s=re.sub(r"^\s*(?:[-*+]|\d+[.)])\s+","",s)
    for p in re.split(r"(?<=[。！？!?;；])|(?<=[.])(?=\s|$)",s):
        n=len(re.sub(r"\s","",p.strip()))
        if n>43: print(i,n,p.strip()[:70])
