# 出图手段探测 (2026-11-01 23:45 CST, Hark 沙箱, Linux 6.12 aarch64, Python 3.12.15)

| 手段 | 结果 | 本篇用在哪 |
|---|---|---|
| 生图后端 (Hark 会话内 generate_image) | 通. 三张 1376x768 PNG, 每张约 1 MB, 缩到 1200 宽并量化 192 色后 177 到 241 KB | cow-hero.png, refcount-touch.png, gc-freeze.png |
| matplotlib 出 PNG | 通, 但要两步: 沙箱原本没装 (`ModuleNotFoundError: No module named 'matplotlib'`), `pip install matplotlib` 装上 3.11.2; 系统没有中文字体 (`fc-list` 无 CJK), 不给字体中文是方框, 下载 NotoSansSC-Regular.otf 用 `--font` 传入后正常 | cow-bars.png |
| graphviz | 不通. `which dot` 为空; `apt-get install graphviz` 报 `Could not open lock file /var/lib/dpkg/lock-frontend - open (13: Permission denied)`, `sudo -n apt-get install` 报 `Package 'graphviz' has no installation candidate` | 未用 |
| 手写 SVG | 能写, 但沙箱没有 rsvg-convert / inkscape / cairosvg, 无法本地栅格化自查中文渲染; ImageMagick 在 | 未用, 本篇不需要 |
| drawio | 未作为交付图使用 | 未用 |
