#!/usr/bin/env bash
# 立绘 PNG 母版 -> 展示用 WebP (宽 760 = 最大槽宽 380px x 2 倍屏), 母版不动.
# 新增或替换立绘后重跑: bash scripts/catgirl-webp.sh
set -euo pipefail
cd "$(dirname "$0")/../static/img/catgirl"
find . -name '*.png' -print0 | while IFS= read -r -d '' f; do
    out="${f%.png}.webp"
    [ "$out" -nt "$f" ] && continue
    magick "$f" -resize '760x>' -quality 82 -define webp:alpha-quality=90 -define webp:method=6 "$out"
done
