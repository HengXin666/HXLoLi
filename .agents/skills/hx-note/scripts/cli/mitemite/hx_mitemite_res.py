# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""填写答题卡里某个问题的答案 (入口)。

用法:
    uv run hx_mitemite_res.py <序号> [答案内容]

    uv run hx_mitemite_res.py "0x00" "我认为核心目标是..."
    uv run hx_mitemite_res.py "0x00" < answer.txt
    echo "多行答案" | uv run hx_mitemite_res.py "0x01"
    uv run hx_mitemite_res.py
"""
from __future__ import annotations

import sys
from pathlib import Path

# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from mitemite.mitemite_res import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
