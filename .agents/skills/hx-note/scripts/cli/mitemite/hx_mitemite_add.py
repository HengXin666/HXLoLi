# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""往答题卡里追加一个问题块 (入口)。

用法:
    uv run hx_mitemite_add.py <序号> [问题内容]

    uv run hx_mitemite_add.py "0x00" "这篇笔记的核心目标是什么?"
    uv run hx_mitemite_add.py "0x01" "$(cat question.md)"
    echo "多行问题内容" | uv run hx_mitemite_add.py "0x02"
"""
from __future__ import annotations

import sys
from pathlib import Path

# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from mitemite.mitemite_add import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
