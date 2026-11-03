# B3: 三张承载事实的图 (替换 A2 的三张氛围图)

规则: 每张图都从 `b3_data.json` 画 (由 `measure_b3.py` 实测生成), 撤掉图就丢掉图里的数字. 环境: Linux 6.12.110+ aarch64, Python 3.12.15. `b3_data.json` 是第二次运行 (run2), `b3_data.run1.json` 是第一次 (run1), 两次每个值相差不超过 16 KB. **图和本文件里的数字一律取 run2**, 只在括号里附 run1.

复现: `python3 measure_b3.py && python3 make_b3.py --font NotoSansSC-Regular.otf`

| 图 | 替换 A2 的 | 图里能核对的事实 | 怎么核对 |
|---|---|---|---|
| `b3-1-layout.png` | cow-hero.png | float 槽位 32 B (24 B 对象加 8 B 空), [i] 列表槽位 64 B (16 B PyGC_Head 加 40 B 对象加 8 B 空); 遍历写的是 ob_refcnt 那 8 B, 全量 gc 写的是 PyGC_Head 那 16 B; 一页 4096 B 放 128 个 float 或 64 个列表 | `b3_data.json` 的 `layout`: ctypes 从 id(obj) 读, @16 读出 12345.5, 多一个引用时 @0 从 3 变 4, list @16 ob_size=1; `stride`: 相邻对象地址差众数 float 32 B (99.7%), list 64 B (99.6%) |
| `b3-2-step.png` | refcount-touch.png | 子进程每隔 k 个对象读一个: float 读到 k=128 (每页读 1 个) 仍是 31292 KB (30.6 MB), 全读是 31432 KB (run2); k=256 起减半. 列表的拐点在 k=64 | `step_sweep`; k 超过一页对象数之后, 脏页 = 读到的对象数 × 4 KB, 比如 float k=256: 1e6/256 × 4 KB = 15625 KB, 实测 15672 KB (run2, 图里画的这次; run1 是 15668 KB) |
| `b3-3-n.png` | gc-freeze.png (freeze 的结论在 B3 正文 0x03 改由一张 2 行的表承载, 数字来自同目录 measure.txt) | 子进程脏页随 N 线性涨, 斜率就是槽位: 2M 个 float 62808 KB = 61.3 MB (理论 N × 32 B = 62500 KB = 61.0 MB), 2M 个列表 125560 KB = 122.6 MB (理论 125000 KB = 122.1 MB), 均为 run2 | `n_sweep` |

A2 正文里对应的改法: 0x00 换成 b3-1 (解释"读也会写"和"写哪几个字节"), 0x01 换成 b3-2 (说明按页复制: 读得再稀也逃不掉), 0x03 在 cow-bars 之后加 b3-3 (给出能预估的公式: 脏页 ≈ N × 槽位). gc.freeze 那一节不再配氛围图, 由 cow-bars 里 freeze 前后两根柱子承载.

出图手段: 三张都是 matplotlib, 没有用生图. 理由: 这三张要画的都是精确的字节数、页数和曲线, 生图后端画不准数字和文字, 只适合不承载数字的示意图.
