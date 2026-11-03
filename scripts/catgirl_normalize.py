#!/usr/bin/env python3
"""catgirl_normalize.py — 探头立绘归一化 (一格一图). 只依赖 pillow + numpy.

规格源: HXLoLi/.agents/research/catgirl-spec.md (26 张成品实测):
  AR = 5:12; 人物贴底, 占下方 FIGH = 0.55, 上方 TOP = 0.45 全透明;
  竖直裁切线贴内侧边 (left 图 x=W, right 图 x=0); 图内不烤淡出 (fade 交给 CSS);
  羽化 gaussian sigma 1.1, 只向内收 (min), 内侧 2 列保持硬边; 原生 W >= 600; RGBA 无损 PNG.

抠图引擎 (--engine):
  chroma  无模型. 四边取样定背景色 -> 背景候选 (颜色距离 / 棋盘格两色 / 暗墙低饱和低明度)
          -> 只从四条边 floodfill 吃掉与边连通的背景 -> 其余全部是人物 (自动补洞).
  rembg   可选, 装了 rembg 才可用 (isnet-anime 模型).
  auto    有 rembg 用 rembg, 否则 chroma. (缺省)

用法:
  python3 catgirl_normalize.py SRC.png --panel {A,B,single} --out OUT.png
        [--side left|right] [--mirror] [--width 760] [--engine auto|chroma|rembg] [--json meta.json]
  --panel  A=左格 B=右格 (双格合成图按中线切) single=整图
  --side   缺省按实测锚点: 天然墙在右 => left 图
  --mirror 水平镜像 (服饰有字的由调用方禁止)
"""
import argparse, json, os
import sys
import numpy as np
from PIL import Image, ImageDraw, ImageFilter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from catgirl_finalize import strip_inner_band  # 内侧墙沿 / 深色背板残带剥离 (判据见该文件头)

AR = 5 / 12
FIGH = 0.55
SIGMA = 1.1
_CACHE = {}
_REMBG = None


# ---------- 小工具 (替代 scipy.ndimage) ----------
def _conv1d(arr, k, axis):
    r = len(k) // 2
    pad = [(0, 0)] * arr.ndim; pad[axis] = (r, r)
    p = np.pad(arr, pad, mode='edge')
    out = np.zeros_like(arr, dtype=np.float32)
    n = arr.shape[axis]
    for i, w in enumerate(k):
        out += w * np.take(p, np.arange(i, i + n), axis=axis)
    return out


def _blur(arr, sigma):
    """可分离高斯 (numpy), 2D 或 HxWxC."""
    r = max(1, int(round(3 * sigma)))
    x = np.arange(-r, r + 1, dtype=np.float32)
    k = np.exp(-x ** 2 / (2 * sigma ** 2)); k /= k.sum()
    return _conv1d(_conv1d(arr.astype(np.float32), k, 0), k, 1)


def _box(arr, size):
    k = np.full(size, 1.0 / size, np.float32)
    return _conv1d(_conv1d(arr.astype(np.float32), k, 0), k, 1)


def _morph(mask, op, k=3):
    im = Image.fromarray((mask * 255).astype(np.uint8), 'L')
    f = ImageFilter.MinFilter(k) if op == 'erode' else ImageFilter.MaxFilter(k)
    return np.asarray(im.filter(f)) > 127


def _fill_from_border(cand):
    """只保留与四边连通的 True 区域 (PIL floodfill)."""
    H, W = cand.shape
    im = Image.fromarray(np.where(cand, 255, 0).astype(np.uint8), 'L').copy()  # fromarray 是只读视图, floodfill 写不进去
    px = im.load()
    seeds = [(x, 0) for x in range(W)] + [(x, H - 1) for x in range(W)] + \
            [(0, y) for y in range(H)] + [(W - 1, y) for y in range(H)]
    for s in seeds:
        if px[s] == 255:
            ImageDraw.floodfill(im, s, 128)
    return np.asarray(im) == 128


def _masked_mean_color(rgb, mask, r):
    """mask 区域颜色向外扩散的估计 (归一化模糊), 用于背景色 / 内部色估计."""
    m = mask.astype(np.float32)
    num = _blur(rgb * m[..., None], r)
    den = _blur(m, r)[..., None]
    return num / np.maximum(den, 1e-4), den[..., 0]


# ---------- 分割 ----------
def bg_kind(rgb):
    f = rgb.astype(np.float32)
    border = np.concatenate([f[:8].reshape(-1, 3), f[-8:].reshape(-1, 3), f[:, :8].reshape(-1, 3), f[:, -8:].reshape(-1, 3)])
    m = border.mean(); s = border.std(0).mean()
    if m > 235 and s < 8: return 'white'
    if m > 190 and s >= 8: return 'checker'
    if m < 14 and s < 6: return 'black'
    if m < 80: return 'wall'
    return 'color'


def segment_chroma(rgb):
    f = rgb.astype(np.float32)
    H, W, _ = f.shape
    border = np.concatenate([f[:6].reshape(-1, 3), f[-6:].reshape(-1, 3), f[:, :6].reshape(-1, 3), f[:, -6:].reshape(-1, 3)])
    lum = f.mean(-1); sat = f.max(-1) - f.min(-1)
    cand = np.zeros((H, W), bool)
    # 1) 边框主色 (白 / 黑 / 紫 等纯色): 取边框颜色的两个主簇 (覆盖棋盘格明暗两色)
    bl = border.mean(-1)
    for sel in (bl >= np.median(bl), bl < np.median(bl)):
        if sel.sum() < 20: continue
        c = np.median(border[sel], 0)
        cand |= np.sqrt(((f - c) ** 2).sum(-1)) < 32
    # 2) 烘焙棋盘格: 低饱和 + 亮 (白/浅灰两色方格)
    cand |= (sat < 12) & (lum > 180)
    # 3) 暗墙纹理: 低饱和 + 暗 (局部有砖缝纹理也算), 人物暗部饱和度通常更高或被亮色包围
    mean = _box(lum, 9); var = _box(lum ** 2, 9) - mean ** 2
    cand |= (lum < 60) & (sat < 26) & (var < 400)
    cand = _morph(_morph(cand, 'erode'), 'dilate')  # 开运算去噪
    bg = _fill_from_border(cand)
    fg = ~bg  # 未被边连通背景吃掉的 = 人物 (内部同色区域天然保留, 等价补洞)
    fg = _morph(_morph(fg, 'erode'), 'dilate')
    return _blur(fg.astype(np.float32), 0.6)


def segment_rembg(rgb):
    global _REMBG
    from rembg import new_session, remove
    if _REMBG is None:
        _REMBG = new_session('isnet-anime')
    m = np.asarray(remove(Image.fromarray(rgb), session=_REMBG, only_mask=True), np.float32) / 255
    a = np.clip((m - 0.46) / 0.08, 0, 1)
    return a * a * (3 - 2 * a)


def segment(rgb, engine):
    if engine in ('rembg', 'auto'):
        try:
            return segment_rembg(rgb), 'rembg'
        except Exception:
            if engine == 'rembg':
                raise
    return segment_chroma(rgb), 'chroma'


# ---------- 锚点 / 溢色 / 羽化 ----------
def measure_anchor(a):
    """逐行最左/最右实像素, 下部 65% 取 ±2px 众数; 竖直直边更长的一侧是墙 (裁切线)."""
    solid = a > 0.5
    rows = np.where(solid.any(1))[0]
    y0, y1 = rows[0], rows[-1]
    band = rows[rows > y0 + (y1 - y0) * 0.35]
    R = np.array([np.where(solid[y])[0][-1] for y in band])
    L = np.array([np.where(solid[y])[0][0] for y in band])
    def straight(v):
        best = max(np.unique(v), key=lambda x: np.sum(np.abs(v - x) <= 2))
        return int(best), float(np.mean(np.abs(v - best) <= 2))
    rx, rs = straight(R); lx, ls = straight(L)
    return dict(anchor='right' if rs >= ls else 'left', right_x=rx, right_share=round(rs, 3),
                left_x=lx, left_share=round(ls, 3))


def decontaminate(rgb, a):
    """去溢色: C = aF + (1-a)B 反解 F; 再把窄边缘颜色拉向人物内部色, 去白边/绿边/黑边."""
    f = rgb.astype(np.float32)
    B, _ = _masked_mean_color(f, a < 0.02, 6)
    aa = a[..., None]
    F = np.clip((f - (1 - aa) * B) / np.maximum(aa, 0.05), 0, 255)
    semi = (a > 0.02) & (a < 0.98)
    out = f.copy(); out[semi] = F[semi]
    inner = _morph(a > 0.9, 'erode')
    I, _ = _masked_mean_color(out, inner, 2)
    edge = (a > 0.02) & (a < 0.6)
    out[edge] = 0.5 * out[edge] + 0.5 * I[edge]
    return out


def feather(binary, inner_side, hard_cols=2):
    """sigma 1.1 高斯后与原 alpha 取 min (只向内收), 内侧 2 列硬边."""
    a = binary.astype(np.float32)
    f = np.minimum(a, _blur(a, SIGMA))
    if inner_side == 'right':
        f[:, -hard_cols:] = a[:, -hard_cols:]
    else:
        f[:, :hard_cols] = a[:, :hard_cols]
    return f


def cut_panel(im, panel):
    W, H = im.size
    if panel == 'A':
        return im.crop((0, 0, W // 2, H))
    if panel == 'B':
        return im.crop((W // 2, 0, W, H))
    return im


# ---------- 主流程 ----------
def normalize(src, panel, side=None, mirror=False, width=760, engine='auto'):
    p = np.asarray(cut_panel(Image.open(src).convert('RGB'), panel))
    key = (src, panel, engine)
    if key not in _CACHE:
        _CACHE[key] = segment(p, engine)
    a, used = _CACHE[key]
    a = a.copy()
    anc = measure_anchor(a)
    nat = anc['anchor']
    if nat == 'right':  # 裁切线 = 墙侧直边众数, 往里收 1-2px 避开墙框高光
        cx = anc['right_x'] - 1; lost = int((a[:, cx:] > 0.5).sum()); a[:, cx:] = 0
    else:
        cx = anc['left_x'] + 2; lost = int((a[:, :cx] > 0.5).sum()); a[:, :cx] = 0
    rgb = decontaminate(p, a)
    rgba = np.dstack([rgb, a * 255]).astype(np.uint8)
    solid = a > 0.5
    top = int(np.where(solid.sum(1) >= 2)[0][0])
    bot = int(np.where(solid.any(1))[0][-1]) + 1
    xs = np.where(solid.any(0))[0]
    x0, x1 = (int(xs[0]), cx) if nat == 'right' else (cx, int(xs[-1]) + 1)
    fig = Image.fromarray(rgba[top:bot, x0:x1], 'RGBA')
    W = width; H = round(W / AR); fh = round(H * FIGH)
    s = fh / fig.height
    fig = fig.resize((max(1, round(fig.width * s)), fh), Image.LANCZOS)
    fig_side = 'left' if nat == 'right' else 'right'
    if mirror:
        fig = fig.transpose(Image.FLIP_LEFT_RIGHT)
        fig_side = 'right' if fig_side == 'left' else 'left'
    want = side or fig_side
    if want != fig_side:
        raise SystemExit(f'side mismatch: figure gives a {fig_side} image, asked {want} (toggle --mirror)')
    over = max(0, fig.width - W)
    canvas = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    if want == 'left':
        canvas.alpha_composite(fig.crop((over, 0, fig.width, fh)), (W - min(W, fig.width), H - fh))
    else:
        canvas.alpha_composite(fig.crop((0, 0, fig.width - over, fh)), (0, H - fh))
    arr = np.asarray(canvas).copy()
    al = (feather(arr[..., 3] > 127, 'right' if want == 'left' else 'left') * 255).round().astype(np.uint8)
    al[: H - fh - 1] = 0  # 上方 45% 全透明
    arr[..., 3] = al
    arr[al == 0, :3] = 0
    arr, band = strip_inner_band(arr, want)  # 找裁切线后再剥残带, 剥完内侧边重新贴到实像素边
    al = arr[..., 3]
    out = Image.fromarray(arr, 'RGBA')
    sol = al > 127
    meta = dict(src=os.path.basename(src), panel=panel, engine=used, bg=bg_kind(p), native_anchor=nat,
                anchor_measure=anc, crop_col_src=int(cx), lost_solid_px=lost, side=want, mirror=mirror,
                w=W, h=H, scale=round(s, 4), overflow_px=int(over),
                fig_share_h=round(sol.any(1).sum() / H, 4), fig_share_area=round(float(sol.mean()), 4), band=band)
    return out, meta


def save(out, path):
    out.save(path, optimize=True)
    lossy = False
    if os.path.getsize(path) > 900 * 1024:  # 超 900KB: 有损量化, 保 alpha
        out.quantize(colors=256, method=Image.FASTOCTREE, dither=Image.NONE).save(path, optimize=True)
        lossy = True
    return os.path.getsize(path), lossy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src'); ap.add_argument('--panel', default='single', choices=['A', 'B', 'single'])
    ap.add_argument('--out', required=True); ap.add_argument('--side', choices=['left', 'right'])
    ap.add_argument('--mirror', action='store_true'); ap.add_argument('--width', type=int, default=760)
    ap.add_argument('--engine', default='auto', choices=['auto', 'chroma', 'rembg'])
    ap.add_argument('--json')
    a = ap.parse_args()
    out, meta = normalize(a.src, a.panel, a.side, a.mirror, a.width, a.engine)
    meta['bytes'], meta['lossy'] = save(out, a.out)
    meta['file'] = os.path.basename(a.out)
    if a.json:
        json.dump(meta, open(a.json, 'w'), ensure_ascii=False, indent=1)
    print(json.dumps(meta, ensure_ascii=False))


if __name__ == '__main__':
    main()
