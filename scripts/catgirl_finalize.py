#!/usr/bin/env python3
"""catgirl_finalize.py — 成品收口: 剥内侧残留竖带 / 成套 (-left/-right 一一对应) / 规格核对 / 写 manifest.

只依赖 pillow + numpy. 规格源: .agents/research/catgirl-spec.md.

用法:
  python3 catgirl_finalize.py strip IN.png OUT.png --side left|right     # 单张剥带
  python3 catgirl_finalize.py check IN.png --side left|right              # 单张核对 (spec_check 的 numpy 版)
  python3 catgirl_finalize.py build --plan plan.json --indir D --outdir D  # 按计划出全套 + manifest.json

剥带判据 (strip_inner_band):
  取画布 45%-95% 高度带, 逐列算不透明像素 (a>127) 的平均亮度.
  参考亮度 ref = 内侧 20-60 列的中位亮度; 若暗列 >= 16, 改取暗带之后 40 列的中位数重扫 (宽暗带污染 ref).
  从内侧边向内扫, 列亮度 < 0.75*ref 就继续, 上限 80 列, 得暗列数 N.
    N >= 8, 或 N >= 5 且相邻列亮度差全 < 6 (恒定背板) : 墙 / 背景深色区域, 整体剥掉.
    5-7 列且暗带近中性 (HSV 饱和度 < 0.15): 墙沿残带, 剥掉.
    其余 1-7 列: 只有 "偏色" 才剥 (与相邻 8 列比, HSV: 两边饱和度 > 0.12 且色相差 > 35 度, 或相邻饱和度 > 0.2 而暗带饱和度不到其 40% = 近灰).
              只暗不偏色 = 人物自身轮廓线 (头发/衣服描边), 保留.
  剥掉 N 列后人物整体向内侧平移 N px, 画布尺寸不变 => AR / TOP / 贴底不变, 内侧 gap 仍为 0; 内侧 2 列重设硬边.
"""
import argparse, colorsys, json, os, sys
import numpy as np
from PIL import Image, ImageFilter

BAND = (0.45, 0.95)


def _inner_first(arr, side):
    """把数组翻成 "内侧边在 x=0"; left 图内侧在右边."""
    return arr[:, ::-1] if side == 'left' else arr


def column_stats(arr, side, ncols=200):
    a = _inner_first(arr, side)
    H = a.shape[0]
    b = a[int(BAND[0] * H):int(BAND[1] * H), :ncols].astype(np.float32)
    solid = b[..., 3] > 127
    cnt = solid.sum(0)
    rgb = np.where(cnt[:, None] > 20, (b[..., :3] * solid[..., None]).sum(0) / np.maximum(cnt, 1)[:, None], np.nan)
    return rgb, rgb.mean(1)


def _hue_sat(rgb):
    r, g, b = [float(v) / 255 for v in rgb]
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    return h * 360, s  # HSV 相对饱和度: 深色描边绝对色差小, 但相对饱和度与本体接近


def dark_run(lum, ratio=0.75, cap=80):
    def ref_at(s):
        seg = lum[s:s + 40]
        seg = seg[~np.isnan(seg)]
        return float(np.median(seg)) if len(seg) else np.nan
    def scan(ref):
        n = 0
        while n < cap and n < len(lum) and (np.isnan(lum[n]) or lum[n] < ratio * ref):
            n += 1
        return n
    ref = ref_at(20)
    n = scan(ref)
    if n >= 16:
        ref = ref_at(n + 4)
        n = scan(ref)
    return n, ref


def judge_band(arr, side):
    rgb, lum = column_stats(arr, side)
    n, ref = dark_run(lum)
    info = dict(dark_cols=int(n), ref=round(ref, 1), strip=0, reason='clean')
    if n == 0:
        return info
    run = lum[:n]
    run = run[~np.isnan(run)]
    steady = len(run) >= 2 and float(np.max(np.abs(np.diff(run)))) < 6  # 相邻列亮度差 < 6 = 恒定背板
    info['steady'] = bool(steady)
    if n >= 8 or (n >= 5 and steady):
        info.update(strip=n, reason='wide-band' if n >= 8 else 'steady-band')
        return info
    band = np.nanmean(rgb[:n], 0)
    neigh = np.nanmean(rgb[n:n + 8], 0)
    hb, sb = _hue_sat(band)
    hn, sn = _hue_sat(neigh)
    dh = min(abs(hb - hn), 360 - abs(hb - hn))
    shifted = (sb > 0.12 and sn > 0.12 and dh > 35) or (sn > 0.2 and sb < 0.4 * sn)
    info.update(band_rgb=[round(v) for v in band], neigh_rgb=[round(v) for v in neigh], hue_diff=round(dh, 1),
                band_sat=round(sb, 3), neigh_sat=round(sn, 3))
    neutral = n >= 5 and sb < 0.15  # 5-7 列且近灰/中性 = 墙沿残带 (人物描边带本体色相)
    if shifted or neutral:
        info.update(strip=n, reason='hue-shift' if shifted else 'neutral-band')
    else:
        info.update(reason='outline-kept')
    return info


def strip_inner_band(arr, side):
    """返回 (新数组, 判定信息). arr: HxWx4 uint8."""
    info = judge_band(arr, side)
    n = info['strip']
    if not n:
        return arr, info
    a = _inner_first(arr, side).copy()
    out = np.zeros_like(a)
    out[:, :a.shape[1] - n] = a[:, n:]
    al = out[..., 3]
    al[:, :2] = np.where(al[:, :2] > 127, 255, 0)  # 内侧 2 列硬边
    out[al == 0, :3] = 0
    out = _inner_first(out, side)
    return np.ascontiguousarray(out), info


def residual_dark(arr, side):
    """用户口径: 参考列 = 距内侧 16px, < 72% 计偏暗 (只计连续段)."""
    _, lum = column_stats(arr, side, 40)
    ref = lum[16]
    n = 0
    for v in lum[:20]:
        if np.isnan(v) or v < 0.72 * ref:
            n += 1
        else:
            break
    return n


# ---------- spec_check (numpy 版, 与 spec 卡第 5 节同口径) ----------
def check(arr, side):
    a = arr[..., 3].astype(int)
    H, W = a.shape
    solid = a > 127
    cov = solid.sum(1) / W
    top = np.where(cov >= 0.005)[0][0] / H
    ys = np.where(solid.any(1))[0]; xs = np.where(solid.any(0))[0]
    fade_img = (np.where(a.max(1) >= 250)[0][-1] + 1) / H
    occ = (xs[-1] - xs[0] + 1) / W
    crop_x = (xs[-1] + 1) / W if side == 'left' else xs[0] / W
    inner = a[:, -1] if side == 'left' else a[:, 0]
    crop_cov = (inner > 127).sum() / H
    cx = np.nonzero(solid)[1].mean() / W
    zone = np.zeros_like(solid); zone[:int(fade_img * H) - 2] = True
    if side == 'left': zone[:, -3:] = False
    else: zone[:, :3] = False
    er = np.asarray(Image.fromarray((solid * 255).astype(np.uint8)).filter(ImageFilter.MinFilter(3))) > 127
    edge = solid & ~er & zone
    feather = (((a > 12) & (a < 243)) & zone).sum() / max(edge.sum(), 1)
    L = side == 'left'
    m = dict(W=W, H=H, AR=round(W / H, 4), TOP=round(top, 3), FIGH=round(1 - top, 3), BOTTOM=bool(ys[-1] >= H - 2),
             FADE_IMG=round(fade_img, 3), OCC=round(occ, 3), CROP_X=round(crop_x, 3), CROP_COV=round(crop_cov, 3),
             CX=round(cx, 3), FEATHER=round(float(feather), 2))
    rules = dict(AR=0.4160 <= W / H <= 0.4175, TOP=0.445 <= top <= 0.465, BOTTOM=m['BOTTOM'], FADE_IMG=fade_img >= 0.998,
                 OCC=(0.51 <= occ <= 0.86) if L else (0.54 <= occ <= 0.97),
                 CROP_X=(crop_x == 1.0) if L else (crop_x == 0.0), CROP_COV=crop_cov >= 0.25,
                 CX=(0.68 <= cx <= 0.80) if L else (0.21 <= cx <= 0.42),
                 FEATHER=1.30 <= feather <= 2.30, W=W >= 600)
    m['fail'] = [k for k, ok in rules.items() if not ok]
    return m


def load(p):
    return np.asarray(Image.open(p).convert('RGBA')).copy()


def save(arr, p):
    Image.fromarray(arr, 'RGBA').save(p, optimize=True)


def build(plan, indir, outdir):
    """plan: [{slug, idx, base: 文件名, base_side, noFlip, src, panel, character, work, pose, fade, anchor}]"""
    entries, log = [], []
    for e in plan:
        arr = load(os.path.join(indir, e['base']))
        side = e['base_side']
        arr, info = strip_inner_band(arr, side)
        other = 'right' if side == 'left' else 'left'
        name = f"{e['slug']}-{e['idx']}"
        files = {side: arr} if e.get('single') else {side: arr, other: np.ascontiguousarray(arr[:, ::-1])}
        res = {}
        for s, a in files.items():
            fn = f'{name}-{s}.png'
            save(a, os.path.join(outdir, fn))
            c = check(a, s)
            c['dark_cols_user'] = residual_dark(a, s)
            c['band_after'] = judge_band(a, s)
            res[s] = c
        fails = {s: c['fail'] for s, c in res.items() if c['fail']}
        b = res[side]
        entries.append(dict(slug=e['slug'], index=e['idx'], source=e['src'], panel=e['panel'], character=e['character'],
                            work=e['work'], pose=e['pose'],
                            left=f'{name}-left.png' if 'left' in res else None,
                            right=f'{name}-right.png' if 'right' in res else None,
                            w=b['W'], h=b['H'], anchor=e.get('anchor', 'right'), fade=e.get('fade', 0.88),
                            needs_gen=other if e.get('single') else False,
                            needs_gen_reason=e.get('why', ''),
                            band=dict(dark_cols_before=info['dark_cols'], stripped=info['strip'], verdict=info['reason']),
                            check={s: ('pass' if not c['fail'] else 'fail: ' + ','.join(
                                f"{k}={c[k]}" for k in c['fail'])) for s, c in res.items()}))
        log.append(dict(name=name, strip=info, res=res))
    return entries, log


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest='cmd', required=True)
    s1 = sp.add_parser('strip'); s1.add_argument('inp'); s1.add_argument('out'); s1.add_argument('--side', required=True)
    s2 = sp.add_parser('check'); s2.add_argument('inp'); s2.add_argument('--side', required=True)
    s3 = sp.add_parser('build'); s3.add_argument('--plan', required=True); s3.add_argument('--indir', required=True)
    s3.add_argument('--outdir', required=True); s3.add_argument('--log')
    a = ap.parse_args()
    if a.cmd == 'strip':
        arr, info = strip_inner_band(load(a.inp), a.side); save(arr, a.out); print(json.dumps(info))
    elif a.cmd == 'check':
        arr = load(a.inp); c = check(arr, a.side); c['band'] = judge_band(arr, a.side); print(json.dumps(c))
    else:
        entries, log = build(json.load(open(a.plan)), a.indir, a.outdir)
        man = {'version': 1, 'generated': '2026-11-03', 'entries': entries}
        json.dump(man, open(os.path.join(a.outdir, 'manifest.json'), 'w'), ensure_ascii=False, indent=1)
        if a.log:
            json.dump(log, open(a.log, 'w'), ensure_ascii=False, indent=1)
        for l in log:
            print(l['name'], l['strip']['reason'], l['strip']['strip'],
                  {s: (c['dark_cols_user'], c['band_after']['reason'], c['fail']) for s, c in l['res'].items()})


if __name__ == '__main__':
    main()
