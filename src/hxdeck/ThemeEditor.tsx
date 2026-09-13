import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Deck } from './Deck';
import { Slide } from './Slide';
import { Cover, Card } from './blocks';
import { PageHeader, Bullets, Callout, Split } from './ui';
import { Stat, PieChartBlock, LineChartBlock } from './charts';
import { CodeBlock } from './code';
import { Diagram } from './diagram';
import { Mascot, SlotMascot } from './meme';
import { whaleTheme } from './theme/whale';
import { hxloliTheme } from './theme/hxloli';
import { parseTheme, mergeTheme, downloadTheme } from './theme/serialize';
import type { DeckTheme } from './theme/types';
import { normalizeSlot, type SkinSlot } from './assets';
import { listThemes, loadManifest, loadThemeFile, registerTheme } from './theme/registry';
import type { ThemeInfo } from './theme/registry';
import cfFig from './figures/cf-gateway';

/**
 * 主题编辑器.
 *
 * 交互模型 (重要): **点击选中元素** -> 右侧出现该元素的属性 -> 改值即时生效.
 * 而不是"一页一页地翻着看". 用户点哪个元素, 就编辑哪个元素; 没选中时右侧给全局项.
 *
 * 保存: 生成一个主题文件放进 static/themes/, 站点自动识别.
 * 用户只需要 **起一个名字** —— 格式、文件名、清单都由系统生成.
 */

/** 可点击编辑的元素类型 */
type TargetKind = 'mascot' | 'peek' | 'pattern' | 'logo' | 'page' | 'card' | 'title' | 'chart' | 'code' | 'diagram';

interface Target {
    kind: TargetKind;
    label: string;
    hint: string;
}

const TARGETS: Record<TargetKind, Target> = {
    mascot: { kind: 'mascot', label: '吉祥物立绘', hint: '封面右下角' },
    peek: { kind: 'peek', label: '角落探头', hint: '卡片右下角' },
    pattern: { kind: 'pattern', label: '背景纹样', hint: '卡片 / 页面底纹' },
    logo: { kind: 'logo', label: '品牌标记', hint: '右上角题头' },
    page: { kind: 'page', label: '页面底色', hint: '整页背景' },
    card: { kind: 'card', label: '卡片表面', hint: '卡片底色与描边' },
    title: { kind: 'title', label: '标题文字', hint: '主标题与正文色' },
    chart: { kind: 'chart', label: '图表', hint: '数据系列色' },
    code: { kind: 'code', label: '代码块', hint: 'VS Code 配色' },
    diagram: { kind: 'diagram', label: '架构图', hint: '图内语义色' },
};

const COLOR_KEYS: { key: keyof DeckTheme['colors']; label: string }[] = [
    { key: 'bg', label: '底色' },
    { key: 'bgAlt', label: '底色渐变终点' },
    { key: 'surface', label: '卡片表面' },
    { key: 'surfaceAlt', label: '次级表面' },
    { key: 'text', label: '主文字' },
    { key: 'textMuted', label: '次要文字' },
    { key: 'primary', label: '主色' },
    { key: 'primarySoft', label: '主色柔和' },
    { key: 'accent', label: '强调色' },
    { key: 'success', label: '成功' },
    { key: 'warn', label: '警告' },
    { key: 'danger', label: '危险' },
    { key: 'border', label: '描边' },
];

/** 元素 -> 与该元素最相关的色键 (点击后就显示这些, 而不是 13 个全砸出来) */
const KEYS_FOR: Record<TargetKind, (keyof DeckTheme['colors'])[]> = {
    mascot: ['primary', 'border'],
    peek: ['border', 'primary'],
    pattern: ['primary', 'surface'],
    logo: ['text', 'primary'],
    page: ['bg', 'bgAlt'],
    card: ['surface', 'surfaceAlt', 'border'],
    title: ['text', 'textMuted'],
    chart: ['primary', 'accent', 'success', 'warn', 'danger'],
    code: ['bg', 'text', 'primary'],
    diagram: ['primary', 'accent', 'textMuted', 'danger', 'warn'],
};

const SLOT_OF: Partial<Record<TargetKind, 'mascot' | 'mascotPeek' | 'pattern' | 'logo'>> = {
    mascot: 'mascot',
    peek: 'mascotPeek',
    pattern: 'pattern',
    logo: 'logo',
};

function toHex(v: string): string {
    const s = v.trim();
    if (s.startsWith('#')) return s.slice(0, 7);
    const m = /rgba?\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)/.exec(s);
    if (!m) return '#000000';
    const h = (n: string) => Number(n).toString(16).padStart(2, '0');
    return `#${h(m[1])}${h(m[2])}${h(m[3])}`;
}

/** 生成一个安全的文件名 (用户只给名字, 其余我们生成) */
function slugify(name: string): string {
    const ascii = name.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '');
    return ascii || 'theme-' + Math.abs([...name].reduce((a, c) => a * 31 + c.charCodeAt(0), 7) % 100000);
}

export default function ThemeEditor(): React.ReactElement {
    const [theme, setTheme] = useState<DeckTheme>(() => JSON.parse(JSON.stringify(whaleTheme)));
    const [target, setTarget] = useState<TargetKind>('page');
    const [name, setName] = useState('我的主题');
    const [saved, setSaved] = useState<ThemeInfo[]>([]);
    const [msg, setMsg] = useState('');
    const fileRef = useRef<HTMLInputElement>(null);

    // 拉取已有主题 (用户放进去的也在内), 支持互相加载
    useEffect(() => {
        loadManifest().then(() => setSaved(listThemes().filter((t) => !t.builtin || t.file)));
    }, []);

    const patch = useCallback((p: Partial<DeckTheme>) => setTheme((t) => mergeTheme(t, p)), []);
    const setColor = useCallback((k: keyof DeckTheme['colors'], v: string) => {
        setTheme((t) => ({ ...t, colors: { ...t.colors, [k]: v } }));
    }, []);
    const setSlot = useCallback((k: string, v: SkinSlot) => {
        setTheme((t) => ({ ...t, assets: { ...(t.assets ?? {}), [k]: v } }));
    }, []);

    const onPickImage = useCallback((key: string, file: File) => {
        const r = new FileReader();
        r.onload = () => {
            const cur = normalizeSlot(theme.assets?.[key as 'mascot']) ?? {};
            setSlot(key, { ...cur, src: String(r.result) });
        };
        r.readAsDataURL(file);
    }, [setSlot, theme.assets]);

    const active = TARGETS[target];
    const slotKey = SLOT_OF[target];
    const slot = slotKey ? normalizeSlot(theme.assets?.[slotKey]) ?? {} : null;

    /** 点击画布上的元素 -> 选中 */
    const pick = (e: React.MouseEvent, kind: TargetKind) => {
        e.stopPropagation();
        e.preventDefault();
        setTarget(kind);
    };

    const asTheme = useMemo(() => ({ ...theme, id: slugify(name), name }), [theme, name]);

    return (
        <div className="hxed">
            {/* 画布: 每个元素都可点选 */}
            <main className="hxed__stage">
                <div className="hxed__bar">
                    <span className="hxed__picked">正在编辑: <b>{active.label}</b> <small>{active.hint}</small></span>
                    <span className="hxed__tip">点画布上的元素即可选中</span>
                </div>
                <div className="hxed__deck" onClick={(e) => pick(e, 'page')}>
                    <div onClick={(e) => pick(e, 'mascot')} className="hxed__hot hxed__hot--mascot" title="吉祥物立绘" />
                    {/* showNav=false: 编辑器的画布不需要 deck 自带的章节条 ——
                        那条会把画布宽度吃掉一块, 也让用户以为"编辑器里还有个侧边栏" */}
                    <Deck theme={asTheme} fill index={0} showNav={false} showDots={false} showPager={false} showBrand={false}>
                        <Slide title="预览" chapter="">
                            <Cover eyebrow={name} title="主题预览" subtitle="点击画布上的元素 → 右侧编辑它" />
                            <Mascot />
                        </Slide>
                        <Slide title="卡片" chapter="">
                            <PageHeader eyebrow="Card" title="卡片与图表" />
                            <div className="hxd-row">
                                <span onClick={(e) => pick(e, 'card')} className="hxed__hot hxed__hot--fill">
                                    <Card tex skin={<SlotMascot />}><Stat label="主色" value={<span>{toHex(theme.colors.primary)}</span>} /></Card>
                                </span>
                                <span onClick={(e) => pick(e, 'chart')} className="hxed__hot hxed__hot--fill">
                                    <Card tex><LineChartBlock data={[{ name: 'W1', a: 5 }, { name: 'W2', a: 9 }, { name: 'W3', a: 7 }]} keys={['a']} height={150} /></Card>
                                </span>
                            </div>
                            <div className="hxd-row">
                                <span onClick={(e) => pick(e, 'chart')} className="hxed__hot hxed__hot--fill">
                                    <Card><PieChartBlock data={[{ name: 'A', value: 4 }, { name: 'B', value: 6 }]} height={170} /></Card>
                                </span>
                                <span onClick={(e) => pick(e, 'code')} className="hxed__hot hxed__hot--fill">
                                    <CodeBlock code={'const t = {\n  primary: "' + toHex(theme.colors.primary) + '",\n};'} language="tsx" filename="theme.ts" />
                                </span>
                            </div>
                        </Slide>
                        <Slide title="架构图" chapter="">
                            <span onClick={(e) => pick(e, 'diagram')} className="hxed__hot hxed__hot--fill">
                                <Diagram asset={cfFig} kind="architecture" pad="sm" />
                            </span>
                        </Slide>
                    </Deck>
                </div>
            </main>

            {/* 右侧属性面板 */}
            <aside className="hxed__panel">
                <div className="hxed__section">
                    <div className="hxed__h">主题名</div>
                    <div className="hxed__row">
                        <input value={name} onChange={(e) => setName(e.target.value)} placeholder="给它起个名字" style={{ flex: 1 }} />
                    </div>
                    <div className="hxed__btns">
                        <button type="button" onClick={() => downloadTheme(asTheme, 'yaml')}>保存主题</button>
                    </div>
                    <div className="hxed__hint">保存后放进主题文件夹即可被自动识别; 只需起个名字, 其余自动生成.</div>
                </div>

                <div className="hxed__section">
                    <div className="hxed__h">{active.label} · 配色</div>
                    {KEYS_FOR[target].map((key) => (
                        <label className="hxed__row" key={key}>
                            <span>{COLOR_KEYS.find((c) => c.key === key)?.label ?? key}</span>
                            <span className="hxed__colorwrap">
                                <input type="color" value={toHex(theme.colors[key])} onChange={(e) => setColor(key, e.target.value)} />
                                <input className="hxed__hex" value={theme.colors[key]} onChange={(e) => setColor(key, e.target.value)} />
                            </span>
                        </label>
                    ))}
                    <button type="button" onClick={() => { setTarget('page'); setMsg('已切到全局配色, 下面是全部色键'); }}>全部颜色…</button>
                    {target === 'page' ? COLOR_KEYS.map(({ key, label }) => (
                        <label className="hxed__row" key={key}>
                            <span>{label}</span>
                            <span className="hxed__colorwrap">
                                <input type="color" value={toHex(theme.colors[key])} onChange={(e) => setColor(key, e.target.value)} />
                                <input className="hxed__hex" value={theme.colors[key]} onChange={(e) => setColor(key, e.target.value)} />
                            </span>
                        </label>
                    )) : null}
                </div>

                {slotKey ? (
                    <div className="hxed__section">
                        <div className="hxed__h">{active.label} · 图片</div>
                        <div className="hxed__slotbody">
                            <div className="hxed__thumb" style={slot?.src ? { backgroundImage: `url(${slot.src})` } : undefined}>
                                {!slot?.src ? <span>无图</span> : null}
                            </div>
                            <div className="hxed__slotfields">
                                <input type="file" accept="image/*" onChange={(e) => { const f = e.target.files?.[0]; if (f) onPickImage(slotKey, f); }} />
                                <div className="hxed__mini">
                                    <label>X<input value={slot?.x ?? ''} placeholder="34px" onChange={(e) => setSlot(slotKey, { ...slot, x: e.target.value })} /></label>
                                    <label>Y<input value={slot?.y ?? ''} placeholder="30px" onChange={(e) => setSlot(slotKey, { ...slot, y: e.target.value })} /></label>
                                    <label>宽<input value={slot?.w ?? ''} placeholder="118px" onChange={(e) => setSlot(slotKey, { ...slot, w: e.target.value })} /></label>
                                    <label>高<input value={slot?.h ?? ''} placeholder="118px" onChange={(e) => setSlot(slotKey, { ...slot, h: e.target.value })} /></label>
                                    <label>透明<input type="number" step="0.05" min="0" max="1" value={slot?.opacity ?? 1} onChange={(e) => setSlot(slotKey, { ...slot, opacity: Number(e.target.value) })} /></label>
                                    <label>旋转<input type="number" value={slot?.rotate ?? 0} onChange={(e) => setSlot(slotKey, { ...slot, rotate: Number(e.target.value) })} /></label>
                                    <label>圆角<input value={slot?.radius ?? ''} placeholder="50%" onChange={(e) => setSlot(slotKey, { ...slot, radius: e.target.value })} /></label>
                                    <label>融合
                                        <select value={slot?.blend ?? 'normal'} onChange={(e) => setSlot(slotKey, { ...slot, blend: e.target.value as SkinSlot['blend'] })}>
                                            {['normal', 'screen', 'luminosity', 'multiply', 'overlay', 'soft-light'].map((b) => <option key={b} value={b}>{b}</option>)}
                                        </select>
                                    </label>
                                </div>
                                {slot?.src ? <button type="button" className="hxed__danger" onClick={() => setSlot(slotKey, { ...slot, src: undefined })}>移除图片</button> : null}
                            </div>
                        </div>
                    </div>
                ) : null}

                <div className="hxed__section">
                    <div className="hxed__h">已有主题 (可直接加载)</div>
                    {saved.length === 0 ? <div className="hxed__msg">暂无</div> : null}
                    {saved.map((t) => (
                        <div className="hxed__row" key={t.id}>
                            <span>{t.name}</span>
                            <button type="button" onClick={async () => {
                                if (!t.file) return;
                                const got = await loadThemeFile(t.file);
                                if (got) { setTheme(got); setName(got.name); setMsg('已加载 ' + got.name); }
                            }}>载入</button>
                        </div>
                    ))}
                    <button type="button" onClick={() => fileRef.current?.click()}>从文件载入…</button>
                    <input ref={fileRef} type="file" accept=".json,.yaml,.yml" style={{ display: 'none' }}
                        onChange={async (e) => {
                            const f = e.target.files?.[0];
                            if (!f) return;
                            try { const t = parseTheme(await f.text()); setTheme(t); setName(t.name); setMsg('已载入 ' + t.name); }
                            catch (err) { setMsg('载入失败: ' + (err as Error).message); }
                        }} />
                </div>

                {/*
                  对外不暴露"配置文件"这件事: 用户只需要起个名字, 剩下的 (文件名/格式/清单) 全部生成.
                  想导出只能通过"保存主题"按钮.
                */}
                {msg ? <div className="hxed__msg">{msg}</div> : null}
            </aside>
        </div>
    );
}