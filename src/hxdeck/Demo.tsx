import React, { useEffect, useState } from 'react';
import { Deck } from './Deck';
import { Slide } from './Slide';
import { Cover, Card } from './blocks';
import {
    PageHeader, Split, Columns, Bullets, Callout, Quote, Steps, Badge, Divider, KeyValues, Figure,
} from './ui';
import { PieChartBlock, LineChartBlock, BarChartBlock, Tree, Stat } from './charts';
import { CodeBlock, Code } from './code';
import { Mascot } from './meme';
import { Diagram, DiagramWithNotes } from './diagram';
import { Embed } from './embed';
import { Timeline, Meter, Gauge, CompareTable, FullBleed, CodeDiff } from './ui-more';
import { hxloliTheme } from './theme/hxloli';
import { whaleTheme } from './theme/whale';
import type { DeckTheme } from './theme/types';
import demoImg from './assets/memes/035.webp';
import cfFig from './figures/cf-gateway';
import pipeFig from './figures/deck-pipeline';

const THEMES: Record<string, DeckTheme> = { whale: whaleTheme, hxloli: hxloliTheme };

const trend = [
    { name: 'W1', 命中: 62, 未命中: 38 },
    { name: 'W2', 命中: 71, 未命中: 29 },
    { name: 'W3', 命中: 78, 未命中: 22 },
    { name: 'W4', 命中: 86, 未命中: 14 },
    { name: 'W5', 命中: 93, 未命中: 7 },
];
const share = [
    { name: '结构化', value: 41 },
    { name: '半结构化', value: 27 },
    { name: '非结构化', value: 32 },
];
const tree = {
    name: 'hxdeck 控件层',
    children: [
        { name: '版式原语', children: [{ name: 'PageHeader' }, { name: 'Split' }, { name: 'Columns' }] },
        { name: '内容块', children: [{ name: 'Bullets' }, { name: 'Callout' }, { name: 'Steps' }] },
        { name: '数据与代码', children: [{ name: 'Charts' }, { name: 'CodeBlock' }] },
        { name: '外挂', children: [{ name: 'Embed' }, { name: 'Diagram' }] },
    ],
};
const SAMPLE = [
    'export function useDeckTheme(): DeckTheme {',
    '    const t = useContext(DeckThemeContext);',
    '    if (!t) throw new Error("必须包在 <Deck> 内");',
    '    return t;',
    '}',
].join('\n');

export default function Demo(): React.ReactElement {
    const [which, setWhich] = useState<string>('whale');
    const theme = THEMES[which];
    const [h, setH] = useState(560);
    const [note, setNote] = useState('');

    useEffect(() => {
        const calc = () => setH(Math.max(380, window.innerHeight - 100));
        calc();
        window.addEventListener('resize', calc);
        return () => window.removeEventListener('resize', calc);
    }, []);

    return (
        <div style={{ width: '100%', padding: '12px 18px 16px' }}>
            <div style={{ display: 'flex', gap: 8, marginBottom: 10, alignItems: 'center', flexWrap: 'wrap' }}>
                {Object.keys(THEMES).map((k) => (
                    <button key={k} type="button" onClick={() => setWhich(k)}
                        style={{ padding: '6px 16px', borderRadius: 9, cursor: 'pointer', fontSize: 14,
                            border: '1px solid #8884', background: which === k ? '#8882' : 'transparent',
                            color: 'inherit', fontWeight: which === k ? 700 : 400 }}>
                        {k === 'whale' ? '🐋 鲸鱼娘' : '💜 HXLoLi'}
                    </button>
                ))}
                <input value={note} onChange={(e) => setNote(e.target.value)} placeholder="试着打字 (验证交互未被吞)"
                    style={{ padding: '6px 12px', borderRadius: 8, border: '1px solid #8884', background: 'transparent', color: 'inherit', fontSize: 13, width: 220 }} />
                <span style={{ opacity: 0.55, fontSize: 13 }}>滚轮 / ↑↓ 翻页 · 地址栏 ?page=N · 左侧目录可点</span>
            </div>

            <div style={{ height: h }}>
                <Deck theme={theme} fill syncUrl themeId={which}>
                    <Slide title="封面" chapter="">
                        <Cover eyebrow="HXDECK · 控件总览" title="原生控件 30 个" subtitle="版式 / 内容 / 数据 / 代码 / 图 / 外挂 —— 全部吃主题 token" />
                        <Mascot />
                    </Slide>

                    <Slide title="页头与两栏" chapter="01 版式">
                        <PageHeader i={0} eyebrow="Layout" title="页头与两栏" desc="每个控件都只吃主题 token; 换主题时结构一像素不动." />
                        <Split i={1} ratio="1.15fr 1fr"
                            left={<Card><Bullets items={[
                                { text: 'PageHeader: 统一每页开场结构', strong: true },
                                'Split: 左文右图 / 左图右文, 支持比例',
                                'Columns: 2~4 等分, 用于并列对比',
                            ]} /></Card>}
                            right={<Figure src={demoImg} alt="示例" ratio="4/3" fit="cover" tone="card" caption="Figure: 任意图片都像版面的一部分" />} />
                    </Slide>

                    <Slide title="内容块" chapter="02 内容">
                        <PageHeader i={0} eyebrow="Content" title="提示 / 徽章 / 键值" />
                        <div className="hxd-row" style={{ marginTop: 12 }}>
                            <div className="hxd-col hxd-fill">
                                <Callout i={1} kind="tip" title="tip">提示条支持 4 种语义色, 取自主题.</Callout>
                                <Callout i={2} kind="warn" title="warn">警示用主题的 warn 语义色.</Callout>
                            </div>
                            <Card className="hxd-fill">
                                <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap', marginBottom: 14 }}>
                                    <Badge tone="brand">brand</Badge>
                                    <Badge tone="success">success</Badge>
                                    <Badge tone="warn">warn</Badge>
                                    <Badge tone="danger">danger</Badge>
                                    <Badge>neutral</Badge>
                                </div>
                                <Divider label="KeyValues" />
                                <KeyValues items={[
                                    { key: '主题数', value: '3' },
                                    { key: '控件', value: '30' },
                                    { key: '新增依赖', value: '0' },
                                ]} />
                            </Card>
                        </div>
                    </Slide>

                    <Slide title="步骤与引用" chapter="02 内容">
                        <PageHeader i={0} eyebrow="Content" title="步骤与引用" />
                        <Split i={1} ratio="1fr 1fr">
                            <Steps steps={[
                                { title: '枚举检测面', desc: '先建立对手模型' },
                                { title: '最小面修补', desc: '只改被检测的出口' },
                                { title: '自适应降级', desc: '失败率驱动切换' },
                            ]} />
                            <Quote cite="方法论">先枚举对手检测面, 再按最小攻击面逐点修补.</Quote>
                        </Split>
                    </Slide>

                    <Slide title="数据类" chapter="03 数据">
                        <PageHeader i={0} eyebrow="Data" title="指标与占比" />
                        <div className="hxd-row" style={{ marginTop: 12 }}>
                            <div className="hxd-col" style={{ flex: '0 0 380px' }}>
                                <Card i={1} tex skin={<Mascot />}><Stat label="主题数" value={3} hint="内置 2 + 示例 1" /></Card>
                                <Card i={2}><Stat label="控件" value={30} unit="个" hint="全部可换主题" /></Card>
                            </div>
                            <Card i={3} className="hxd-fill" tex><PieChartBlock data={share} height={272} /></Card>
                        </div>
                    </Slide>

                    <Slide title="趋势与层级" chapter="03 数据">
                        <PageHeader i={0} eyebrow="Data" title="趋势与层级" />
                        <div className="hxd-row" style={{ marginTop: 12 }}>
                            <Card i={1} className="hxd-fill" tex><LineChartBlock data={trend} keys={['命中', '未命中']} height={272} /></Card>
                            <Card i={2} className="hxd-fill" style={{ flex: '0 0 430px' }}><Tree root={tree} /></Card>
                        </div>
                    </Slide>

                    <Slide title="架构图" chapter="04 图库">
                        <PageHeader i={0} eyebrow="Diagram / architecture" title="架构图: 滚轮缩放 · 拖拽平移" desc="内联 SVG 跟随主题换色; 滚轮只缩放图, 不会把页面滚走." />
                        <DiagramWithNotes i={1} ratio="1.75fr 1fr" asset={cfFig} kind="architecture"
                            caption="CF-Gateway-Pro 过盾链路"
                            notes={<>
                                <Bullets items={[
                                    { text: '滚轮 = 缩放, 拖拽 = 平移', strong: true },
                                    '不用 iframe: 内联 SVG 才与版面共用字体与圆角',
                                    '配色全部来自主题变量, 切主题图跟着变',
                                ]} />
                                <Callout kind="tip" title="换主题试试">点顶部下拉换主题, 图的配色会一起变.</Callout>
                            </>} />
                    </Slide>

                    <Slide title="流程图" chapter="04 图库">
                        <PageHeader i={0} eyebrow="Diagram / workflow" title="流程图 · 泳道与节点" desc="与架构图同一套容器, 只换 asset 与 kind." />
                        <Diagram i={1} asset={pipeFig} kind="workflow" caption="演示页生产流程 (泳道: 作者 / AI / 站点)" />
                    </Slide>

                    <Slide title="外挂嵌入" chapter="04 图库">
                        <PageHeader i={0} eyebrow="Embed" title="外挂: drawio / HTML / 图片" desc="drawio 走 img 以保留可编辑性; HTML 走 iframe 以隔离样式." />
                        <div className="hxd-row" style={{ marginTop: 14 }}>
                            <Card i={1} className="hxd-fill">
                                <Bullets items={[
                                    { text: 'Embed kind="drawio" —— 在线可编辑的矢量图', strong: true },
                                    'Embed kind="html" —— 任意自包含 HTML',
                                    'Embed kind="image" —— 普通图片, 自动推断',
                                ]} />
                            </Card>
                            <Card i={2} className="hxd-fill" style={{ flex: '0 0 420px' }}>
                                <div className="hxd-body">把 <Code>.drawio.svg</Code> 或导出好的 <Code>.html</Code> 放进本目录, 用 Embed 引用即可.</div>
                            </Card>
                        </div>
                    </Slide>

                    <Slide title="时间轴与进度" chapter="05 补充">
                        <PageHeader i={0} eyebrow="Timeline / Meter" title="时间轴与进度" />
                        <Split i={1} ratio="1fr 1fr" align="start">
                            <Timeline items={[
                                { time: '2026-07', title: '骨架', desc: 'Deck + 主题契约' },
                                { time: '2026-08', title: '控件层', desc: '版式 / 内容 / 数据', tone: 'brand' },
                                { time: '2026-09', title: '图库', desc: '内联 archify 产物', tone: 'success' },
                            ]} />
                            <Card>
                                <Meter items={[
                                    { label: '控件覆盖', value: 88, hint: '版式与内容块基本齐了' },
                                    { label: '图型覆盖', value: 40, tone: 'warn', hint: '5 种图型已完成 2 种' },
                                    { label: '导出能力', value: 0, tone: 'danger', hint: '离线 HTML / PDF 未开始' },
                                ]} />
                            </Card>
                        </Split>
                    </Slide>

                    <Slide title="环形与对照" chapter="05 补充">
                        <PageHeader i={0} eyebrow="Gauge / CompareTable" title="环形指标与方案对照" />
                        <Split i={1} ratio="0.7fr 1.6fr" align="center">
                            <Gauge value={30} max={40} unit="个" label="已实现控件" size={190} />
                            <CompareTable
                                highlight={1}
                                columns={['静态 HTML 侧车', 'React 组件 (本方案)']}
                                rows={[
                                    { label: '复用博客依赖', values: ['否', '是'], tone: 'good' },
                                    { label: '换主题', values: ['重出图', '改 token'], tone: 'good' },
                                    { label: '包体', values: ['700KB+', '零额外'], tone: 'good' },
                                    { label: '离线交付', values: ['原生支持', '需导出'], tone: 'bad' },
                                ]}
                            />
                        </Split>
                    </Slide>

                    <Slide title="通栏与 diff" chapter="05 补充">
                        <PageHeader i={0} eyebrow="FullBleed / CodeDiff" title="通栏大图与代码 diff" />
                        <Split i={1} ratio="1fr 1fr" align="start">
                            <FullBleed src={demoImg} headline="一张图铺满" sub="配一句话, 压暗保证可读" overlay={0.5} />
                            <CodeDiff filename="Deck.tsx" lines={[
                                { type: 'ctx', text: 'const onWheel = (e) => {' },
                                { type: 'del', text: '  setIndex(i + 1);' },
                                { type: 'add', text: '  if (locked) return;' },
                                { type: 'add', text: '  go(currentRef.current + dir);' },
                                { type: 'ctx', text: '};' },
                            ]} />
                        </Split>
                    </Slide>

                    <Slide title="代码呈现" chapter="06 代码">
                        <PageHeader i={0} eyebrow="Code" title="代码类: 自建, VS Code 配色" />
                        <Split i={1} ratio="1fr 1.1fr" align="start">
                            <Card><BarChartBlock data={trend} keys={['命中']} height={240} /></Card>
                            <CodeBlock code={SAMPLE} language="tsx" filename="theme/context.tsx" showLineNumbers highlight={[2, 3]} maxHeight={280} />
                        </Split>
                    </Slide>

                    <Slide title="图片融入" chapter="07 图片">
                        <PageHeader i={0} eyebrow="Figure" title="图片是版面的一部分" desc="Figure 提供统一容器: 背景 / 同心圆角 / 比例 / 说明位. 换图不改版面." />
                        <Columns i={1} cols={3}>
                            <Figure src={demoImg} alt="cover" ratio="1/1" fit="cover" tone="card" caption="cover 填满裁切" />
                            <Figure src={demoImg} alt="contain" ratio="1/1" fit="contain" tone="glass" caption="contain 完整显示" />
                            <Figure src={demoImg} alt="plain" ratio="1/1" fit="cover" tone="plain" caption="plain 无框" />
                        </Columns>
                    </Slide>

                    <Slide title="收尾" chapter="">
                        <Cover eyebrow="下一步" title="控件层已成型" subtitle="待做: sequence / dataflow / lifecycle · 离线单文件导出 · PDF" />
                    </Slide>
                </Deck>
            </div>
        </div>
    );
}