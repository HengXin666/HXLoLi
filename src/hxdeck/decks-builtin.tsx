import React from 'react';
import { Slide } from './Slide';
import { Cover, Card } from './blocks';
import { PageHeader, Bullets, Callout, Split, Figure } from './ui';
import { Stat, PieChartBlock, LineChartBlock } from './charts';
import { CodeBlock } from './code';
import { Diagram } from './diagram';
import { registerDeck } from './decks';
import cfFig from './figures/cf-gateway';
import pipeFig from './figures/deck-pipeline';

/**
 * 内置演示页.
 *
 * 放在 .tsx 而非 .ts: 含 JSX 的文件必须是 .tsx, 否则 Babel 会把 `<>` 当成 TS 类型参数而报错.
 * 这些 deck 用 `[##PPT 页码 主题##](名字)` 在笔记里引用.
 */

/** demo: 最小自检, 用于验证指令解析 */
registerDeck('demo', {
    title: '指令自检',
    render: () => (
        <>
            <Slide title="指令生效" chapter="自检">
                <Cover eyebrow="PPT 指令" title="新语法已生效" subtitle="页码 / 主题 / 宽度 均由链接文字声明" />
            </Slide>
            <Slide title="第二页" chapter="自检">
                <Cover eyebrow="页码" title="你通过指令跳到了第 2 页" subtitle="[##PPT 2##](demo) 会直接打开这一页" />
            </Slide>
            <Slide title="第三页" chapter="自检">
                <Cover eyebrow="主题" title="主题来自指令或 URL" subtitle="?theme=hxloli 也会生效" />
            </Slide>
        </>
    ),
});

/** 过盾链路: 真实内容演示, 含架构图 */
registerDeck('cf-gateway', {
    title: 'CF 过盾链路',
    render: () => (
        <>
            <Slide title="封面" chapter="">
                <Cover eyebrow="CF-Gateway-Pro" title="从环境伪装到凭证复用" subtitle="Turnstile 检测面与过盾链路" />
            </Slide>
            <Slide title="架构图" chapter="01 机制">
                <PageHeader eyebrow="Architecture" title="五段式链路" desc="滚轮缩放 · 拖拽平移 · 可全屏" />
                <Diagram asset={cfFig} kind="architecture" pad="sm" caption="反检测 → 指纹随机化 → 点击 → Cookie 复用 → 降级" />
            </Slide>
            <Slide title="生产流程" chapter="01 机制">
                <PageHeader eyebrow="Workflow" title="演示页生产流程" />
                <Diagram asset={pipeFig} kind="workflow" pad="sm" />
            </Slide>
            <Slide title="数据" chapter="02 数据">
                <PageHeader eyebrow="Data" title="命中率与构成" />
                <div className="hxd-row">
                    <Card className="hxd-fill"><Stat label="W5 命中" value={93} unit="%" hint="较 W1 提升 31pt" /></Card>
                    <Card className="hxd-fill"><PieChartBlock data={[{ name: '结构化', value: 41 }, { name: '半结构化', value: 27 }, { name: '非结构化', value: 32 }]} height={220} /></Card>
                </div>
            </Slide>
            <Slide title="代码" chapter="03 代码">
                <PageHeader eyebrow="Code" title="一致性约束" />
                <Split ratio="1fr 1fr" align="start">
                    <Card><Bullets items={[{ text: '过盾时是 Chrome 136', strong: true }, '复用时必须 impersonate 同一版本', '否则指纹不匹配被拒']} /></Card>
                    <CodeBlock code={'s = requests.Session()\nr = s.get(url, impersonate="chrome136")'} language="python" filename="fetch.py" showLineNumbers />
                </Split>
            </Slide>
            <Slide title="收尾" chapter="">
                <Cover eyebrow="结论" title="先枚举检测面, 再最小面修补" subtitle="顺序与具体对手无关" />
            </Slide>
        </>
    ),
});
