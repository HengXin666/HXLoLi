import React from 'react';
import { Slide } from '@site/src/hxdeck/Slide';
import { Cover, Card } from '@site/src/hxdeck/blocks';
import { PageHeader, Bullets, Split } from '@site/src/hxdeck/ui';
import { Stat, LineChartBlock } from '@site/src/hxdeck/charts';
import { CodeBlock } from '@site/src/hxdeck/code';
import { Diagram } from '@site/src/hxdeck/diagram';
import cfFig from '@site/src/hxdeck/figures/cf-gateway';
import pipeFig from '@site/src/hxdeck/figures/deck-pipeline';

/**
 * CF 过盾链路 —— 演示页.
 *
 * 放在笔记同目录, 用 [标题 ##PPT##](cf-gateway-deck.tsx) 引用.
 * 与本地 .html 侧车同一套思路: 按"相对路径 + 扩展名"识别.
 *
 * 为什么导出 slides() 而不是只写组件:
 *   Deck 需要一份**铺平的 <Slide> 列表**. 若只给组件, Deck 看到的是"一个组件",
 *   数不出屏数, 导航/翻页都不会渲染.
 *   slides() 是普通函数 (不是组件), 直接调用即可拿到列表, 不涉及 React 渲染,
 *   因此没有递归风险. 默认导出只是它的包装, 供别处当普通组件用.
 */
export const slides = (): React.ReactNode => (
    <>
        <Slide title="封面" chapter="">
            <Cover eyebrow="CF-Gateway-Pro" title="从环境伪装到凭证复用" subtitle="Turnstile 检测面与过盾链路" />
        </Slide>

        <Slide title="检测面" chapter="01 机制">
            <PageHeader eyebrow="Background" title="Turnstile 在看什么" desc="它几乎不出视觉题, 只在后台收集环境信号." />
            <Split ratio="1fr 1fr">
                <Card>
                    <Bullets items={[
                        { text: '自动化工具特征: webdriver / cdc_ / plugins', strong: true },
                        'TLS 指纹: JA3 与 UA 交叉验证',
                        'Canvas / WebGL: 渲染像素差异',
                        '行为特征: 交互延迟与鼠标轨迹',
                    ]} />
                </Card>
                <Card><Diagram asset={pipeFig} kind="workflow" pad="sm" /></Card>
            </Split>
        </Slide>

        {/*
          架构图这一屏刻意做成"可把玩"的:
          打开卡片放大后 —— 节点可点 (语义护照: 上下游 + 复制链接),
          工具条可缩放/重播动效, 图本身带 trace 动效 (作者编排的顺序脉冲).
        */}
        <Slide title="架构图" chapter="01 机制">
            <PageHeader eyebrow="Architecture" title="五段式链路" desc="点节点看上下游 · 滚轮缩放 · 可放大查看 · 支持复制链接与导出" />
            <Diagram asset={cfFig} kind="architecture" pad="sm" caption="反检测 → 指纹随机化 → 点击 → Cookie 复用 → 降级" />
        </Slide>

        <Slide title="数据" chapter="02 数据">
            <PageHeader eyebrow="Data" title="命中率" />
            <div className="hxd-row">
                <Card className="hxd-fill"><Stat label="W5 命中" value={93} unit="%" hint="较 W1 提升 31pt" /></Card>
                <Card className="hxd-fill"><LineChartBlock data={[{ name: 'W1', a: 62 }, { name: 'W3', a: 78 }, { name: 'W5', a: 93 }]} keys={['a']} height={200} /></Card>
            </div>
        </Slide>

        <Slide title="一致性约束" chapter="03 代码">
            <PageHeader eyebrow="Code" title="Cookie 复用必须保指纹一致" />
            <Split ratio="1fr 1fr" align="start">
                <Card>
                    <Bullets items={[{ text: '过盾时是 Chrome 136', strong: true }, '复用时必须 impersonate 同一版本', '否则指纹不匹配被拒']} />
                </Card>
                <CodeBlock code={'s = requests.Session()\nr = s.get(url, impersonate="chrome136")'} language="python" filename="fetch.py" showLineNumbers />
            </Split>
        </Slide>

        <Slide title="结论" chapter="">
            <Cover eyebrow="结论" title="先枚举检测面, 再最小面修补" subtitle="顺序与具体对手无关" />
        </Slide>
    </>
);

export default function CfGatewayDeck(): React.ReactElement {
    return <>{slides()}</>;
}
