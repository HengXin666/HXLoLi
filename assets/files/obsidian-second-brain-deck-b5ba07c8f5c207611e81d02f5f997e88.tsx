import React from 'react';
import { Slide } from '@site/src/hxdeck/Slide';
import { Cover, Card } from '@site/src/hxdeck/blocks';
import {
    PageHeader, Split, Columns, Bullets, Callout, Quote, Steps, Badge, Divider, KeyValues,
} from '@site/src/hxdeck/ui';
import { CompareTable, Meter } from '@site/src/hxdeck/ui-more';
import { Stat, BarChartBlock } from '@site/src/hxdeck/charts';
import { CodeBlock } from '@site/src/hxdeck/code';
import { Mascot } from '@site/src/hxdeck/meme';

/**
 * obsidian-second-brain 全解 —— 总览演示页.
 *
 * 放在笔记同目录, 用 [标题 ##PPT##](obsidian-second-brain-deck.tsx) 引用.
 * 机制图另见同目录的 #ppt 侧车 (vault-architecture.html / ingest-dataflow.html).
 */
export const slides = (): React.ReactNode => (
    <>
        <Slide title="封面" chapter="">
            <Cover
                eyebrow="HXLOLI · AI-DOCS"
                title="obsidian-second-brain 全解"
                subtitle="一个不靠增长、只靠进化的知识库 —— 它究竟由哪些机制拼出来?"
            />
            <Mascot />
        </Slide>

        <Slide title="它要解决什么" chapter="01 定位">
            <PageHeader
                i={0}
                eyebrow="Problem"
                title="两个很强的工具, 完全断开"
                desc="Claude 每次会话从零开始; Obsidian 里的几百个文件没人连接线索."
            />
            <CompareTable
                i={1}
                columns={['Claude Code', 'Obsidian vault']}
                rows={[
                    { label: '强在哪', values: ['推理、写作、联网抓取', '长期保存、可检索、可 diff'] },
                    { label: '弱在哪', values: ['关掉就忘, 每次重新解释背景', '只是文件堆, 想法烂在 daily note'], tone: 'bad' },
                ]}
            />
            <Callout i={2} kind="tip" title="它换掉的前提">
                知识库不是往里面加东西, 而是围绕新信息重写自己 —— 让 vault 自己维护自己.
            </Callout>
        </Slide>

        <Slide title="与 LLM Wiki 的差别" chapter="01 定位">
            <PageHeader
                i={0}
                eyebrow="Positioning"
                title="它是 Karpathy LLM Wiki 的演进版"
                desc="不是把模式推翻重来, 而是补齐了四个「然后呢」."
            />
            <CompareTable
                i={1}
                columns={['Karpathy 的 LLM Wiki', 'obsidian-second-brain']}
                highlight={1}
                rows={[
                    { label: '新来源', values: ['追加页面并交叉引用', '重写已有页面, 替换陈旧主张'] },
                    { label: '矛盾', values: ['标出来, 人工处理', '自动调和, 或显式记录'] },
                    { label: '模式发现', values: ['用户问才浮现', '自动找未命名模式'] },
                    { label: '运行时机', values: ['按需', '4 个定时 agent 在夜里维护'] },
                    { label: '笔记格式', values: ['人类可读 wiki 页', 'AI-first, 为未来检索而写'] },
                ]}
            />
        </Slide>

        <Slide title="它不是插件" chapter="01 定位">
            <PageHeader
                i={0}
                eyebrow="Boundary"
                title="它不是 Obsidian 插件"
                desc="这条边界决定了它能做什么, 也决定了它为什么能跨六个平台."
            />
            <Split i={1} ratio="1fr 1fr" align="start">
                <Card>
                    <KeyValues items={[
                        { key: '运行位置', value: 'Claude Code 等 CLI 内部' },
                        { key: '能力上限', value: '受「shell 能做什么」限制' },
                        { key: '对 vault 的认知', value: '就是一堆普通 Markdown' },
                        { key: '覆盖平台', value: '六个 CLI 共用一套规则' },
                    ]} />
                </Card>
                <Card>
                    <div className="hxd-body">
                        <Badge tone="brand">Obsidian 插件</Badge> 受 Vault API 限制, 做 UI 与编辑器扩展.
                        <Divider />
                        <Badge tone="success">这个 skill</Badge> 能联网研究、跑定时任务、跨年综合 —— 因为这些插件做不了.
                    </div>
                </Card>
            </Split>
        </Slide>

        <Slide title="七条 AI-first 规则" chapter="02 规则">
            <PageHeader
                i={0}
                eyebrow="AI-first"
                title="vault 是写给未来的 AI 读的"
                desc="不是给人逐页读的. 所以格式优先服务「被检索」, 而不是「被阅读」."
            />
            <Columns i={1} cols={3}>
                <Card><Badge tone="brand">1 自包含</Badge><div className="hxd-body">单条拉出来也能读懂.</div></Card>
                <Card><Badge tone="brand">2 前言</Badge><div className="hxd-body">开头 2~3 句 <code>For future Claude</code>.</div></Card>
                <Card><Badge tone="brand">3 frontmatter</Badge><div className="hxd-body">date / type / tags / ai-first.</div></Card>
                <Card><Badge tone="success">4 时效</Badge><div className="hxd-body">外部主张带 as of 日期.</div></Card>
                <Card><Badge tone="success">5 来源</Badge><div className="hxd-body">原样保留 URL, 不改写.</div></Card>
                <Card><Badge tone="success">6 双链</Badge><div className="hxd-body">人 / 项目 / 概念强制 wikilink.</div></Card>
            </Columns>
            <Callout i={2} kind="info" title="第 7 条: 置信度">
                stated / high / medium / speculation —— 推断必须自报家门, 不能和事实混在一句里.
            </Callout>
        </Slide>

        <Slide title="三条反幻觉铁律" chapter="02 规则">
            <PageHeader
                i={0}
                eyebrow="Guardrails"
                title="三种失败会静默毁掉知识库"
                desc="页面照样打开, 只是内容开始骗人 —— 所以它们被列为不可协商."
            />
            <Columns i={1} cols={3}>
                <Card><Badge tone="danger">假不存在</Badge><Divider /><div className="hxd-body">没穷尽搜索就说「没有这条笔记」. 原文称它<b>比编造更常见</b>.</div></Card>
                <Card><Badge tone="warn">采样当穷举</Badge><Divider /><div className="hxd-body">把部分扫描汇报成完整扫描, 比承认「只查了 X」更糟.</div></Card>
                <Card><Badge tone="warn">编造</Badge><Divider /><div className="hxd-body">未知写 <code>TBD</code>. 没有决策时, 空的 Decisions 章节是正确答案.</div></Card>
            </Columns>
            <Quote i={2} cite="references/ai-first-rules.md">
                不要为了让章节看起来完整而伪造内容.
            </Quote>
        </Slide>

        <Slide title="写入即传播" chapter="03 写入">
            <PageHeader
                i={0}
                eyebrow="Propagation"
                title="写入不是保存, 是传播"
                desc="每次落笔都要追问「这件事还属于哪里」, 然后把变更铺开."
            />
            <CompareTable
                i={1}
                columns={['事件', '还要更新']}
                rows={[
                    { label: '新项目', values: ['board Backlog + 今天的 daily note'] },
                    { label: '任务完成', values: ['board 移到 Done + 项目笔记 + daily'] },
                    { label: '人物互动', values: ['daily note + 人物笔记 (不存在就建 stub)'] },
                    { label: '决策产生', values: ['项目笔记 Key Decisions + daily'] },
                    { label: '任意写入', values: ['操作日志 + index.md 目录'] },
                ]}
            />
        </Slide>

        <Slide title="sentinel" chapter="03 写入">
            <PageHeader
                i={0}
                eyebrow="Regeneration"
                title="生成区与人工区必须物理隔开"
                desc="否则第一次刷新就会擦掉人工补充 —— 这是可重复生成器唯一的安全前提."
            />
            <CodeBlock
                language="markdown"
                filename="artifacts/overview.md"
                code={'<!-- @generated:start -->\n...下次刷新时可以安全覆盖...\n<!-- @generated:end -->\n\n<!-- @user:start -->\n...人工补充, 任何刷新永不触碰...\n<!-- @user:end -->'}
            />
            <Callout i={2} kind="warn" title="规则只有一条">
                只替换 @generated 区间; 标记之外的一切都视为人工所有.
            </Callout>
        </Slide>

        <Slide title="双时间事实" chapter="03 写入">
            <PageHeader
                i={0}
                eyebrow="Bitemporal"
                title="事实变化时, 不删旧值"
                desc="event time 记「何时为真」, transaction time 记「何时学到」."
            />
            <Split i={1} ratio="1.15fr 0.85fr" align="start">
                <Card>
                    <CodeBlock
                        language="yaml"
                        code={'timeline:\n  - fact: "CTO at Acme Corp"\n    from: 2024-01-01\n    until: 2026-04-07\n    learned: 2026-02-23\n  - fact: "Architect at Acme Corp"\n    from: 2026-04-07\n    until: present\n    learned: 2026-04-07'}
                    />
                </Card>
                <div className="hxd-col">
                    <Callout kind="tip" title="于是这些问题可答">
                        一月时谁是 CTO? 你周三之后为什么改了看法?
                    </Callout>
                    <Callout kind="info" title="顶层字段">
                        永远反映当前状态; timeline 保留完整历史.
                    </Callout>
                </div>
            </Split>
        </Slide>

        <Slide title="四层能力" chapter="04 命令">
            <PageHeader
                i={0}
                eyebrow="44 Commands"
                title="44 个命令, 四层能力"
                desc="43 个跨平台; 只有日历命令依赖 Google Calendar MCP."
            />
            <Columns i={1} cols={4}>
                <Card><Stat label="vault" value={16} hint="保存 · 捕获 · 查找 · 项目" /></Card>
                <Card><Stat label="thinking" value={13} hint="反驳 · 综合 · 决策 · 回顾" /></Card>
                <Card><Stat label="research" value={8} hint="X / Web / YouTube / podcast" /></Card>
                <Card><Stat label="meta" value={7} hint="初始化 · 体检 · 架构文档" /></Card>
            </Columns>
            <Callout i={2} kind="info" title="外加一个常驻层">
                background + scheduled agents: 上下文压缩后触发, 及 morning / nightly / weekly / health.
            </Callout>
        </Slide>

        <Slide title="命令的四种性格" chapter="04 命令">
            <PageHeader
                i={0}
                eyebrow="Examples"
                title="最能说明设计意图的四个"
                desc="它们分别代表保存、重写、反驳、记账四种姿态."
            />
            <CompareTable
                i={1}
                columns={['命令', '一句话', '姿态']}
                rows={[
                    { label: '/obsidian-save', values: ['抽取决策 / 人物 / 任务, 放到正确笔记', '不问你该放哪'] },
                    { label: '/obsidian-ingest', values: ['让 vault 围绕新知识重写自己', '一个来源触达 5~15 页'] },
                    { label: '/obsidian-challenge', values: ['用你的历史反驳你', '翻出失败与反转的决策'] },
                    { label: '/obsidian-world', values: ['按 L0~L3 加载身份与状态', '控制 token 预算'] },
                ]}
            />
            <Quote i={2} cite="/obsidian-challenge 的实际形态">
                你说「我想用 Rust 重写 API」—— 它翻出 2025 年失败的 post-mortem, 再翻出一份「未来两年继续用 TypeScript」的决策记录, 然后问: 还要继续吗?
            </Quote>
        </Slide>

        <Slide title="适配器模式" chapter="05 工程">
            <PageHeader
                i={0}
                eyebrow="Build"
                title="一个源, 六个平台"
                desc="commands/ 是唯一真相源; 适配层只负责编译, 不分叉维护."
            />
            <Steps i={1} steps={[
                { title: <code>{'commands/<name>.md'}</code>, desc: '声明 description / category / triggers_en, 是平台中立的产品表面.' },
                { title: <code>scripts/build.sh</code>, desc: '编排 adapters/: 全量构建, 或 --platform 只出一个平台.' },
                { title: <code>{'dist/<platform>/'}</code>, desc: '产物目录被 gitignore —— 任何时候都应该重新生成, 而不是手写修改.' },
            ]} />
            <Callout i={2} kind="tip" title="对贡献者只有一句">
                增加或修改命令时只改 <code>{'commands/<name>.md'}</code>, 下一次构建由适配器自动拾取.
            </Callout>
        </Slide>

        <Slide title="安全的默认值" chapter="05 工程">
            <PageHeader
                i={0}
                eyebrow="Safety"
                title="自动化承认自己是危险的"
                desc="这部分比功能列表更值得抄 —— 它把「不信任自己」写进了默认配置."
            />
            <Columns i={1} cols={2}>
                <Card><Badge tone="success">双开关</Badge><Divider /><div className="hxd-body">后台 agent 默认关闭, 需 <code>OBSIDIAN_VAULT_PATH</code> 且 <code>BG_AGENT_ENABLED=1</code>.</div></Card>
                <Card><Badge tone="success">只增不删</Badge><Divider /><div className="hxd-body">无人值守运行不删除、不归档、不合并.</div></Card>
                <Card><Badge tone="warn">破坏性动作要确认</Badge><Divider /><div className="hxd-body">归档 / 合并 / 解决矛盾必须显式批准.</div></Card>
                <Card><Badge tone="warn">写入后校验</Badge><Divider /><div className="hxd-body">非阻塞检查四个必备字段与前言; 失败只报警告, 不回滚写入.</div></Card>
            </Columns>
        </Slide>

        <Slide title="生态边界" chapter="05 工程">
            <PageHeader
                i={0}
                eyebrow="Ecosystem"
                title="upstream 给原语, fork 给领域"
                desc="把每个领域都吸进上游, 结果会是一个无人能维护的庞大 skill."
            />
            <Split i={1} ratio="1fr 1fr" align="start">
                <Card>
                    <Badge tone="brand">upstream 拥有</Badge>
                    <Bullets items={[
                        'vault 管理与 AI-first 规则',
                        'rewrite engine 与适配层',
                        '通用研究工具四阶段形状',
                        '可插拔的 Phase 3 backend 协议',
                    ]} />
                </Card>
                <Card>
                    <Badge tone="success">fork 拥有</Badge>
                    <Bullets items={[
                        'PubMed routing 属于学术 fork',
                        '案例法检索属于法律 fork',
                        '回流标准: 非领域用户是否受益',
                    ]} />
                </Card>
            </Split>
        </Slide>

        <Slide title="166 个 fork 的真相" chapter="06 结论">
            <PageHeader
                i={0}
                eyebrow="Reality Check"
                title="上游做了一次很诚实的复盘"
                desc="结论比功能列表更有参考价值: 生态里绝大多数 fork 并没有产生信号."
            />
            <div className="hxd-row" style={{ marginTop: 22 }}>
                <Card i={1} className="hxd-fill"><Stat label="未改动镜像" value={156} unit="个" hint="0 commits ahead" /></Card>
                <Card i={2} className="hxd-fill"><Stat label="有自己的提交" value={11} unit="个" hint="其中实质工作仅 3 个" /></Card>
                <Card i={3} className="hxd-fill"><Stat label="上游测试" value={0} unit="个" hint="被列为 P0 缺口" /></Card>
            </div>
            <Callout i={4} kind="warn" title="归纳出的强信号">
                付费 API 是采用门槛 · 日历集成被反复要求 · Codex / Windows 支持被独立验证 · 缺少反幻觉 guard.
            </Callout>
        </Slide>

        <Slide title="最值得抄的四件事" chapter="06 结论">
            <PageHeader
                i={0}
                eyebrow="Takeaways"
                title="抄设计, 不抄代码"
                desc="这四条与具体是 Obsidian 还是 Docusaurus 无关."
            />
            <Steps i={1} steps={[
                { title: '机制与代码分开', desc: '想学的机制和想抄的代码是两件事.' },
                { title: '写入规则必须机器可检查', desc: '否则 AI 生成内容很快退化成不可审计的散文.' },
                { title: '可重复生成必须带 sentinel', desc: '否则第一次刷新就会擦掉人工补充.' },
                { title: '没有穷尽搜索, 就没资格说「不存在」', desc: '它同时是反幻觉规则与检索质量的前提.' },
            ]} />
            <Callout i={2} kind="tip" title="一句话">
                最值得学的不是「换成 Obsidian」, 而是让知识写入变得可审计、可传播、可刷新、可被未来检索.
            </Callout>
        </Slide>
    </>
);

export default function ObsidianSecondBrainDeck() {
    return <>{slides()}</>;
}
