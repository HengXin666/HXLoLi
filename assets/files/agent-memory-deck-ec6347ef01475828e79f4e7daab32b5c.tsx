import React from 'react';
import { Slide } from '@site/src/hxdeck/Slide';
import { Cover, Card } from '@site/src/hxdeck/blocks';
import {
    PageHeader, Split, Columns, Bullets, Callout, Quote, Steps, Badge, Divider, KeyValues,
} from '@site/src/hxdeck/ui';
import { Timeline, Meter, Gauge, CompareTable } from '@site/src/hxdeck/ui-more';
import { Stat, BarChartBlock } from '@site/src/hxdeck/charts';
import { CodeBlock } from '@site/src/hxdeck/code';
import { Diagram } from '@site/src/hxdeck/diagram';
import { Mascot } from '@site/src/hxdeck/meme';
import memFig from '@site/src/hxdeck/figures/agent-memory-pipeline';

/**
 * Agent Memory 选型 —— 演示页.
 *
 * 放在笔记同目录, 用 [标题 ##PPT##](agent-memory-deck.tsx) 引用.
 * 图复用 archify 产物抽出的 TS 模块 (src/hxdeck/figures/agent-memory-pipeline.ts),
 * 与 .html 侧车共用同一张图, 不必画两遍.
 */
export const slides = (): React.ReactNode => (
    <>
        <Slide title="封面" chapter="">
            <Cover
                eyebrow="HXLOLI · AI-DOCS"
                title="Agent Memory 选型"
                subtitle="窗口解决容量, 记忆解决连续性 —— 在一个连分数都不可比的领域里, 选型该依据什么?"
            />
            <Mascot />
        </Slide>

        <Slide title="三概念边界" chapter="01 定位">
            <PageHeader
                i={0}
                eyebrow="Foundations"
                title="长上下文 / RAG / Agent Memory 是三件事"
                desc="它们的失效方式完全不同, 而选型失误大多始于概念混淆."
            />
            <CompareTable
                i={1}
                columns={['长上下文', 'RAG', 'Agent Memory']}
                highlight={2}
                rows={[
                    { label: '内容来源', values: ['本次请求的输入', '预先置入的静态语料', '运行期双向产生'] },
                    { label: '生命周期', values: ['单次请求, 用完即销毁', '与语料同寿, 只读', '跨会话、跨任务持久'] },
                    { label: '解决什么', values: ['容量 (装得下)', '事实召回 (查得到)', '连续性 (记得住)'] },
                    { label: '典型失效', values: ['上下文腐化', '语料过期、切分噪声', '幽灵记忆、遗忘失败'], tone: 'bad' },
                ]}
            />
            <Callout i={2} kind="tip" title="一句话判据">
                同一个客户第五次来访: RAG 每次都命中同一份产品文档, 而 Agent Memory 记得他上次问过什么.
            </Callout>
        </Slide>

        <Slide title="窗口在打折" chapter="01 定位">
            <PageHeader
                i={0}
                eyebrow="Context"
                title="买来的窗口本身就在打折"
                desc="所以把窗口从 200K 推到 1M, 并不会让 Agent 记住你上周说过什么."
            />
            <div className="hxd-row" style={{ marginTop: 22 }}>
                <Card i={1} className="hxd-fill"><Stat label="有效上下文" value={58} unit="%" hint="仅为标称窗口的 50% ~ 65%" /></Card>
                <Card i={2} className="hxd-fill"><Stat label="中段事实损失" value={30} unit="%+" hint="Lost in the Middle" /></Card>
                <Card i={3} className="hxd-fill"><Stat label="退化模型" value={'18 / 18'} hint="Context Rot: 全部随输入增长退化" /></Card>
            </div>
            <Callout i={4} kind="warn" title="正交的两件事">
                窗口扩张解决「装得下」, 记忆系统解决「记得住」. 记忆评测要先写入、再跨轮检索, 写入质量本身就是被评测对象.
            </Callout>
        </Slide>

        <Slide title="四类记忆" chapter="01 定位">
            <PageHeader i={0} eyebrow="Taxonomy" title="记忆分四类, 各有各的不可替代" />
            <Columns i={1} cols={4}>
                <Card><Badge tone="brand">语义记忆</Badge><Divider /><div className="hxd-body">事实: X 是 Y.<br />载体: 事实库、知识图谱.</div></Card>
                <Card><Badge tone="success">情景记忆</Badge><Divider /><div className="hxd-body">经历: 遇到 X 时发生了什么.<br />载体: 轨迹、事件流.</div></Card>
                <Card><Badge tone="warn">程序记忆</Badge><Divider /><div className="hxd-body">怎么做: 先做 Y、别做 Z.<br />载体: 技能包、提示词改写.</div></Card>
                <Card><Badge>参数记忆</Badge><Divider /><div className="hxd-body">学进权重里的偏好.<br />载体: 微调、蒸馏.</div></Card>
            </Columns>
            <Quote i={2} cite="本文 0x00">
                情景记忆的独有价值是保留失败与试错的分支: 事实没有褒贬, 语义记忆永远表达不出「不要先做 X」这句话.
            </Quote>
        </Slide>

        <Slide title="五大范式" chapter="02 范式">
            <PageHeader
                i={0}
                eyebrow="Paradigm"
                title="选型第一步是选范式, 不是选产品"
                desc="范式边界由三个问题划定: 记住什么 / 谁来管理 / 如何验证与审计."
            />
            <CompareTable
                i={1}
                columns={['代表实现', '换取什么', '代价']}
                highlight={0}
                rows={[
                    { label: '向量抽取型', values: ['Mem0 / LangMem', '接入成本最低', '关系推理弱'] },
                    { label: '时序知识图谱型', values: ['Zep / Graphiti', '双时间轴: 可答「当时什么是真的」', '图数据库运维, 写延迟最高'] },
                    { label: 'Agent 自编辑型', values: ['Letta', '共享记忆块是一等原语', '可审计性差, 每次操作耗推理'] },
                    { label: 'OS 调度型', values: ['MemOS', '明文 / 激活 / 参数三形态可互转', '偏架构参考, 非当下首选'] },
                    { label: '文件即真相型', values: ['Claude Code / Cursor', '零基础设施, 可 diff 可审计', '检索能力弱'] },
                ]}
            />
            <Callout i={2} kind="info" title="别被范式框住">
                工程侧的存储已收敛为混合形态 —— 向量、图、KV/SQL、文件四种基质并存, 差别只在「谁是主基质、谁是增强层」.
            </Callout>
        </Slide>

        <Slide title="失效语义" chapter="02 范式">
            <PageHeader
                i={0}
                eyebrow="Invalidation"
                title="写入没有 DELETE, 失效有五种语义"
                desc="删除请求针对的是表层形式, 而图谱抽象丢掉的恰恰是表层形式."
            />
            <Split i={1} ratio="1fr 1.15fr" align="start">
                <Card>
                    <Steps steps={[
                        { title: '写入只有三个动作', desc: 'ADD / UPDATE / NO-OP, 加版本回滚与状态标记 —— 没有 DELETE.' },
                        { title: '五种失效语义', desc: '就地覆盖 / 版本化 / 失效不删除 / 时间衰减 / Agent 自主淘汰.' },
                        { title: '召回与删除是两套器官', desc: '召回被做透了, 删除集体翻车 —— 这是遗忘评测的结构性根因.' },
                    ]} />
                </Card>
                <Card><Diagram asset={memFig} kind="dataflow" pad="sm" caption="读写两条路径只在索引层相接" /></Card>
            </Split>
        </Slide>

        <Slide title="读写铁律" chapter="02 范式">
            <PageHeader i={0} eyebrow="Read / Write" title="读取同步阻塞, 写入必须异步" desc="两条路径面对的是完全不同的约束: 读要快, 写要稳." />
            <Split i={1} ratio="0.9fr 1.1fr" align="start">
                <Card>
                    <KeyValues items={[
                        { key: '读取 p50', value: '约 300 ~ 800 ms' },
                        { key: '图遍历', value: '绝不在读路径上' },
                        { key: '写入时机', value: '会话结束时异步持久化' },
                        { key: '失败代价', value: '阻塞对话 = 用户可直接感知' },
                    ]} />
                </Card>
                <div className="hxd-col">
                    <Callout kind="danger" title="最常见的架构错误">
                        把写入耦合进实时对话路径 —— 让用户去承担索引构建与实体消解的延迟.
                    </Callout>
                    <Callout kind="tip" title="正确的分工">
                        读取只做「查」, 所有重活 (抽取、消解、建索引、归档) 全部挪到写入侧或后台.
                    </Callout>
                </div>
            </Split>
            <CompareTable
                i={2}
                columns={['反模式', '症状', '改法']}
                rows={[
                    { label: '写进对话路径', values: ['抽取 + 消解 + 建索引', '首字延迟随记忆量线性上涨', '改成会话结束事件异步落库'], tone: 'bad' },
                    { label: '读取时做图遍历', values: ['每次召回都多跳几层', 'p99 抖动, 与历史规模正相关', '预计算邻域, 读路径只取结果'], tone: 'bad' },
                    { label: '同步等待写入结果', values: ['await save() 再回包', '写入失败直接打断对话', 'fire-and-forget + 失败重试队列'] },
                ]}
            />
        </Slide>

        <Slide title="评测危机" chapter="03 评测">
            <PageHeader
                i={0}
                eyebrow="Benchmarks"
                title="在一个分数不可比的领域里选型"
                desc="同一系统在同一基准上能从 38% 跑到 92%, 变量既不是模型也不是产品, 而是谁写的测试脚手架."
            />
            <Split i={1} ratio="1fr 1fr" align="start">
                <Card>
                    <Bullets marker="arrow" items={[
                        { text: '独立审计中, 奖励向量存储的 judge 接受了约 63% 的刻意错误答案', strong: true },
                        '换一套口径 (是否计入不可回答题), 结论直接翻转',
                        'LoCoMo 头部饱和, BEAM 最贴近生产规模',
                        '有时换个底层模型, 比换个记忆框架收益更大',
                    ]} />
                </Card>
                <Card tex>
                    <div className="hxd-body" style={{ marginBottom: 14 }}>读数三件套, 缺一项就不能作为选型依据:</div>
                    <KeyValues items={[
                        { key: 'harness 版本', value: '测试脚手架' },
                        { key: 'judge 模型', value: '谁来打分' },
                        { key: '产品档位', value: '免费 / 付费 / 托管' },
                    ]} />
                </Card>
            </Split>
            <Card i={2} style={{ marginTop: 14 }} tex>
                <BarChartBlock
                    height={150}
                    data={[
                        { name: '宽松 harness', 得分: 92 },
                        { name: '严格 harness', 得分: 61 },
                        { name: '计入不可回答', 得分: 38 },
                        { name: '剔除不可回答', 得分: 76 },
                    ]}
                    keys={['得分']}
                />
            </Card>
        </Slide>

        <Slide title="遗忘盲区" chapter="03 评测">
            <PageHeader
                i={0}
                eyebrow="Forgetting"
                title="召回与删除是两套不同的器官"
                desc="受监管场景里, 能不能删干净 比 能不能想起来 更硬 —— 而现有榜单几乎不覆盖它."
            />
            <Split i={1} ratio="1.15fr 1fr" align="center">
                <Card className="hxd-fill" tex>
                    <Meter items={[
                        { label: 'LLM Hook 形态 (控制平面某些用例)', value: 93, tone: 'success', hint: '可覆盖' },
                        { label: 'Graphiti (对抗性遗忘)', value: 7, tone: 'danger', hint: '4.4% ~ 7.0%' },
                    ]} />
                </Card>
                <div className="hxd-col">
                    <Callout kind="danger" title="根因是结构性的 (合规红线)">
                        图谱抽象把经历合成事实、丢弃表层形式, 而删除请求恰恰指向那个表层形式 —— 医疗、金融、法务上线前先回答「能不能删干净」.
                    </Callout>
                </div>
            </Split>
            <Card i={2} style={{ marginTop: 14 }} tex>
                <BarChartBlock
                    height={150}
                    data={[
                        { name: '事实召回', 可覆盖: 88, 遗漏: 12 },
                        { name: '关系召回', 可覆盖: 74, 遗漏: 26 },
                        { name: '陈旧条目失效', 可覆盖: 21, 遗漏: 79 },
                        { name: '按请求删除', 可覆盖: 7, 遗漏: 93 },
                    ]}
                    keys={['可覆盖', '遗漏']}
                />
            </Card>
        </Slide>

        <Slide title="受控实验" chapter="03 评测">
            <PageHeader
                i={0}
                eyebrow="Controlled Study"
                title="只换记忆层, 变量压到只剩一个"
                desc="完全相同的 Agent 循环、相同的开源权重答案模型、相同的 judge."
            />
            <div className="hxd-row" style={{ marginTop: 16 }}>
                <Card i={1} className="hxd-fill"><Stat label="结构化存储" value={73.6} unit="%" hint="每答对一题约 27K token" /></Card>
                <Card i={2} className="hxd-fill"><Stat label="策展文件" value={44.9} unit="%" hint="每答对一题约 665K token" /></Card>
                <Card i={3} className="hxd-fill"><Stat label="弃答维度" value={'+11.1'} hint="策展文件反而赢了: 88.9% vs 77.8%" /></Card>
            </div>
            <Callout i={4} kind="info" title="反例比结论更值钱">
                准确率差 28.7 分、token 差 24.6 倍, 但材料更少时模型更容易承认「历史里没说」. 更怕幻觉的场景, 文件式反而值得考虑.
            </Callout>
        </Slide>

        <Slide title="决策树" chapter="04 落地">
            <PageHeader
                i={0}
                eyebrow="Decision"
                title="两个问题就能定范式"
                desc="先问知识来自对话还是文档, 再问要不要知道「当时什么是真的」."
            />
            <Steps i={1} steps={[
                { title: '知识主要来自文档 / 业务数据', desc: '走知识图谱 / GraphRAG 路线 —— 前提是没有强合规需求.' },
                { title: '知识来自用户对话, 且事实会随时间改变', desc: '选 Zep / Graphiti, 并接受图数据库的运维成本与写延迟.' },
                { title: '只需知道现在什么是对的', desc: '按技术栈绑定与合规要求选: 云原生 / 长时程自主状态 / 可读可审计.' },
            ]} />
            <div className="hxd-row" style={{ marginTop: 20 }}>
                <Card i={3} className="hxd-fill"><Bullets marker="dot" items={[{ text: '云原生优先', strong: true }, 'AWS AgentCore Memory']} /></Card>
                <Card i={4} className="hxd-fill"><Bullets marker="dot" items={[{ text: '要长时程自主状态', strong: true }, 'Letta (自编辑记忆)']} /></Card>
                <Card i={5} className="hxd-fill"><Bullets marker="dot" items={[{ text: '要可读可审计', strong: true }, '文件即真相型']} /></Card>
            </div>
        </Slide>

        <Slide title="四个必补动作" chapter="04 落地">
            <PageHeader
                i={0}
                eyebrow="Must-Do"
                title="无论选谁, 这四件事都要补做"
                desc="它们不在任何框架的开箱能力里, 而成本远低于换框架."
            />
            <Columns i={1} cols={4}>
                <Card><Badge tone="brand">01</Badge><Divider label="元数据隔离" /><div className="hxd-body">防跨租户合并: 元数据不同的事件永不被合并, 哪怕语义高度相似.</div></Card>
                <Card><Badge tone="brand">02</Badge><Divider label="状态过滤层" /><div className="hxd-body">防幽灵记忆: 同一实体的多个有效版本不应被同时召回.</div></Card>
                <Card><Badge tone="brand">03</Badge><Divider label="独立遗忘测试" /><div className="hxd-body">不要用召回榜单代替遗忘验证 —— 它们测的是两套器官.</div></Card>
                <Card><Badge tone="brand">04</Badge><Divider label="写入侧安全校验" /><div className="hxd-body">记忆写入是特权操作, 不是普通的一次函数调用.</div></Card>
            </Columns>
        </Slide>

        <Slide title="集成形态" chapter="04 落地">
            <PageHeader i={0} eyebrow="Integration" title="三种集成模式, 推荐组合使用" desc="业务逻辑不该接触记忆基础设施 —— 记忆层要可替换、可独立单测." />
            <Split i={1} ratio="1.05fr 0.95fr" align="start">
                <Card>
                    <Bullets marker="num" items={[
                        { text: '两端挂钩 (最推荐)', strong: true },
                        '调用开始注入、调用结束异步持久化, 见右侧代码',
                        { text: 'Hook 驱动 (零侵入)', strong: true },
                        '宿主不改循环, 叠两层: 本地 Markdown + 云端跨设备',
                        { text: '自主工具调用只是补充', strong: true },
                        '模型不会主动记住所有值得记的东西, 它替代不了上面两种',
                    ]} />
                </Card>
                <CodeBlock
                    filename="agent-hooks.ts"
                    language="ts"
                    code={'// 读取: 同步, 只做注入\nctx.on("session/start", () => {\n  ctx.inject(await memory_load(scope));\n});\n\n// 写入: 异步, 绝不阻塞对话\nctx.on("turn/end", ({ data }) => {\n  void memory_save(data);  // fire and forget\n});'}
                />
            </Split>
            <CompareTable
                i={2}
                columns={['模式', '侵入性', '适合']}
                highlight={1}
                rows={[
                    { label: '两端挂钩', values: ['中', '自己掌握对话循环的产品', '语义最准, 可控性最高'] },
                    { label: 'Hook 驱动', values: ['零', '宿主 Agent 不允许改循环', '不改一行宿主代码即可叠加记忆'] },
                ]}
            />
        </Slide>

        <Slide title="记忆投毒" chapter="05 安全">
            <PageHeader
                i={0}
                eyebrow="Security"
                title="从提示注入升级为独立攻击面"
                desc="OWASP 2026 把 Memory & Context Poisoning 单列为 ASI06."
            />
            <Split i={1} ratio="1.25fr 0.75fr" align="start">
                <div className="hxd-col">
                    <Timeline items={[
                        { time: '阶段一', title: '注入', desc: '载荷伪装成事实 / 偏好 / 规则, 经常规抽取管线落库', tone: 'brand' },
                        { time: '阶段二', title: '激活', desc: '后续检索召回该条目, Agent 把它当可信内部状态采信', tone: 'warn' },
                        { time: '副作用', title: '持久化驻留', desc: '跨会话生效, 低可见性、静默漂移', tone: 'default' },
                    ]} />
                </div>
                <div className="hxd-col">
                    <Card><Stat label="InjecMEM → MemoryOS" value={76.6} unit="%" hint="攻击成功率" /></Card>
                    <Card><Stat label="GhostWriter 注入率" value={98} unit="%" hint="近乎普遍" /></Card>
                </div>
            </Split>
            <Callout i={2} kind="danger" title="这已经是真实漏洞 (CVE-2026-41713, CVSS 8.2)">
                Spring AI 的 PromptChatMemoryAdvisor 把输入存进记忆后被模型以非预期方式重新解释. 写入时一切正常, 出事发生在下一次召回.
            </Callout>
        </Slide>

        <Slide title="防护与指标" chapter="05 安全">
            <PageHeader i={0} eyebrow="Mitigation" title="四条防护 + 一个该盯的指标" desc="核心健康指标不是命中率, 而是幽灵率." />
            <Split i={1} ratio="1.1fr 0.9fr" align="start">
                <Card>
                    <Steps steps={[
                        { title: 'Schema 绑定记忆', desc: '不允许自由文本, 用结构化 Schema 约束一切写入与修改.' },
                        { title: '写入即特权操作', desc: '写入 / 修改必须经过权限校验与上下文审核.' },
                        { title: '读取以「数据」角色注入', desc: '检索结果只作数据, 不作指令; 按可信度、时效性、一致性评分.' },
                        { title: '来源与置信度分级', desc: '每条记忆标注来源与信任值.' },
                    ]} />
                </Card>
                <div className="hxd-col">
                    <Gauge value={7} max={100} unit="%" label="健康幽灵率上限" size={200} />
                    <Callout kind="info" title="为什么盯幽灵率">
                        命中率告诉你「想起来了」, 幽灵率告诉你「想起来的可能是错的版本」—— 它更能提前预警失败.
                    </Callout>
                </div>
            </Split>
        </Slide>

        <Slide title="趋势" chapter="06 趋势">
            <PageHeader i={0} eyebrow="Outlook" title="记忆正在变成一层可治理的服务" desc="从「记住更多」转向「控制得更好」—— 核心难点不在存储, 而在选择." />
            <Split i={1} ratio="1fr 1fr" align="start">
                <Card>
                    <Bullets marker="check" items={[
                        { text: '从功能到独立服务层', strong: true },
                        '三个信号同时出现: 厂商出专用策略、安全侧独立成条、按操作或按条计费',
                        { text: '程序记忆走向产品', strong: true },
                        '让 Agent 学习「自己是如何完成某个任务的」并复用流程',
                    ]} />
                </Card>
                <Card tex>
                    <BarChartBlock
                        height={168}
                        data={[
                            { name: '厂商策略', 成熟度: 72 },
                            { name: '安全条目', 成熟度: 64 },
                            { name: '计费模式', 成熟度: 81 },
                            { name: '遗忘语义', 成熟度: 18 },
                        ]}
                        keys={['成熟度']}
                    />
                </Card>
            </Split>
            <CompareTable
                i={2}
                columns={['尚未解决的真问题', '卡在哪', '谁最该关心']}
                highlight={2}
                rows={[
                    { label: '会话身份', values: ['同一用户跨设备 / 跨入口', 'B 端多租户'] },
                    { label: '规模化时间抽象', values: ['长历史压缩后仍可追溯查询', '长时程 Agent'] },
                    { label: '陈旧性遗忘的语义化', values: ['什么算「过期」没有统一定义', '合规与风控, 最被低估'], tone: 'bad' },
                ]}
            />
        </Slide>

        <Slide title="结论" chapter="">
            <Cover
                eyebrow="结论"
                title="先选范式, 再选产品"
                subtitle="然后用「要不要知道当时什么是真的」决定是否上双时间轴 —— 无论选谁, 先把那四个必补动作做完."
            />
            <Quote cite="个人判断, 非事实陈述">
                选型的本质是按记忆类型与失效语义做判断, 而不是按榜单分数排序.
            </Quote>
        </Slide>
    </>
);

export default function AgentMemoryDeck(): React.ReactElement {
    return <>{slides()}</>;
}
