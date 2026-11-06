# Agent Note: おすすめ (susume) 杂志特辑式重构 + Stage 舞台组件

Status: implemented

Decision-ID: anime-susume-neko-redesign

## Code

- `src/components/anime/stage/Stage.tsx`
- `src/components/anime/susume/SusumeCategoryPage.tsx`
- `src/pages/anime/susume/index.tsx`

## Problem

susume 推荐的三个分类页、总览页和 /anime 顶部入口卡前后改了两版 (推荐板 + 探头猫娘, 之后是 ui 原子组件拼的 bento), 站长评价是 "没有设计和布局": 版面像后台列表, 没有视觉主角, 和博客的粉色 + 猫娘主题脱节, 动效只有淡入. 同批的 kyara 角色配图页用的是同一套 ui 原子组件, 问题一样. 另外, 标注目录 (HXLoLi-ANiMe/data/labels) 现在是空的, 三个分类都是 0 部, 所以空态本身也要像一个设计好的页面.

## Decision

1. 旧实现全部删除: `SusumePage.tsx` / `SusumeEntryCards.tsx` / `Susume.module.css`. 新代码放进 `src/components/anime/susume/` (数据 hook `useSusume.ts`, 共用小件 `parts.tsx`, 分类页 / 总览页 / 入口带, 样式 `susume.module.css`). 四个路由文件只负责转发.
2. 新增共用舞台 `src/components/anime/stage/` (`Stage.tsx` + `stage.module.css`), susume 和 kyara 共用:
   - 全幅 Hero: 倾斜 -9° 的封面墙, 多行左右交替缓慢平移 (CSS keyframes, 70s 起), 滚动时视差.
   - 左侧白雾或夜色遮罩上压圆体超大标题, 逐字弹簧弹入, 下面是马克笔式主色高亮条.
   - 罗马音副标, 统计数字滚动到目标值; 探头猫娘 (`neko_left.png`) 从舞台右下角探出, 旁边是台词气泡.
   - 另有描边大号名次数字 `Numeral` 和区块标题 `StageHead` (日文小标 + 圆体大标题).
3. 分类页: 粘性三分类切换条 (layoutId 滑块, 分类色) → 第 1 名特写卡 (封面模糊铺底, 竖封面随指针 3D 倾斜, 描边 01, 观后感做成大引号 pull-quote, CTA) + 第 2 / 3 名横卡 → 第 4 名起排行榜 (描边大数字压在海报下, 悬浮海报上浮倾斜、观后感从底部升起) → 收录口径.
4. 空态: 三个虚线空位 (01 / 02 / 03, 第一位带皇冠) 加上墙说明, 不显示假数据.
5. 总览页: 封面墙 Hero → 三个分类大横幅 (水印编号, 分类名, 前三名, 右侧封面扇悬浮摊开) → 评分最高的六部. /anime 入口带: 渐变「站长推荐」头卡 + 三个分类卡 (描边编号 + 名字 + 数量 + 小封面扇).
6. 颜色只取 `--hx-*` 令牌, 分类色经 `--stage-accent` 注入; `prefers-reduced-motion` 时封面墙不滚、标题不逐字、数字不滚、卡片直接出现.
7. Hero 封面墙: 分类自己的推荐够 4 部用推荐, 否则用看过的高分番 (纯装饰, 不代表分类内容).

## Alternatives considered

- 什么都不做: 最强理由是上一版三态齐全、接口冻结. 否决: 站长明确要求删掉重做, 版面和主题都不成立.
- 在 ui 原子组件 (`src/components/anime/ui`) 上继续改样式: 改动最小. 否决: 那套组件的结构本身就是 "标题 + pill + 卡片网格" 的后台式拼装, 换皮解决不了没有视觉主角的问题; kyara 剩下的面板和按钮仍在用它.
- 标注为空时用 ANiMeTags.json 的自动推断填满分类: 页面更热闹. 否决: 2026-11-04 起站长决定自动上墙不参与渲染, 推荐只认人工标注.
- 纯 CSS 实现全部动效: 少一层依赖. 否决: 逐字弹簧、数字滚动、3D 指针倾斜、layoutId 滑块用 framer-motion 更稳, 项目已经依赖它.

## Consequences

`src/components/anime/kyara/` 的 Hero 和区块标题已换成 Stage, 画廊改成按序号的等高行布局; 本地模式面板、导入弹窗、灯箱仍用 ui 原子组件, 见 `2026-11-06-anime-kyara-pages.md`. `src/css/custom.css` 是非 AST 资源, 走 resource-review. 标注目录有数据之后, 分类页自动从空推荐板切到排行版面, 不需要改代码.

## 第二版 (2026-11-06 14:30): 改成首页语汇

用户看了第一版截图: "ai 味道太多", 圈出了点阵底纹、发光小圆点、马克笔大标题、舞台里探出身子的猫娘、皇冠, 以及两侧立绘与正文之间没有边界. 要求参考首页 (`src/components/home/page.module.css`) 和博客文章页 (正文与侧栏有分界).

决定:
1. `stage/Stage.tsx` 保留导出名 (StageHero / StageHead / Rise / Numeral / CountUp / useCalm / accentStyle), 内部全部换成首页语汇: 11px 小标 + 28px 标题 + 等宽罗马音 + 13px 说明, 统计是 1px 分格 (首页左栏 stats), 节标题 = 首页 `.head` (20px + 右侧灰字). 封面墙、视差、逐字弹入、数字滚动、舞台猫娘全部删除; `backdrop` / `say` 字段保留但不再渲染.
2. 新增 `paperClass`: 正文是一张 max-width 1180 的纸, 左右 1px 边线 + 实底 + z-index 2, 两侧探头立绘留在纸外. ≤996 时去掉边线 (那时立绘本来不渲染).
3. 推荐分类页: 页头右侧是首页「最近更新」那种分段切换 (分类色只用在小圆点上) → 前三 (三张细线卡: 封面 112px + 名次 / 标题 / 等宽元信息 / 一句观后感) → 第 4 名起 (首页最近更新的细线列表行) → 收录口径.
4. 总览: 每个分类一节 (节标题 + 全部 →) + 首页「在追的番」式封面行; 最后是评分最高六部.
5. `/anime` 入口: 一行 chips (首页标签行), 不再是三张大卡.
6. kyara: 页头随 Stage 变朴素, 主容器加 `paperClass`; 空态说明 (`KyaraGuide`) 去掉虚线 + 光晕 + 描边大数字, 改为细线面板.

被否决: 只删猫娘和点阵, 保留封面墙 Hero (改动小, 但用户点名的是整体 "ai 味", 封面墙和光晕正是来源).
