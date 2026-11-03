# 番剧页重构方案: 筛选 / 标签 / 展示 (待确认, 未改代码)

日期: 2026-11-01. 状态: **方案稿, 等你看过再动手**. 本轮没有改任何源码, 只写了这一个文件.

---

## 0. 定位 (全部读盘确认)

| 项 | 位置 | 说明 |
| --- | --- | --- |
| 路由 | `/anime` → `HXLoLi/src/pages/anime/index.tsx` | Docusaurus 页面路由, 本方案的对象 |
| 同目录其它路由 | `src/pages/anime/details.tsx` (`/anime/details?id=`) · `cv.tsx` · `graph.tsx` | 不在本次范围; details 与首页的标签链接 `/anime?tag=X` 必须继续可用 |
| 番剧数据 | `HXLoLi-ANiMe/data/ANiMeRecord.json` (约 12.5 MB) | 运行时经 CDN (tag 见 `HXLoLi/data/animeCdnVersion.ts`) 由 `src/utils/anime/animeStore.ts#useAnimeRecords` 拉取; 本仓库只读 |
| 数据类型 | `src/utils/anime/types.ts` | `anime_data` (id = bgm subject id, name, name_cn, score, eps, date, short_summary...) + `user_status` (watch_status, watched_eps, last_update, comment, tags) |
| 本站标签表 | `HXLoLi/data/animeTags.ts` | `ANIME_TAG_DEFS` 8 个分类 + 口径; `ANIME_TAGS` 按 id 列分类 |
| 标签合并 | `src/utils/anime/tags.ts` | `getRecordTags` = 表内分类 + `user_status.tags` 去重; `countTags` 计数 |
| 列表页组件 | `src/components/anime/WatchStatusTabs/` · `SortTabs/` · `Tag/TagFilter.tsx` · `Tag/index.tsx` · `AnimeCard/` | 页签都套 `src/components/common/Tabs` |
| 其它用到标签的地方 | `src/components/home/HomeAnime.tsx` (首页"在追的番") · `details.tsx` | 只读 `getRecordTags` / `countTags`, 本方案改合并规则时它们自动跟随 |
| 现有决策 note | `.agents/notes/implemented/feature/2026-11-01-anime-tags-and-home-anime.md` | 本次改动需与它配对改写 |
| 设计语言 | `components/HX-UI` (`src/tokens.css` 为唯一色源) + `HXLoLis/.agents/skills/hx-ui-system` (规范 `references/visual-rules.md`, 门禁 `scripts/check-ui-rules.ts`) | 注: 规范在**父目录** `HXLoLis/.agents/skills/hx-ui-system`, `HXLoLi/.agents/skills/` 下没有这个目录 |

数据现状 (2026-11-01 读 ANiMeRecord.json): 共 160 部, 看过 134 / 在看 13 / 搁置 10 / 想看 2 / 抛弃 1. `last_update` 为带 `+08:00` 的 ISO 串; `anime_data.date` 为 `YYYY-MM-DD`, 2 部为空. 写了 comment 的只有 2 部.

---

## 1. 筛选: 现在筛什么 / 漏了什么 / 新规则

### 1.1 现在的逻辑 (`src/pages/anime/index.tsx`)

1. 观看状态单选 (想看 / 在看 / 看过 / 搁置 / 抛弃 / 全部), 存 localStorage, 默认"在看".
2. 标签单选, 存 URL `?tag=`. URL 带 tag 时用 effect 把状态强制改成"全部".
3. 排序两种: 首播日期 / 最近更新. 两种都按"季度"分组.
4. 季度函数 `getYearAndQuarter`: 1-3 月 → 春, 4-6 → 夏, 7-9 → 秋, 10-12 → 冬.

### 1.2 漏了 / 错了什么

| # | 问题 | 后果 |
| --- | --- | --- |
| F1 | **季度标错一季**. 番剧季度约定 1 月冬 / 4 月春 / 7 月夏 / 10 月秋 | 每个分组标题都错. 例: Fate/strange Fake (首播 2026-01-03) 现在显示在"2026 春", 应为"2026 冬" |
| F2 | 标签只能单选 | "萝莉 + 日常" 这种组合查不了 (实际有 12 部) |
| F3 | 标签计数只看状态, 不看已选条件; 状态页签没有部数 | 点下去可能是空页; 不知道每个状态有几部 |
| F4 | 没有搜索 | 160 部只能滚 |
| F5 | 没有按评分排 | 想挑高分番做不到 |
| F6 | 状态在 localStorage、标签在 URL, 两套来源; 带 tag 进来时把"全部"**写进 localStorage** | 从首页点一次"萝莉", 之后正常进 `/anime` 也停在"全部"; 分享链接带不上状态 |
| F7 | 没有标签的番找不出来 | 3 部 (白沙的水族馆、魔女之旅、工作细胞BLACK) 永远不出现在任何标签下, 也没法发现要补 |
| F8 | 空结果时文案写死"这个状态下没有...", 加载中显示一张"とあるアニメ"假卡 | 假卡像真数据; 空态没有出路 |

### 1.3 新规则

筛选状态收成一个对象, **URL 是唯一事实源**, 写在新的纯逻辑模块 `src/utils/anime/filter.ts`:

```ts
type AnimeFilter = {
  status: 0 | WatchStatus;          // 0 = 全部
  tags: string[];                    // 多选, 取交集 (AND)
  q: string;                         // 搜索词
  sort: "update" | "air" | "score";
  view: "timeline" | "grid";
};
// URL: /anime?status=watching&tag=萝莉&tag=日常&q=longwang&sort=score&view=grid
```

- **过滤顺序**: 状态 → 搜索 → 标签交集.
- **搜索**: 命中 `name_cn` 或 `name`; 中文名支持拼音全拼与首字母 (复用 `src/utils/tags/pinyin.ts#matchesTagQuery`). 输入用 `useDeferredValue`; `/` 聚焦, `Esc` 清空.
- **分面计数**: 每个标签旁的数字 = "在当前状态 + 搜索 + 已选标签基础上再加这个标签剩几部"; 为 0 的置灰不可点 (已选的除外). 状态页签的数字 = 当前搜索 + 标签条件下各状态的部数. 保证点任何可点控件都不会落到空页 (修 F2, F3).
- **"未分类"伪标签**: 命中合并后没有任何标签的番, 放在标签条末尾 (修 F7).
- **排序**: 最近更新 (`last_update` 降序) / 首播 (`date` 降序, 无日期最后) / 评分 (`score` 降序, 同分按最近更新).
- **分组**: 日期类排序按季度分组, 改为 1-3 冬 / 4-6 春 / 7-9 夏 / 10-12 秋, 同年内 秋 > 夏 > 春 > 冬; 组标题带部数; 无日期归"未知日期"并排最后. 评分排序按分数段分组 (9 分以上 / 8 / 7 / 6 / 6 分以下) (修 F1, F5).
- **默认值** (修 F6):
  - URL 有 `status` 就用它; 没有时: 带了 `tag` 或 `q` → 全部; 否则用 localStorage 里用户**亲手**选过的状态; 再没有 → 在看.
  - `sort` / `view`: URL > localStorage > (最近更新, 时间线).
  - 只有用户亲手点控件才写 localStorage; 从链接带进来的条件不改偏好.
  - 每次改动用 `history.replace` 写回 URL (不堆返回栈), 旧链接 `/anime?tag=萝莉` 原样兼容.
- **空态 / 加载态** (修 F8): 加载中显示骨架; 空结果显示说明 + 快捷操作 ("清除标签" / "清除搜索" / "看全部状态", 只列有意义的).

---

## 2. 标签: 重看结果与前后对照

### 2.1 现状统计 (合并后, 全部状态)

本站分类 (`ANIME_TAG_DEFS`): 战斗 41 · 异世界 35 · 萝莉 34 · 日常 30 · 搞笑 29 · 恋爱 17 · 悬疑 11 · 催泪 6.
随手标签 (`user_status.tags`, 只有 9 部写了): 异世界 ×5、萝莉 ×3、日常 ×1 (都与表内重复, 已被去重); 百合 ×1、舰娘 ×1 (都在 碧蓝航线：微速前行！); 妹控 ×1、纯爱 ×1 (都在 晚安sex).
覆盖: 112 部 1 个标签, 41 部 2 个, 4 部 3 个以上, 3 部没有.
一致性: 按作品系列核对, 续作之间标签一致 (唯一差异是 碧蓝航线 两部作品类型本来就不同, 合理).

### 2.2 改动清单

| 动作 | 标签 | 现在 | 改为 | 理由 |
| --- | --- | --- | --- | --- |
| 合并 | 纯爱 → 恋爱 | 筛选条单独一个"纯爱 1" | 作为"恋爱"的别名, 不再单独出现; 晚安sex 本来就在"恋爱"下, 恋爱仍为 17 | 同义; 单独一个 1 部的胶囊只占位 |
| 删除 (只在本站展示层隐藏) | 舰娘 | "舰娘 1" | 不出现在筛选条与卡片上 | 这是题材 / 系列标记不是类型, 而且同系列的 碧蓝航线 THE ANIMATION 没有它, 筛出来不成一类 |
| 保留 (随手组) | 百合、妹控 | 与分类混在一排 | 归入"随手"分组, 排在分类之后 | 是真实的属性标签, 只是目前各 1 部; 删了会丢信息 |
| 去重 (已生效, 不改) | 异世界 / 萝莉 / 日常 的随手副本 | 已被合并去重 | 不变 | 表内分类为准 |
| 改口径 | 异世界 | "转生 / 穿越 / 召唤到另一个世界, **或以异世界为主舞台**" | "转生 / 穿越 / 召唤到另一个世界 (含游戏世界); 原生奇幻世界不算" | 原口径的后半句会把芙莉莲、迷宫饭、魔女之旅这类原生奇幻全卷进来, 但表里实际没这么标. 核对现有 35 部全部是转生 / 穿越 / 召唤, **数据不用改, 只改口径文字** |
| 补 | 白沙的水族馆 (325281) | 无 | 建议: 日常 | 职场群像为主, 没有贯穿全篇的战斗 / 悬疑 主线. **需你确认** |
| 补 | 工作细胞BLACK (304065) | 无 | 建议: 不进现有分类, 留"未分类" | 本篇 工作细胞 标"搞笑", 但 BLACK 基调严肃, 不符合搞笑口径; 8 个分类里没有合适的. **需你确认** |
| 补 | 魔女之旅 (292970) | 无 | 建议: 留"未分类" | 按新异世界口径不算; 单元剧旅行, 也不符合日常口径. 若你想要, 可新增"奇幻"分类 (见 6.2) |
| 排序 | 萝莉 | 第一 (靠 `i === 0` 硬编码高亮) | 仍第一, 靠 `ANIME_TAG_DEFS` 定义顺序; 去掉硬编码高亮 | 保持重点, 去掉魔法数字 |

### 2.3 落地方式

- `data/animeTags.ts` 新增 `ANIME_TAG_ALIASES: Record<string, string | null>`: `{ 纯爱: "恋爱", 舰娘: null }` (`null` = 隐藏). 改"异世界"口径文字. 补标签按你确认结果加行.
- `src/utils/anime/tags.ts#getRecordTags` 合并时先过别名表. 首页 `HomeAnime`、详情页、列表页都走它, 自动一致.
- 番剧数据仓库 (HXLoLi-ANiMe) 一个字不改; 别名只影响本站展示.

### 2.4 前后对照 (筛选条, 全部状态)

```
之前: [全部标签] 萝莉 34 · 异世界 35 · 日常 30 · 恋爱 17 · 搞笑 29 · 战斗 41 · 催泪 6 · 悬疑 11 · 百合 1 · 舰娘 1 · 妹控 1 · 纯爱 1
之后: 分类  萝莉 34 · 异世界 35 · 日常 30(+1 若确认) · 恋爱 17 · 搞笑 29 · 战斗 41 · 催泪 6 · 悬疑 11
      随手  百合 1 · 妹控 1
            未分类 3 (确认补标后为 2)
```

---

## 3. 展示重构 (按 HX-UI 设计语言)

### 3.1 怎么接 HX-UI

- **不直接装 `@hx/ui` 组件包**: 它的 peer 依赖是 Tailwind 4 + radix-ui 等, 博客是 Docusaurus 3.7 + Tailwind 3.4, 硬接要先迁 Tailwind, 风险远大于一个页面.
- **接令牌层**: HX-UI 的 `tokens.css` 零依赖. 新增 `src/css/hx-ui-scope.css` (经 `docusaurus.config.ts` 的 `customCss` 引入), 内容从 `components/HX-UI/src/tokens.css` 的暗色段拷入, 但**只在 `[data-hx-ui]` 作用域内生效**, 不影响站点其它页面. 唯一的覆盖: `--primary` / `--ring` / `--link` 改为站点主色粉 (`#ff88ff` 系), 保留站点辨识度. 番剧页根节点加 `data-hx-ui`.
- 组件样式写 CSS module, **只引用 `var(--token)` 与 `color-mix(in srgb, var(--x) N%, transparent)`**, N 只取 5/10/15/20/25/30/40/50/60/70/80.
- 照 HX-UI 的形态在本地实现几件小组件 (不拷库源码): SegmentedControl、Badge、Progress、Skeleton、EmptyState.

### 3.2 视觉规则 (逐条对应 `visual-rules.md`)

| 规则 | 本页落法 |
| --- | --- |
| 颜色单一来源 | tsx 里零色值; 旧代码里的 `#2b2b2b`、`rgba(255,136,255,…)`、`white` 全部换 token |
| 层级靠 1px 线 + 半透明底, 不靠阴影 | 卡片: `border 1px var(--border) 50%` + `var(--card) 40%` 底 + `backdrop-filter`; 普通卡片、列表项、封面格**一律无阴影** (现在的卡片有 `box-shadow`, 去掉) |
| hover | 只换边框色 (→ primary 40%) 与底透明度 (40% → 60%), 位移上限 `translateY(-2px)`, 不叠 scale / 阴影 |
| 选中态 | 选择器模式: `border primary 50%` + `bg primary 10%`, 不叠 ring |
| 线条 | 全部 1px (时间线标题现在是 2px 边框, 改 1px) |
| 圆角 | 卡片 / 面板 / 封面格 `--radius + 4px` 档 (xl); 输入框 lg; chip / 标签 / 分段控件 full |
| 字号 | 只用 10–15px 七档; 统计数字 20px+ |
| 网格 | 封面墙 `repeat(auto-fill, minmax(150px, 1fr))`, 不写死列数 |
| 动效 | 进入 `opacity 0→1 + y 8→0`, ≤ 0.5s, stagger 0.04; 只过渡 color / background-color / border-color / transform; 不写 transition-all; `prefers-reduced-motion` 全部关掉 |
| 语义色 | 评分 = warning; 看过 = success; 在看 = primary 10%; 搁置 = warning; 想看 / 抛弃 = muted |

### 3.3 页面结构

```
┌ PageHeader ─ 追番记录 · 160 部 · 最近更新 YYYY-MM-DD ────────────────┐
├ 吸顶工具栏 (navbar 下方, 半透明 + 模糊) ─────────────────────────────┤
│  [全部 160|在看 13|看过 134|想看 2|搁置 10|抛弃 1]  [🔍 搜番名/拼音 /]  │
│                                     [最近更新|首播|评分] [时间线|封面墙]│
│  分类  萝莉 34  异世界 35  日常 30 ...                                  │
│  随手  百合 1  妹控 1      未分类 3                                     │
│  共 12 部 · 萝莉 × · 日常 × · 清空筛选     (有条件时才出现)              │
├ 内容 ───────────────────────────────────────────────────────────────────┤
│  时间线视图: 季度组标题 (带部数) + 时间轴 + 卡片                        │
│  封面墙视图: 季度组标题 + 3:4 封面网格                                  │
└─────────────────────────────────────────────────────────────────────────┘
两侧猫娘保留 (≥1300px 才显示, 沿用现有 .neko-sticky)
```

- **时间线卡片**: 封面 · 标题 (中文名 + 原名) · 右上评分 Badge · 追番进度条 `watched_eps / eps` (看完变 success 色) · "全部"状态下显示状态 Badge · 标签 (点击即加入 / 移出筛选, 已选高亮) · 短评 (有才显示) · 简介 3 行截断 · "详情 →". 现在的"x 话"是 episodes 里本篇条数, 不是进度, 换成进度条.
- **封面墙格子**: 3:4 封面 (封面与标题是详情链接) · 右上评分 · 左上状态 (全部时) · 底部 4px 进度条 · 标题 2 行截断 · 前 3 个标签 (按钮, 不嵌在链接里) + `+N`.
- **窄屏 (≤768px)**: 工具栏贴顶, 标签行横向滑动不换行; 时间线去掉轴线, 卡片封面缩到 96px.
- **键盘与无障碍**: 分段控件 `role="radiogroup"`; 标签 `aria-pressed`; 所有可聚焦元素 1px 可见焦点环; 结果数 `aria-live="polite"`.
- **性能**: `getRecordTags` 每条只算一次 (useMemo 建索引); 列表项 `content-visibility: auto`.

---

## 4. 改动文件清单

| 文件 | 动作 |
| --- | --- |
| `src/utils/anime/filter.ts` | 新增. URL 读写、默认值、季度、排序、分组、分面计数、搜索. 纯函数 |
| `src/components/anime/AnimeListPage/index.tsx` + `.module.css` | 新增. 列表页主体 (原 `pages/anime/index.tsx` 的内容搬来), 让 UI 门禁能只扫这个目录 |
| `src/pages/anime/index.tsx` | 改为薄壳: `<Layout>` + `<AnimeListPage/>` |
| `src/components/anime/AnimeToolbar/index.tsx` + `.module.css` | 新增. 吸顶工具栏 + 分段控件 |
| `src/components/anime/Tag/TagFilter.tsx` | 改. 多选 + 分组 + 分面计数 + 已选分类的口径列表 |
| `src/components/anime/Tag/index.tsx` | 改. 可选 `active` / `onClick` (有 onClick 渲染 button), 颜色换 token |
| `src/components/anime/AnimeCard/index.tsx` + `.module.css` | 改. 新卡片 + 新增 `AnimeTile` (封面墙) + `AnimeSkeleton` |
| `src/utils/anime/tags.ts` | 改. 合并时先过 `ANIME_TAG_ALIASES` |
| `data/animeTags.ts` | 改. 加别名表、改异世界口径、按确认补标签 |
| `src/css/hx-ui-scope.css` + `docusaurus.config.ts` 的 `customCss` | 新增 / 改一行. HX-UI 令牌作用域 |
| `WatchStatusTabs/`、`SortTabs/`、`common/Tabs` | **不动** (保留导出, 列表页不再使用) |
| `.agents/notes/implemented/feature/2026-11-01-anime-page-filter-redesign.md` | 新增 note. 代表文件: `filter.ts`、`AnimeToolbar/index.tsx`、`AnimeListPage/index.tsx` |
| `.agents/notes/implemented/feature/2026-11-01-anime-tags-and-home-anime.md` | 就地改写: 标签条单选 → 多选、别名表、季度修正、口径变更 |

不加任何 npm 依赖. 不启动博客 (仓库红线), 手测由你启动.

---

## 5. 验收判据

数字按 2026-11-01 的 ANiMeRecord.json 计算, 数据更新后以同口径重算.

**筛选**

- [ ] A1 季度: 首播排序下 Fate/strange Fake (2026-01-03) 落在"2026 冬"; 2026 年三组依次为 夏 5 / 春 4 / 冬 5, 季度标题在组内顺序为 秋 > 夏 > 春 > 冬.
- [ ] A2 旧链接: 打开 `/anime?tag=萝莉` → 状态为"全部", 结果 34 部; 之后打开 `/anime` (不带参数) → 状态回到你之前亲手选的 (或"在看"), 不是"全部".
- [ ] A3 多选交集: 萝莉 + 日常 = 12 部; 萝莉 + 异世界 = 6 部.
- [ ] A4 分面: 选中"萝莉"后, 每个标签数字都 ≤ 34, 为 0 的置灰且不可点; 状态页签显示 在看 3 (萝莉中在看的部数).
- [ ] A5 搜索: 输入 `longwang` 命中"龙王的工作！" 1 部; 输入原名片段也能命中; `/` 聚焦, `Esc` 清空.
- [ ] A6 排序: 评分排序第一组为"9 分以上"; 无首播日期的 2 部在首播排序下位于"未知日期"且排最后.
- [ ] A7 URL 可分享: 任意组合筛选后复制地址, 在新标签打开得到同样结果; 浏览器"后退"不会逐次回退每一次筛选.
- [ ] A8 未分类: "未分类"显示 3 部 (补标确认后相应减少).
- [ ] A9 空态 / 加载: 加载中不再出现"とあるアニメ"假卡; 制造空结果 (如搜索 `zzzz`) 出现说明与"清除搜索"按钮.

**标签**

- [ ] B1 筛选条与卡片上不再出现"纯爱""舰娘"; "恋爱"仍为 17.
- [ ] B2 分类组顺序与 `ANIME_TAG_DEFS` 一致, 萝莉第一; 随手组只有 百合、妹控.
- [ ] B3 首页"在追的番"的标签入口与详情页标签同样不再出现 纯爱 / 舰娘 (共用 getRecordTags).
- [ ] B4 HXLoLi-ANiMe 仓库零改动 (`git -C HXLoLi-ANiMe status` 干净).

**展示 / 规范**

- [ ] C1 `node ../.agents/skills/hx-ui-system/scripts/check-ui-rules.ts src/components/anime/AnimeListPage src/components/anime/AnimeToolbar src/components/anime/AnimeCard src/components/anime/Tag` 零 ERROR (raw-button 为 WARN, 标签 chip 用原生 button 并写 `type`, 允许).
- [ ] C2 上述目录的 CSS module 里无 `box-shadow` (工具栏是浮层, 除外, 用 shadow-md 档), 无 `transition: all`, 无 2px 以上边框.
- [ ] C3 375px 宽无横向滚动条; 工具栏吸顶; 标签行可横滑.
- [ ] C4 只用键盘能完成: 切状态、搜索、选 / 取消标签、切视图; 焦点环可见.
- [ ] C5 系统开启"减少动态效果"时没有位移动画.

**工程**

- [ ] D1 `npm run typecheck` 通过.
- [ ] D2 `uv run scripts/redlines/agent_notes.py --diff` 无阻断项 (resource-review 等人工审核项列出由你核对).
- [ ] D3 `WatchStatusTabs` / `SortTabs` / `common/Tabs` 的导出未删, 其它引用处不受影响.

---

## 6. 需要你拍板

1. 补标签: 白沙的水族馆 → 日常? 工作细胞BLACK、魔女之旅 留"未分类"?
2. 要不要新增"奇幻"分类 (原生奇幻世界冒险, 如 芙莉莲 / 迷宫饭 / 魔女之旅)? 加的话我会逐部列出候选给你过目.
3. 舰娘: 隐藏 (本方案) 还是给 碧蓝航线 两部都补上?
4. 主色: 令牌作用域里用站点粉 (本方案) 还是 HX-UI 默认蓝?
5. 默认视图: 时间线 (本方案, 保持现状) 还是封面墙?
