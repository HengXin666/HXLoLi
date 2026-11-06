# Agent Note: 番剧页筛选的三个原生下拉换成复用 Tooltip 定位层的 common/Tooltip/Select

Status: implemented

Decision-ID: anime-custom-select

## Code

- `src/components/common/Tooltip/Select.tsx`

## Problem

番剧页工具栏 (`src/components/anime/Tag/FilterBar.tsx`) 里有 3 个原生 select: 排序 (最近更新 / 首播日期 / 评分 / 名称)、"标签与更多"面板里的首播年份、Bangumi 评分下限. 原生下拉的弹出列表由操作系统画, 在深色站点上是白底系统菜单, 字体、圆角、选中色都和周围的 chip 与面板对不上, 也没法在选项旁放部数. 仓库里已经有一套悬浮层: `src/components/common/Tooltip/index.tsx` 的 Floating (fixed 定位、量完尺寸再显示、上下翻转、视口内夹取、滚动与 resize 时重算), 但它没导出, 只会居中放在上方、带箭头、固定挂在 document.body. 番剧页的年份与评分下拉在模态 dialog 里, 挂到 body 的弹层会落在顶层之下, 被 inert 遮住点不到.

## Decision

扩展现有 Tooltip, 而不是另写下拉的定位. Floating 改名并导出为 `FloatingLayer`, 加 5 个可选参数, 默认值保持 tooltip 原样: `prefer` (先上还是先下, 放不下再翻)、`align` (居中或左缘对齐触发元素)、`arrow` (是否画箭头, 关闭时用 `data-arrow="false"` 在 Tooltip.module.css 里隐藏)、`container` (挂载点)、`role` (外层角色). Tooltip 与 useHoverCard 都改用 FloatingLayer, 行为不变.

在它上面新增 `src/components/common/Tooltip/Select.tsx`: 泛型单选下拉 `Select<T extends string | number>`, 选项可带 hint (右侧淡色附注) 与 disabled. 采用 WAI-ARIA 的 select-only combobox 模式: 触发按钮 `role="combobox"`, 焦点始终留在按钮上, 列表 `role="listbox"`, 当前项用 `aria-activedescendant` 指示. 键盘: 上下方向键移动 (列表关着时打开)、Home / End 到首尾、Enter / 空格选中或打开、Esc 只关列表 (在 dialog 里阻止冒泡, 不会把面板一起关掉)、Tab 关闭并照常移走焦点、输入字符按标签首字跳转. 点外部关闭 (pointerdown 捕获阶段判断按钮与弹层之外). 列表默认最高 280px, 超出滚动, 当前项自动滚入视野. 弹层下方优先、左缘对齐、最小宽度等于按钮宽、不画箭头; 触发元素在 dialog 里时挂进那个 dialog. 选项用 mousedown preventDefault, 不抢按钮焦点. 外壳沿用 Tooltip 的深色半透明卡片与淡入, 选中项为站点粉, 当前项为粉色 14% 底.

FilterBar 的 3 处换成 Select: 排序 (`aria-label="排序"`); 年份 ("不限" + 有记录的年份, hint 为部数, 跟着当前其他条件变); 评分 ("不限" / ≥ 6 / 7 / 8). 年份与评分用可见文字的 id 作 `aria-labelledby`, 外层 label 元素改成 div, 避免点文字再触发一次按钮. FilterBar.module.css 删掉原生 select / option 的样式.

## Alternatives considered

- 只给原生 select 加样式 (appearance: none + 自绘箭头): 改动最小, 移动端保留系统滚轮. 否决是因为弹出的选项列表仍由系统画, 深色站点上还是白底菜单; 也放不了部数 hint.
- 新写一个独立的 Popover 组件, 自带定位: 不碰 Tooltip. 否决是因为仓库里会有两套 fixed 定位与翻转逻辑, 用户明确要求不再造第三套.
- 引入 Radix Select / Headless UI Listbox: 可访问性现成. 否决是因为要新增依赖 (约 10 到 30 KB gzip) 且需要先批; 这里只要单选 + 键盘 + 滚动, 复用现有定位层足够.
- 用 Popover API (`popover` 属性) 进顶层, 免去 container 参数: 新浏览器上更省事. 否决是因为 Safari 17 以下不支持, 而且 Tooltip 现有的定位与淡入都基于 fixed 层, 两种机制混用更难维护.

## Consequences

`src/components/common/Tooltip/index.tsx` 的内部组件名 Floating 变成导出的 FloatingLayer; 仓库里引用 Tooltip 的地方 (首页、番剧详情、声优页) 只用默认导出与 useHoverCard, 不受影响. `src/components/common/Tooltip/Tooltip.module.css`、`src/components/common/Tooltip/Select.module.css`、`src/components/anime/Tag/FilterBar.module.css` 是非 AST 资源, 需人工核对. 其余原生 select (`src/components/anime/AnimeGraphPage/index.tsx` 1 处; `src/hxdeck/Dropdown.tsx` 2 处、`src/hxdeck/ThemeEditor.tsx` 1 处、`src/hxdeck/PptEmbed.tsx` 1 处) 本轮没动, 之后换成同一个 Select. 移动端失去系统滚轮式选择器, 换来与站点一致的列表; 列表项高约 30px, 触屏可点. 下拉开合目前只有 Tooltip 原有的 140ms 淡入, 筛选结果的过渡动画是下一项工作.

2026-11-06 补充: 触发按钮的 `.value` 原先继承 `.trigger` 的 `line-height: 1` 又带 `overflow: hidden` (省略号要用), 中文字形上缘超出 1em 行盒, "按最近更新" 顶部约 5% 被裁. `.value` 改为 `line-height: 1.4` + `padding-block: 1px`, 按钮仍是 32px 高. Select 新增可选 `icon` 参数 (文字左侧, 粉色, 悬停 / 展开时微转), 不传不渲染, 年份与评分下拉外观不变; FilterBar 的排序下拉用 `SORT_ICONS` 按当前排序方式传 svg (最近更新 = 时钟, 首播 = 日历, 评分 = 星, 名称 = 字母序). 否决过: 什么都不做 (裁字是可见缺陷); 把 `.trigger` 整体改 line-height (会连带改所有下拉的垂直对齐, 范围过大); 去掉 overflow: hidden (长标签会撑破按钮, 丢省略号).
