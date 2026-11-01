# Agent Note: 分享番单加表格视图 (按年份密排) 与分享图预览

Status: implemented

Decision-ID: anime-share-table-view-and-preview

## Problem

/anime/share (见 `.agents/notes/implemented/feature/2026-11-01-anime-share-link.md`) 只有封面卡片网格, 一屏只能看到几部到十几部, 番单一长就要一直滚动才能勾完. 用户要求加第二种形态, 学参考站 https://anime-sedai.egoist.dev/ : 按首播年份 (2006, 2007 ...) 一行, 行内是可点击的番名短标签, 不带封面和简介, 一屏几十部以上. 两种模式要共享同一份筛选结果和勾选, 切换不丢状态; 模式要进 URL, 分享出去别人打开仍是同一种模式.

另一个缺口: 分享图 (animeShareExport.makeShareImage -> animeShareImage.renderShareImage) 生成后只能直接下载或复制进剪贴板, 页面上看不到图, 用户不知道带封面 / 不带封面、分成几页会是什么样, 只能下载后再打开确认.

第二轮反馈 (2026-11-02): 表格模式第一版是"年份 + N 部 + 一列描边标签块", 用户要的是参考站那种: 年份后面直接跟番名, 不显示部数, 番名小字连续密排, 没勾偏淡、勾上醒目; 没有首播日期的番实际是"还没播", 分组名应叫"待推出"而不是"未知年份".

约束: 不改 filters.ts / tags.ts / overlay.ts / animeStore.ts, 不改 src/pages/anime/index.tsx / details.tsx, 不新增依赖 (不用表格库), 只用 CSS Modules, 不启动或构建博客.

## Decision

- 视图参数 `view`: 只认 `view=table` 为表格模式, 其余 (含缺省) 为卡片模式. 卡片是默认, 不写参数, 已发出的旧链接行为不变.
- 解析: `viewModeOf(location.search)`; 番单定义与分享者 pick 改为从去掉 `view` 后的查询串解析 (`withoutView`), 所以切视图不会让 baseList / storageKey 重算, 勾选与页内筛选 (React state) 原样保留.
- 切换: 页内筛选区第一行"显示: 卡片 / 表格 (按年份)", 用 `history.replace` 改地址栏 (不新增历史记录), 地址栏可直接复制; "生成我的链接"与"在本页打开"经 `withView` 追加 `view=table`. 视图变了已生成的链接收起, 避免复制到旧模式的链接.
- 表格模式: `yearRowsOf(visible)` 按 `airYearOf` 分组, 年份升序 (同参考站从早到晚), 组内保持当前排序, 无日期的归"未知年份"放最后. 每年一行: 左列年份与部数, 右列 flex-wrap 的番名按钮 (`aria-pressed`, 13px, 单行省略, 最长 22em), 点一下调用与卡片模式同一个 `toggle`, 顶部"我看过 N/M"与进度条照常更新; 链接带 pick 时番名前标"TA". 窄屏年份改为行首横排.
- 布局: 筛选与列表在左, 分享区 (生成链接 / 复制清单 / 分享图预览) 在右侧 360px 吸顶栏, 预览不再压在页面最底; 宽度不足 1000px 时分享区排在筛选之后、列表之前.
- 分享图预览: 新函数组件 `ShareImagePreview` (同文件), 主组件只传图片标题 / 条件 / 分组 / 站点地址 / 内容指纹. 画图逻辑不动, 仍调 `makeShareImage`. 生成后每页 Blob 用 `URL.createObjectURL` 转成 blob: URL, 以 `<img>` 显示在分享区; 版本切换"带封面 / 不带封面"两个版本分别缓存, 切到未生成的版本时自动生成. 多页时 `page` 下标翻页, 显示"第 i / n 页"与当前页像素尺寸. 预览右侧 (窄屏在下方) 是"下载 (全部页)"与"复制当前页"; 下载复用 `downloadImages`, 复制复用 `copyImage`.
- object URL 生命周期: 同一版本重新生成先 revoke 旧 URL; 图片内容指纹 (标题 + 排序 + 番单条件 + 入图 id 列表) 变化时两个版本全部 revoke 并清空, 同时递增 job 号作废进行中的生成; 组件卸载时全部 revoke.
- 密度: 桌面 1200px 容器下每行约 8 到 10 个番名, 标签行高约 24px, 1080p 一屏约 100 到 150 部 (卡片模式约 10 到 20 部).

## Alternatives considered

- 什么都不做: 最强理由是卡片网格已能完成勾选与分享, 封面辨识度高, 少一种模式就少一份维护和测试面. 否决: 番单几十上百部时卡片要滚很多屏, 用户明确要求参考站那种高密度按年份形态, 这是勾选效率问题, 不是样式偏好.
- 复用现有方案 (把卡片网格缩小、去掉封面, 或沿用 `groupRecords` 的分组): 最强理由是不新增渲染分支, groupRecords 已有分组与标题. 否决: groupRecords 的分组随排序键变化 (按更新 / 评分排序时不是年份), 无法保证"按年份一行"; 只缩小卡片仍是竖排块, 密度上不去. 改为同一份 visible 与同一个 toggle 上加一个按年份的渲染分支, 数据与状态完全复用.
- 视图状态存 localStorage: 刷新可保留. 否决: 分享出去对方打不开同一模式, 不满足"链接带模式".
- 预览什么都不做, 继续只给下载 / 复制: 最强理由是零代码, 下载后系统看图器也能看. 否决: 每次确认效果都要下载打开, 多页与带不带封面要来回试, 用户明确要求页内预览.
- 预览复用现有方案 (下载链接或把 canvas 直接插进页面): 最强理由是 renderShareImage 内部已有画布, 插 canvas 不用再转 URL. 否决: renderShareImage 只对外返回 PNG Blob, 拿 canvas 要改画图模块; Blob 转 blob: URL 给 img 与最终下载 / 复制的是同一份字节, 所见即所得.
- 引入表格 / 虚拟列表库: 否决, 不允许新增依赖; 纯文字标签几百个 DOM 节点不需要虚拟化.

## Consequences

- URL 多一个可选参数 `view`, 与番剧库筛选参数不冲突 (parseFilters 不认它); 本地勾选键不含它, 两种模式看到的是同一份勾选.
- 表格模式不显示封面与部数, 辨识靠番名; 鼠标悬停 title 显示原名与站长状态. "待推出"按"没有首播日期"判断, 数据里日期缺失但其实已播的番也会落进这一组.
- 预览常驻内存: 两个版本的 PNG Blob 各保留一份直到内容变化或离开页面; 封面版几百部时可能有数 MB.
- 复制只复制当前页 (剪贴板一次一张图), 下载一次下全部页.
- 原先分享区的"图片带封面"复选框与"复制图片 / 下载图片"按钮由预览区的版本切换与按钮取代; `.switch` 样式保留未删.
- 同一组件也挂在番剧库 /anime 的分享弹窗里 (见 `.agents/notes/implemented/feature/2026-11-01-anime-share-on-index.md`): 弹窗里视图模式只存组件 state, 不写 URL.
- `.module.css` 为非 AST 资源, 需人工核对.

## Code

- `src/components/anime/AnimeShareBoard.tsx`
