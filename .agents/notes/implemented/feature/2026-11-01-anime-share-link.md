# Agent Note: 番单分享改为"可交互页面 + 链接传播"

Status: implemented

Decision-ID: anime-share-link

## Problem

用户原话: "你这个分享页面里面为什么不能进行筛选? 而且你根本就没有按照我之前那个网址的那种状态, 它本身应该是可交互的. 就是说我分享之后, 它可以生成 1 个链接, 然后大家就可以进来填写."

参考站 https://anime-sedai.egoist.dev/ 的核心不是出图, 而是: 每部番是可点击条目, 点一下标记"看过"; 顶部实时"我看过 N/239 部"; 有年份范围筛选; 生成链接, 别人打开能看到这份番单、自己勾选、再生成自己的版本.

此前的 /anime/share (2026-11-01-anime-share-image-and-list.md) 只读 URL 筛选出 PNG 与文本清单: 页面上不能筛选, 不能勾选, 分享出去的是一张静态图. 约束: 纯静态站点, 没有后端, 状态只能在 URL 与 localStorage; 不调 Bangumi API; 不新增依赖; 不改 filters.ts / tags.ts / overlay.ts, index.tsx 只动分享面板挂载点 (本次连挂载点也没动).

## Decision

链接格式: `/anime/share?<番剧库同款筛选>&pick=<压缩 id>`.

- 筛选部分直接是 `serializeFilters` 的输出, 与 /anime 的查询串同一套 (status / tag / match / year / score / q / sort). 好处: 番剧库地址栏里的条件原样搬过来就是番单定义; 番剧库新增筛选维度时分享链接自动跟上; 链接人类可读.
- `pick` 是分享者勾了"我看过"的 Bangumi subject id 集合. 编码 (`encodeIds`): 去重升序 -> 相邻差分 -> 每个差值写成小端 5 bit 一组的变长整数, 每组 1 个 base64url 字符 (`0-9a-zA-Z-_`, 第 6 bit 表示"后面还有"), 前缀版本号 `1`. 差 < 32 占 1 字符, < 1024 占 2, < 32768 占 3. 不需要分隔符, 也不会被 URL 转义. 实测 (站点 CDN 上的真实 ANiMeRecord.json, 177 部, 其中看过 148 部): 全部看过 148 部的 pick 402 字符, Cloudflare 整条链接 456 字符; 勾 30 部 pick 92 字符; 按真实 id 分布外推 300 部约 713、500 部约 1103 字符. 同样 148 部用逗号分隔十进制约 1000 字符. 解码另兼容手写的 `pick=12,345,6789`.
- 不用 `/anime?pick=`: 番剧库是站长的筛选视图, 带勾选会让同一个页面有两种语义, 也要改 index.tsx; 不用单个不透明 `?s=`: 筛选部分本来就短且可读, 混进压缩串就没法手改.

分享页 (`src/pages/anime/share.tsx` 外壳 + `src/components/anime/AnimeShareBoard.tsx`):

- 番单 = `applyFilters(records, URL 筛选)`, M = 番单部数, 固定不随页内筛选变化. 顶部吸顶显示"我看过 N/M 部"与进度条; 链接带 pick 时另显示"TA 看过 K 部 · 我们都看过 Z 部", 条目上标"TA 看过".
- 每部是一个按钮 (封面 + 名称 + 年份), 点一下切换"我看过". 没用 PosterCard: 它整卡是详情链接加标签按钮, 套不进"点一下切换"的语义.
- 页内筛选只改显示: 年份范围 (近 5 / 10 / 15 年 / 全部)、勾选状态 (全部 / 我看过 / 我没看过 / TA 看过 / 我们都看过)、站长观看状态、标签 (含全部 / 任一命中切换)、搜索、排序. 状态 / 标签 / 搜索 / 排序复用 `AnimeFilters` + `applyFilters` + `facetCounts` + `groupRecords` + `STATUS_OPTIONS` + `SORT_OPTIONS`; 年份范围与勾选状态是分享页专属, 写在 `src/components/anime/animeShareLink.ts`, 不进 filters.ts.
- 批量操作: 全选当前、取消当前、照抄 TA 的勾选、清空我的、重置筛选.
- 勾选存 localStorage, 键 `<baseUrl>anime/share:v1:<番单标识>`, 番单标识 = 筛选去掉排序后的查询串做 FNV-1a 哈希 (`listKeyOf`). 与 pick 无关: 同一份番单无论谁转发, 打开的人看到的都是自己那份勾选; 不同番单互不污染. 存的值也是 `encodeIds` 串.
- "生成我的链接" = 原番单筛选 + 我在番单内的勾选重新编码, 自动复制并显示链接与字符数. 勾选一变, 旧链接收起.
- 附带"复制图片 / 下载图片": 复用 animeShareImage.ts; 有勾选画"我看过的番 · N/M", 否则画当前显示的部分. 复制图片用 ClipboardItem, 浏览器不支持时隐藏按钮. 下载 / 复制出口在 `src/components/anime/animeShareExport.ts`.

番剧库 /anime 的分享弹窗: 2026-11-02 起直接挂本组件 (`embedded`), 原 AnimeSharePanel 已删除, 见 `.agents/notes/implemented/feature/2026-11-01-anime-share-on-index.md`.

## Alternatives considered

- 什么都不做: 最强理由是现有方案已经能出图出清单, 不加代码就不加风险, 番剧库本身也能用 URL 分享筛选结果. 否决: 用户明确指出分享页不能筛选、不可交互, 别人拿到的是一张图, 不能"进来填写", 与参考站的核心玩法 (勾选 + 链接传播) 完全不同.
- 复用现有出图方案 (只在 /anime/share 加筛选控件, 继续以 PNG 为分享物): 最强理由是 animeShareImage.ts 已经处理好封面 CORS、分页与画布上限, 图片在微信 / QQ 等不便点链接的场景传播更顺. 否决: 图片是终点, 收到的人不能勾选也不能生成自己的版本, 正是用户要推翻的形态; 出图保留为分享页和番剧库面板的附带功能.
- 把番单冻结成显式 id 列表放进 URL (不用筛选): 番单不随站长数据变化. 否决: 全库几百部时链接是 pick 的两倍长, 且站长新看的番进不了老链接; 筛选定义 + 显式 pick 两部分已覆盖需求.
- `/anime?pick=` 挂在番剧库上: 少一个路由. 否决: 要改 index.tsx 的列表逻辑 (不允许), 且番剧库是站长自己的视图, 混入访客勾选语义不清.
- lz-string / 逗号分隔十进制 / 排序差分 + base36 加分隔符: 前者是新依赖 (不允许); 逗号分隔 300 部约 2090 字符超标; base36 + 分隔符约每部 4 字符, 变长 base64url 约 2.4 字符.
- 访客勾选写回 pick 后直接替换地址栏: 刷新不丢. 否决: 会改掉别人发来的原链接, 失去"TA 看过"对照; 改用 localStorage, 由"生成我的链接"显式产出新链接.

## Consequences

- 链接里的番单是"筛选定义", 站长数据更新后同一链接的 M 会变; pick 里已不在番单内的 id 被忽略 (不计入 N/M, 不进新链接).
- 访客勾选只在本浏览器; 换设备要靠自己生成的链接. 存储键版本号 v1, 编码前缀 `1`, 以后换方案可兼容.
- 进度栏用 `position: sticky` 吸顶, 依赖 `--ifm-navbar-height`.
- `npx tsc --noEmit` 由用户在仓库内运行: 10 条错误与 `scripts/quality-baseline.json` 一一对应, 新增 0. `.module.css` 为非 AST 资源, 需人工核对.
- localStorage 键的 baseUrl 统一补尾斜杠 (GitHub Pages 的 baseUrl 为 `/HXLoLi`).
- 编码器做过往返测试 (100 / 239 / 300 / 500 / 800 部随机 id 全部无损, 非法输入返回空).

## Code

- `src/pages/anime/share.tsx`
